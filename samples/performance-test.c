/*! \file listener.c
 *  \brief The file implements a cROS program for measuring the performance of cROS.
 *
 *  To exit safely press Ctrl-C or 'kill' the process once. If this actions are repeated, the process
 *  will be finished immediately.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#  include <direct.h>

#  define DIR_SEPARATOR_STR "\\"
#else
#  include <unistd.h>
#  include <errno.h>
#  include <signal.h>

#  define DIR_SEPARATOR_STR "/"
#endif

#include "cros.h"

#define ROS_MASTER_PORT 11311
#define ROS_MASTER_ADDRESS "127.0.0.1"

CrosNode *node; //! Pointer to object storing the ROS node. This object includes all the ROS node state variables
static unsigned char exit_flag = 0; //! ROS node loop exit flag. When set to 1 the cRosNodeStart() function exits

int64_t sub_time_stamps[100];
size_t n_sub_time_stamps = 0;

void compute_times(void)
{
  size_t n_stamp;
  printf("Time differences between reception times: ");
  for (n_stamp = 0; n_stamp+1 < n_sub_time_stamps; n_stamp++)
    printf("%fus ", cRosClockTimeStampToUSec(sub_time_stamps[n_stamp + 1] - sub_time_stamps[n_stamp]));
  printf("\n");
}

// This callback will be invoked when the subscriber receives a message
static CallbackResponse callback_sub(cRosMessage *message, void* data_context)
{
  sub_time_stamps[n_sub_time_stamps++] = cRosClockGetTimeStamp();
  cRosMessageField *data_field = cRosMessageGetField(message, "data");
  if (data_field != NULL)
  {
//    ROS_INFO(node, "I heard: [%s]\n", data_field->data.as_string);
  }

  return 0; // 0=success
}

// This callback will be invoked when the service provider receives a service call
static CallbackResponse callback_srv_add_two_ints(cRosMessage *request, cRosMessage *response, void* data_context)
{
  cRosMessageField *a_field = cRosMessageGetField(request, "a");
  cRosMessageField *b_field = cRosMessageGetField(request, "b");
  //cRosMessageFieldsPrint(request, 0);

  if(a_field != NULL && a_field != NULL)
  {
    int64_t a = a_field->data.as_int64;
    int64_t b = b_field->data.as_int64;

    int64_t response_val = a+b;

    cRosMessageField *sum_field = cRosMessageGetField(response, "sum");
    if(sum_field != NULL)
    {
      sum_field->data.as_int64 = response_val;
      ROS_INFO(node,"Service add 2 ints. Arguments: {a: %lld, b: %lld}. Response: %lld\n", (long long)a, (long long)b, (long long)response_val);
    }
  }

  return 0; // 0=success
}

// Ctrl-C-and-'kill' event/signal handler: (this code is no strictly necessary for a simple example and can be removed)
#ifdef _WIN32
// This callback function will be called when the console process receives a CTRL_C_EVENT or
// CTRL_CLOSE_EVENT signal.
// Function set_signal_handler() should be called before calling cRosNodeStart() to set function
// exit_deamon_handler() as the handler of these signals.
// These functions are declared as 'static' to allow the declaration of other (independent) functions with
// the same name in this project.
static BOOL WINAPI exit_deamon_handler(DWORD sig)
{
  BOOL sig_handled;

  switch(sig)
  {
    case CTRL_C_EVENT:
    case CTRL_CLOSE_EVENT:
      SetConsoleCtrlHandler(exit_deamon_handler, FALSE); // Remove the handler
      printf("Signal %u received: exiting safely.\n", sig);
      exit_flag = 1; // Cause the exit of cRosNodeStart loop (safe exit)
      sig_handled = TRUE; // Indicate that this signal is handled by this function
      break;
    default:
      sig_handled = FALSE; // Indicate that this signal is not handled by this functions, so the next handler function of the list will be called
      break;
  }
  return(sig_handled);
}

// Sets the signal handler functions of CTRL_C_EVENT and CTRL_CLOSE_EVENT: exit_deamon_handler
static DWORD set_signal_handler(void)
  {
   DWORD ret;

   if(SetConsoleCtrlHandler(exit_deamon_handler, TRUE))
      ret=0; // Success setting the control handler
   else
     {
      ret=GetLastError();
      printf("Error setting termination signal handler. Error code=%u\n",ret);
     }
   return(ret);
  }
#else
struct sigaction old_int_signal_handler, old_term_signal_handler; //! Structures codifying the original handlers of SIGINT and SIGTERM signals (e.g. used when pressing Ctrl-C for the second time);

// This callback function will be called when the main process receives a SIGINT or
// SIGTERM signal.
// Function set_signal_handler() should be called to set this function as the handler of
// these signals
static void exit_deamon_handler(int sig)
{
  printf("Signal %i received: exiting safely.\n", sig);
  sigaction(SIGINT, &old_int_signal_handler, NULL);
  sigaction(SIGTERM, &old_term_signal_handler, NULL);
  exit_flag = 1; // Indicate the exit of cRosNodeStart loop (safe exit)
}

// Sets the signal handler functions of SIGINT and SIGTERM: exit_deamon_handler
static int set_signal_handler(void)
  {
   int ret;
   struct sigaction act;

   memset (&act, '\0', sizeof(act));

   act.sa_handler = exit_deamon_handler;
   // If the signal handler is invoked while a system call or library function call is blocked,
   // then the we want the call to be automatically restarted after the signal handler returns
   // instead of making the call fail with the error EINTR.
   act.sa_flags=SA_RESTART;
   if(sigaction(SIGINT, &act, &old_int_signal_handler) == 0 && sigaction(SIGTERM, &act,  &old_term_signal_handler) == 0)
      ret=0;
   else
     {
      ret=errno;
      printf("Error setting termination signal handler. errno=%d\n",ret);
     }
   return(ret);
  }
#endif

int main(int argc, char **argv)
{
  char path[4097]; // We need to tell our node where to find the .msg files that we'll be using
  const char *node_name;
  int subidx, pubidx; // Index (identifier) of the subscriber and publisher to be created
  cRosErrCodePack err_cod;
  int op_mode;

  getcwd(path, sizeof(path));
  strncat(path, DIR_SEPARATOR_STR"rosdb", sizeof(path) - strlen(path) - 1);

  printf("PATH ROSDB: %s\n", path);

  printf("Press s for subscriber or p for publisher: ");
  op_mode = getchar();
  if (op_mode == 's')
    node_name = "/node_sub";
  else if (op_mode == 'p')
    node_name = "/node_pub";
  else
  {
    printf("Invalid option");
    return EXIT_FAILURE;
  }

  // Create a new node and tell it to connect to roscore in the usual place
  node = cRosNodeCreate(node_name, "127.0.0.1", ROS_MASTER_ADDRESS, ROS_MASTER_PORT, path);
  if( node == NULL )
  {
    printf("cRosNodeCreate() failed; is this program already being run?");
    return EXIT_FAILURE;
  }

  err_cod = cRosWaitPortOpen(ROS_MASTER_ADDRESS, ROS_MASTER_PORT, 0);
  if(err_cod != CROS_SUCCESS_ERR_PACK)
  {
    cRosPrintErrCodePack(err_cod, "Port %s:%hu cannot be opened: ROS Master does not seems to be running", ROS_MASTER_ADDRESS, ROS_MASTER_PORT);
    return EXIT_FAILURE;
  }

  printf("Node RPCROS port: %i\n", node->rpcros_port);

  // Function exit_deamon_handler() will be called when Ctrl-C is pressed or kill is executed
  set_signal_handler();

  if (op_mode == 's')
  {
    // Create a subscriber to topic /chatter of type "std_msgs/String" and supply a callback for received messages (callback_sub)
    err_cod = cRosApiRegisterSubscriber(node, "/chatter", "std_msgs/String", callback_sub, NULL, NULL, 0, &subidx);
    if (err_cod != CROS_SUCCESS_ERR_PACK)
    {
      cRosPrintErrCodePack(err_cod, "cRosApiRegisterSubscriber() failed; did you run this program one directory above 'rosdb'?");
      cRosNodeDestroy(node);
      return EXIT_FAILURE;
    }

  }
  else if (op_mode == 'p')
  {
    cRosMessage *msg;
    cRosMessageField *data_field;

    // Create a publisher to topic /chatter of type "std_msgs/String" and request that the associated callback be invoked every 100ms (10Hz)
    err_cod = cRosApiRegisterPublisher(node, "/chatter", "std_msgs/String", -1, NULL, NULL, NULL, &pubidx);
    if (err_cod != CROS_SUCCESS_ERR_PACK)
    {
      cRosPrintErrCodePack(err_cod, "cRosApiRegisterPublisher() failed; did you run this program one directory above 'rosdb'?");
      cRosNodeDestroy(node);
      return EXIT_FAILURE;
    }

    msg = cRosApiCreatePublisherMessage(node, pubidx);
    // Popullate msg
    data_field = cRosMessageGetField(msg, "data");
    if (data_field != NULL)
    {
      char buf[1024];
      int pub_count = 0;

      printf("Publishing strings...\n");
      for (pub_count = 0; pub_count < 99; pub_count++)
      {
        snprintf(buf, sizeof(buf), "hello world %d", pub_count);
        cRosMessageSetFieldValueString(data_field, buf);
        err_cod = cRosNodeSendTopicMsg(node, pubidx, msg, 1000);
        if (err_cod == CROS_SUCCESS_ERR_PACK)
        {
          //printf("Published string %d\n", pub_count);
        }
        else
          cRosPrintErrCodePack(err_cod, "cRosNodeSendTopicMsg() failed: message not sent");
      }
      printf("End of string publication.\n");

    }

    cRosMessageFree(msg);
  }

  // Run the main loop until exit_flag is 1
  err_cod = cRosNodeStart( node, CROS_INFINITE_TIMEOUT, &exit_flag );
  if(err_cod != CROS_SUCCESS_ERR_PACK)
    cRosPrintErrCodePack(err_cod, "cRosNodeStart() returned an error code");

  compute_times();

  printf("Unregistering in ROS master\n");
  // Free memory and unregister
  err_cod=cRosNodeDestroy( node );
  if(err_cod != CROS_SUCCESS_ERR_PACK)
  {
    cRosPrintErrCodePack(err_cod, "cRosNodeDestroy() failed; Error unregistering from ROS master");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
