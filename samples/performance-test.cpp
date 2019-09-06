/*! \file performance-tesp.cpp
 *  \brief This file implements a cROS program for measuring the performance of the cROS library.
 *
 *  To exit safely press Ctrl-C or 'kill' the process once. If this actions are repeated, the process
 *  will be finished immediately.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

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

CrosNode *node; //! Pointer to object storing the ROS node. This object includes all the ROS-node state variables
static unsigned char exit_flag = 0; //! ROS-node loop exit flag. When set to 1 the cRosNodeStart() function exits

double **Time_stamps;
size_t N_measure_exper = 0;
size_t N_measure_rep = 0;
size_t Msg_sizes[]={1, 1, 1024/10, 1024*10, 1024*1024, 1024*1024*100};
size_t Measure_reps[]={1000000, 1000000, 100000, 100000, 1000, 100};

#define TOTAL_MEASURE_EXPERS (sizeof(Measure_reps)/sizeof(size_t))
#define MAX_MEASURE_REPS 1000000
#define MAX_MSG_SIZE (1024*1024*100)

double **allocate_heter_2d_array(size_t n_sub_arrays, const size_t *sub_array_sizes)
{
  double **array_2d;
  size_t n_total_elems, cur_sub_array_ind;

  n_total_elems=0;
  for(cur_sub_array_ind=0;cur_sub_array_ind<n_sub_arrays;cur_sub_array_ind++)
    n_total_elems+=sub_array_sizes[cur_sub_array_ind];

  array_2d = (double **)malloc(sizeof(double *)*n_sub_arrays + sizeof(double)*n_total_elems);
  if(array_2d != NULL)
  {
    double *cur_sub_array_pos;

    cur_sub_array_pos = (double *)(array_2d+n_sub_arrays);
    for(cur_sub_array_ind=0;cur_sub_array_ind<n_sub_arrays;cur_sub_array_ind++)
    {
      array_2d[cur_sub_array_ind] = cur_sub_array_pos;
      cur_sub_array_pos += sub_array_sizes[cur_sub_array_ind];
    }
  }

  return array_2d;
}

void free_heter_2d_array(double **array_2d)
{
  free(array_2d);
}

int print_array(FILE *out_fd, const double *values, size_t n_values)
{
  size_t n_val;
  int tot_chars;

  tot_chars=0;
  for (n_val = 0; n_val < n_values; n_val++)
    tot_chars+=fprintf(out_fd, "%f ", values[n_val]);
  tot_chars+=fprintf(out_fd, "\n");

  return(tot_chars);
}

int store_heter_2d_array(const char *output_file_name, const double * const *array_2d, size_t n_sub_arrays, const size_t *sub_array_sizes)
{
  FILE *fd;
  int ret;

  fd=fopen(output_file_name, "wt");
  if(fd != NULL)
  {
    size_t n_sub_array;
    for (n_sub_array = 0; n_sub_array < n_sub_arrays; n_sub_array++)
      print_array(fd, array_2d[n_sub_array], sub_array_sizes[n_sub_array]);

    fclose(fd);
    ret=0;
  }
  else
    ret=-1;
  return(ret);
}


//// CALLBACKS ////////////////////////////////////////////////////////////////////////////////////////////////////////

// This callback will be invoked when the subscriber receives a message
static CallbackResponse callback_sub(cRosMessage *message, void *data_context)
{
  Time_stamps[N_measure_exper][N_measure_rep] = cRosClockTimeStampToUSec(cRosClockGetTimeStamp());

  cRosMessageField *data_field = cRosMessageGetField(message, "data");
  if (data_field != NULL)
  {
    //printf("Heard %lu\n",strlen(data_field->data.as_string));
    //ROS_INFO(node, "I heard: [%s]\n", data_field->data.as_string);
  }

  N_measure_rep++;

  if(N_measure_rep >= Measure_reps[N_measure_exper])
  {
    N_measure_rep=0;
    N_measure_exper++;
    //printf("*** %lu **\n", N_measure_exper);
  }

  if(N_measure_exper>=TOTAL_MEASURE_EXPERS)
    exit_flag = 1;

  return 0; // 0=success
}

static CallbackResponse callback_pub(cRosMessage *message, void *data_context)
{
  static int pub_count = 0;
  char buf[1024];
  // We need to index into the message structure and then assign to fields
  cRosMessageField *data_field;

  data_field = cRosMessageGetField(message, "data");
  if(data_field)
  {
    snprintf(buf, sizeof(buf), "periodic hello world %d", pub_count);
    if(cRosMessageSetFieldValueString(data_field, buf) == 0)
    {
      ROS_INFO(node, "%s\n", buf);
    }
    pub_count+=10;
  }

  return 0; // 0=success
}

// This callback will be invoked when the service provider receives a service call
static CallbackResponse callback_service_provider(cRosMessage *request, cRosMessage *response, void* data_context)
{
  Time_stamps[N_measure_exper][N_measure_rep++] = cRosClockTimeStampToUSec(cRosClockGetTimeStamp());
  if(N_measure_rep >= Measure_reps[N_measure_exper])
  {
    N_measure_rep=0;
    N_measure_exper++;
  }

  cRosMessageField *name_field = cRosMessageGetField(request, "name");
  if(name_field != NULL)
  {
    cRosMessageField *ok_field = cRosMessageGetField(response, "ok");
    if(ok_field != NULL)
    {
      ok_field->data.as_uint8 = 1;
      //ROS_INFO(node,"Service Resp: %hu\n", (unsigned short)ok_field->data.as_uint8);
    }
  }

  if(N_measure_exper>=TOTAL_MEASURE_EXPERS)
  exit_flag = 1;

  return 0; // 0=success
}

static CallbackResponse callback_service_caller(cRosMessage *request, cRosMessage *response, int call_resp_flag, void* context)
{
  static int call_count = 0;

  if(!call_resp_flag) // Check if this callback function has been called to provide the service call arguments or to collect the response
  {
    cRosMessageField *a_field = cRosMessageGetField(request, "a");
    cRosMessageField *b_field = cRosMessageGetField(request, "b");
    if(a_field != NULL && b_field != NULL)
    {
      a_field->data.as_int64=10;
      b_field->data.as_int64=call_count;
      ROS_INFO(node, "Service add 2 ints call arguments: {a: %lld, b: %lld}\b\n", (long long)a_field->data.as_int64, (long long)b_field->data.as_int64);
    }
  }
  else // Service call response available
  {
    cRosMessageField *sum_field = cRosMessageGetField(response, "sum");
    if(sum_field != NULL)
      ROS_INFO(node, "Service add 2 ints response: %lld (call_count: %i)\n", (long long)sum_field->data.as_int64, call_count++);
  }

  if(call_count > 10) exit_flag=1;
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

cRosErrCodePack run_topic_subscriber(void)
{
  cRosErrCodePack err_cod;
  int subidx; // Index (identifier) of the subscriber to be created

  // Create a subscriber to topic /chatter of type "std_msgs/String" and supply a callback for received messages (callback_sub)
  err_cod = cRosApiRegisterSubscriber(node, "/chatter", "std_msgs/String", callback_sub, NULL, NULL, 0, &subidx);
  if (err_cod == CROS_SUCCESS_ERR_PACK)
  {
    // Run the main loop until exit_flag is 1
    err_cod = cRosNodeStart( node, CROS_INFINITE_TIMEOUT, &exit_flag );
    if(err_cod != CROS_SUCCESS_ERR_PACK)
      cRosPrintErrCodePack(err_cod, "cRosNodeStart() returned an error code");
  }
  else
  {
    cRosPrintErrCodePack(err_cod, "cRosApiRegisterSubscriber() failed.");
  }
  return err_cod;
}

cRosErrCodePack run_service_provider(void)
{
  cRosErrCodePack err_cod;

  // Create a service provider named /bulk of type "controller_manager_msgs/LoadController.srv" and supply a callback for received calls
  err_cod = cRosApiRegisterServiceProvider(node,"/bulk","controller_manager_msgs/LoadController", callback_service_provider, NULL, NULL, NULL);
  if(err_cod == CROS_SUCCESS_ERR_PACK)
  {
    // Run the main loop until exit_flag is 1
    err_cod = cRosNodeStart( node, CROS_INFINITE_TIMEOUT, &exit_flag );
    if(err_cod != CROS_SUCCESS_ERR_PACK)
      cRosPrintErrCodePack(err_cod, "cRosNodeStart() returned an error code");
  }
  else
  {
    cRosPrintErrCodePack(err_cod, "cRosApiRegisterServiceProvider() failed; did you run this program one directory above 'rosdb'?");
  }

  return err_cod;
}

cRosErrCodePack run_topic_publisher(void)
{
  cRosErrCodePack err_cod;
  cRosMessage *msg;
  cRosMessageField *data_field;
  int pubidx; // Index (identifier) of the publisher to be created

  // Create a publisher to topic /chatter of type "std_msgs/String"
  err_cod = cRosApiRegisterPublisher(node, "/chatter", "std_msgs/String", -1, NULL, NULL, NULL, &pubidx); // callback_pub
  if (err_cod != CROS_SUCCESS_ERR_PACK)
  {
    cRosPrintErrCodePack(err_cod, "cRosApiRegisterPublisher() failed; did you run this program one directory above 'rosdb'?");
    cRosNodeDestroy(node);
    return err_cod;
  }

  msg = cRosApiCreatePublisherMessage(node, pubidx);
  // Popullate msg
  data_field = cRosMessageGetField(msg, "data");
  if (data_field != NULL)
  {
    static char buf[MAX_MSG_SIZE+1];
    int pub_count;

    memset(buf,' ',MAX_MSG_SIZE+1);
    for (pub_count = 0; pub_count < TOTAL_MEASURE_EXPERS; pub_count++)
      buf[Msg_sizes[pub_count]]='\0'; // Previously set end of string for all message sizes



    data_field->data.as_string = buf;

    printf("Publishing strings...\n");
    //cRosMessageSetFieldValueString(data_field, buf);
    err_cod = cRosNodeStart( node, 200, &exit_flag );
    for (pub_count = 0; pub_count < TOTAL_MEASURE_EXPERS && err_cod == CROS_SUCCESS_ERR_PACK && exit_flag == 0; pub_count++)
    {
      int rep_count;
      printf("Publishing %lu bytes %lu times\n", strlen(buf),Measure_reps[pub_count]);
      //cRosMessageSetFieldValueString(data_field, buf);
      for(rep_count=0;rep_count<Measure_reps[pub_count] && err_cod == CROS_SUCCESS_ERR_PACK;rep_count++)
      {
        err_cod = cRosNodeSendTopicMsg(node, pubidx, msg, 1000);
        if (err_cod == CROS_SUCCESS_ERR_PACK)
        {
          //printf("Published string %d\n", pub_count);
          //err_cod = cRosNodeStart( node, 400, &exit_flag );
        }
        else
          cRosPrintErrCodePack(err_cod, "cRosNodeSendTopicMsg() failed: message not sent");
      }
      if(pub_count+1 < TOTAL_MEASURE_EXPERS && Msg_sizes[pub_count] < Msg_sizes[pub_count+1])
        buf[Msg_sizes[pub_count]]=' ';
    }
    printf("End of message publication.\n");
    data_field->data.as_string = NULL; // Prevent cRosMessageFree from trying to free the char array

  }
  else
    printf("Error accessing message fields\n");

  cRosMessageFree(msg);

  if (err_cod == CROS_SUCCESS_ERR_PACK)
  {
    err_cod = cRosNodeStart( node, 20000, &exit_flag );
    if(err_cod != CROS_SUCCESS_ERR_PACK)
      cRosPrintErrCodePack(err_cod, "cRosNodeStart() returned an error code");
  }

  return err_cod;
}

cRosErrCodePack run_service_caller(void)
{
  cRosErrCodePack err_cod;
  cRosMessage *msg_req, msg_res;
  cRosMessageField *name_field;
  int calleridx; // Index (identifier) of the service caller to be created


  // Create a service caller named /bulk of type "controller_manager_msgs/LoadController.srv" and request that the associated callback
  err_cod = cRosApiRegisterServiceCaller(node,"/bulk","controller_manager_msgs/LoadController", -1, NULL, NULL, NULL, 1, 1, &calleridx);
  if(err_cod != CROS_SUCCESS_ERR_PACK)
  {
    cRosPrintErrCodePack(err_cod, "cRosApiRegisterServiceCaller() failed; did you run this program one directory above 'rosdb'?");
    cRosNodeDestroy( node );
    return err_cod;
  }


  msg_req = cRosApiCreateServiceCallerRequest(node, calleridx);
  cRosMessageInit(&msg_res);

  name_field = cRosMessageGetField(msg_req, "name");
  if (name_field != NULL)
  {
    static char buf[MAX_MSG_SIZE+1]={0};
    int call_count = 0;

    printf("Calling service...\n");

    err_cod = cRosNodeStart( node, 200, &exit_flag );
    for (call_count = 0; call_count < TOTAL_MEASURE_EXPERS && err_cod == CROS_SUCCESS_ERR_PACK && exit_flag == 0; call_count++)
    {
      int rep_count;
      //snprintf(buf, sizeof(buf), "hello world %d", call_count);
      memset(buf,' ',Msg_sizes[call_count]);

      cRosMessageSetFieldValueString(name_field, buf);

      for(rep_count=0;rep_count<MAX_MEASURE_REPS && err_cod == CROS_SUCCESS_ERR_PACK;rep_count++)
      {
        err_cod = cRosNodeServiceCall(node, calleridx, msg_req, &msg_res, 5000);
        if (err_cod == CROS_SUCCESS_ERR_PACK)
        {
          cRosMessageField *ok_field;
          ok_field = cRosMessageGetField(msg_req, "name");
          printf("Called service %d: %i\n", call_count, ok_field->data.as_uint8);
          //err_cod = cRosNodeStart( node, 1000, &exit_flag );
        }
        else
          cRosPrintErrCodePack(err_cod, "cRosNodeServiceCall() failed: service call not made");
      }
    }

  }
  else
    printf("Error accessing message fields\n");


  cRosMessageFree(msg_req);
  cRosMessageRelease(&msg_res);

  printf("End of service call.\n");

  // Run the main loop until exit_flag is 1
  if (err_cod == CROS_SUCCESS_ERR_PACK)
  {
    err_cod = cRosNodeStart( node, 200, &exit_flag );
    if(err_cod != CROS_SUCCESS_ERR_PACK)
      cRosPrintErrCodePack(err_cod, "cRosNodeStart() returned an error code");
  }

  return err_cod;
}


int main(int argc, char **argv)
{
  char path[4097]; // We need to tell our node where to find the .msg files that it will be using
  const char *node_name;

  cRosErrCodePack err_cod;
  int op_mode;

  if(getcwd(path, sizeof(path)) == NULL)
  {
    perror("Could not get current directory");
    return EXIT_FAILURE;
  }

  strncat(path, DIR_SEPARATOR_STR"rosdb", sizeof(path) - strlen(path) - 1);

  printf("PATH ROSDB: %s\n", path);

  printf("Press s for subscriber, p for publisher, r for service server or c for service client: ");
  op_mode = getchar();
  if (op_mode == 's')
    node_name = "/node_sub";
  else if (op_mode == 'r')
    node_name = "/node_server";
  else if (op_mode == 'p')
    node_name = "/node_pub";
  else if (op_mode == 'c')
    node_name = "/node_caller";
  else
  {
    printf("Invalid option\n");
    return EXIT_FAILURE;
  }

  Time_stamps = allocate_heter_2d_array(TOTAL_MEASURE_EXPERS, Measure_reps);
  if(Time_stamps == NULL)
  {
    printf("Cannot allocate memory for the time stamps\n");
    return EXIT_FAILURE;
  }

  // Create a new node and tell it to connect to roscore in the usual place
  node = cRosNodeCreate(node_name, "127.0.0.1", ROS_MASTER_ADDRESS, ROS_MASTER_PORT, path);
  if( node == NULL )
  {
    printf("cRosNodeCreate() failed; is this program already being run?");
    free_heter_2d_array(Time_stamps);
    return EXIT_FAILURE;
  }

  err_cod = cRosWaitPortOpen(ROS_MASTER_ADDRESS, ROS_MASTER_PORT, 0);
  if(err_cod != CROS_SUCCESS_ERR_PACK)
  {
    cRosPrintErrCodePack(err_cod, "Port %s:%hu cannot be opened: ROS Master does not seems to be running", ROS_MASTER_ADDRESS, ROS_MASTER_PORT);
    free_heter_2d_array(Time_stamps);
    return EXIT_FAILURE;
  }

  printf("Node RPCROS port: %i\n", node->rpcros_port);

  // Function exit_deamon_handler() will be called when Ctrl-C is pressed or kill is executed
  set_signal_handler();

  if (op_mode == 's')
  {
    err_cod = run_topic_subscriber();
  }
  else if (op_mode == 'r')
  {
    err_cod = run_service_provider();
  }
  else if (op_mode == 'p')
  {
    err_cod = run_topic_publisher();
  }
  else if (op_mode == 'c')
  {
    err_cod = run_service_caller();
  }


  printf("Unregistering in ROS master\n");
  // Free memory and unregister
  err_cod=cRosNodeDestroy( node );
  if(err_cod != CROS_SUCCESS_ERR_PACK)
  {
    cRosPrintErrCodePack(err_cod, "cRosNodeDestroy() failed; Error unregistering from ROS master");
    free_heter_2d_array(Time_stamps);
    return EXIT_FAILURE;
  }

  printf("Node end. Current N_measure_exper: %lu N_measure_rep: %lu.\n", N_measure_exper, N_measure_rep);

  if(op_mode == 's' || op_mode == 'r')
  {
    store_heter_2d_array("times.txt", Time_stamps, TOTAL_MEASURE_EXPERS, Measure_reps);
  }

  free_heter_2d_array(Time_stamps);
  return EXIT_SUCCESS;
}
