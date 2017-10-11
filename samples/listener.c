/*! \file listener.c
 *  \brief The file is an example of cROS usage implementing a subscriber and service provider, which
 *         Can be used together with talker to prove the operation of message transmissions and service
 *         calls between two nodes.
 *
 *  It creates a subscriber to the topic /chatter. Each time a message of this topic is
 *  published a string is received and the callback function  callback_sub() is executed.
 *  This node also implements a provider of the service /sum. Each time a the service is called
 *  two 64bit integers are received, the callback function callback_srv_add_two_ints is executed,
 *  this function computes the sum of them and this result is sent back to service caller.
 */
#include <cros.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <errno.h>
#include <signal.h>

CrosNode *node; //! Pointer to object storing the ROS node. This object includes all the ROS node state variables
unsigned char exit_flag = 0; //! ROS node loop exit flag. When set to 1 the cRosNodeStart() function exits

// This callback will be invoked when the subscriber receives a message
static CallbackResponse callback_sub(cRosMessage *message, void* data_context)
{
  cRosMessageField *data_field = cRosMessageGetField(message, "data");
  if(data_field)
  {
    ROS_INFO(node, "I heard: [%s]\n", data_field->data.as_string);
  }
  return 0;
}

// This callback will be invoked when the service provider receives a service call
static CallbackResponse callback_srv_add_two_ints(cRosMessage *request, cRosMessage *response, void* context)
{
  cRosMessageField *a_field = cRosMessageGetField(request, "a");
  cRosMessageField *b_field = cRosMessageGetField(request, "b");

  int64_t a = a_field->data.as_int64;
  int64_t b = b_field->data.as_int64;

  int64_t response_val = a+b;

  cRosMessageField *sum_field = cRosMessageGetField(response, "sum");
  sum_field->data.as_int64 = response_val;

  ROS_INFO(node,"Service add 2 ints. Arguments: {a: %lld, b: %lld}. Response: %lld\n", (long long)a, (long long)b, (long long)response_val);

  return 0;
}

// This callback function should be called when the main process receives a SIGINT or
// SIGTERM signal.
// Function set_signal_handler() should be called to set this function as the handler of
// these signals
static void exit_deamon_handler(int sig)
  {
   printf("Signal %i received: exiting.\n", sig);
   exit_flag = 1;
}

// Sets the signal handler functions of SIGINT and SIGTERM: exit_deamon_handler
int set_signal_handler(void)
  {
   int ret;
   struct sigaction act;

   memset (&act, '\0', sizeof(act));

   act.sa_handler = exit_deamon_handler;
   // If the signal handler is invoked while a system call or library function call is blocked,
   // then the we want the call to be automatically restarted after the signal handler returns
   // instead of making the call fail with the error EINTR.
   act.sa_flags=SA_RESTART;
   sigaction(SIGINT, &act, NULL);
   if(sigaction(SIGTERM, &act, NULL) == 0)
      ret=0;
   else
     {
      ret=errno;
      printf("Error setting termination signal handler. errno=%d\n",errno);
     }
   return(ret);
  }

int main(int argc, char **argv)
{
  // We need to tell our node where to find the .msg files that we'll be using
  char path[1024];
  getcwd(path, sizeof(path));
  strncat(path, "/rosdb", sizeof(path));
  // Create a new node and tell it to connect to roscore in the usual place
  node = cRosNodeCreate("/listener", "127.0.0.1", "127.0.0.1", 11311, path, NULL);

  // Create a subscriber and supply a callback for received messages
  if(cRosApiRegisterSubscriber(node, "/chatter", "std_msgs/String", callback_sub, NULL, NULL, 0) < 0)
  {
    printf("cRosApiRegisterSubscriber failed; did you run this program one directory above 'rosdb'?\n");
    return EXIT_FAILURE;
  }

  // Create a service provider and supply a callback for received calls
  if(cRosApiRegisterServiceProvider(node,"/sum","roscpp_tutorials/TwoInts", callback_srv_add_two_ints, NULL, NULL) < 0)
  {
    printf("cRosApiRegisterServiceProvider failed; did you run this program one directory above 'rosdb'?\n");
    return EXIT_FAILURE;
  }

  // Function exit_deamon_handler() will be called when Ctrl-C is pressed or kill is executed
  set_signal_handler();

  // Run the main loop until exit_flag is 1
  cRosNodeStart( node, &exit_flag );

  // Free memory
  cRosNodeDestroy( node );

  return EXIT_SUCCESS;
}
