//
// Created by nico on 26/05/16.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

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

CrosNode *node;
static unsigned char exit_flag = 0; //! ROS-node loop exit flag. When it is set to 1 the cRosNodeStart() function exits

static void printHelp( char *cmd_name )
{
    printf("Usage: %s [OPTION] ... \n", cmd_name);
    printf("Options:\n");
    printf("\t-name  <node_host>       Set the node name (default: /test_node)\n");
    printf("\t-host  <node_host>       Set the node host (default: 127.0.0.1)\n");
    printf("\t-chost <roscore host>    Set the roscore host (default: 127.0.0.1)\n");
    printf("\t-cport <roscore port>    Set the roscore port (default: 11311)\n");
    printf("\t-h                       Print this help\n");
}

static CallbackResponse jointstates_sub_callback(cRosMessage *message, void* data_context)
{
    int joints_size, joint_ind;

    cRosMessageField *field_name = cRosMessageGetField(message, "name");
    cRosMessageField *field_position = cRosMessageGetField(message, "position");
    cRosMessageField *field_velocity = cRosMessageGetField(message, "velocity");
    cRosMessageField *field_effort = cRosMessageGetField(message, "effort");

    joints_size = field_name->array_size;

    printf("Joint names:");
    for(joint_ind = 0; joint_ind < joints_size; joint_ind++)
    {
        char* joint_name;
        joint_name = cRosMessageFieldArrayAtStringGet(field_name, joint_ind);
        printf(" %s", joint_name);
    }
    printf("\n");

    return 0;
}

#ifdef _WIN32
// This callback function will be called when the console process receives a CTRL_C_EVENT or
// CTRL_CLOSE_EVENT signal.
// Function set_signal_handler() should be called to set function exit_deamon_handler() as the handler of
// these signals
static BOOL WINAPI exit_deamon_handler(DWORD sig)
{
  BOOL sig_handled;

  switch (sig)
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

  if (SetConsoleCtrlHandler(exit_deamon_handler, TRUE))
    ret = 0; // Success setting the control handler
  else
  {
    ret = GetLastError();
    printf("Error setting termination signal handler. Error code=%u\n", ret);
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

  memset(&act, '\0', sizeof(act));

  act.sa_handler = exit_deamon_handler;
  // If the signal handler is invoked while a system call or library function call is blocked,
  // then the we want the call to be automatically restarted after the signal handler returns
  // instead of making the call fail with the error EINTR.
  act.sa_flags = SA_RESTART;
  if (sigaction(SIGINT, &act, &old_int_signal_handler) == 0 && sigaction(SIGTERM, &act, &old_term_signal_handler) == 0)
    ret = 0;
  else
  {
    ret = errno;
    printf("Error setting termination signal handler. errno=%d\n", ret);
  }
  return(ret);
}
#endif

int main(int argc, char **argv)
{
    const char *default_node_name = "/robot_listener",
               *node_name = default_node_name;
    const char *default_host = "127.0.0.1",
               *node_host = default_host,
               *roscore_host = default_host;
    unsigned short roscore_port = 11311;

    if(argc == 2)
    {
        printHelp( argv[0] );
        return EXIT_SUCCESS;
    }

    // srand (time(NULL));

    int i = 1;
    for( ; i < argc - 1; i+=2)
    {
        if( strcmp(argv[i],"-name") == 0)
            node_name = argv[i+1];
        else if( strcmp(argv[i],"-host") == 0)
            node_host = argv[i+1];
        else if( strcmp(argv[i],"-chost") == 0)
            roscore_host = argv[i+1];
        else if( strcmp(argv[i],"-cport") == 0)
        {
            int i_port = atoi(argv[i+1]);
            if( i_port < 0 || i_port > USHRT_MAX )
            {
                fprintf(stderr,"Invalid port %d\n",i_port);
                exit(EXIT_FAILURE);
            }
            roscore_port = (unsigned short)i_port;
        }
        else
        {
            fprintf(stderr,"Invalid option %s\n",argv[i]);
            exit(EXIT_FAILURE);
        }
    }

    printf("Running node \"%s\" with host : %s, roscore host : %s and roscore port : %d\n",
           node_name, node_host, roscore_host, roscore_port );
    printf("To set a different node/host/port, take a look at the options: ");
    printf("%s -h\n", argv[0]);

    char path[4096];
    getcwd(path, sizeof(path));
    strncat(path, DIR_SEPARATOR_STR"rosdb", sizeof(path) - strlen(path) - 1);
    node = cRosNodeCreate(node_name, node_host, roscore_host, roscore_port, path);

    cRosErrCodePack err_cod;
    ROS_INFO(node, "cROS Node created!\n");

    err_cod = cRosApiRegisterSubscriber(node, "/arm/joint_states", "sensor_msgs/JointState",
                                   jointstates_sub_callback, NULL, NULL, 0, NULL);
    if (err_cod != CROS_SUCCESS_ERR_PACK)
        return EXIT_FAILURE;

    if (err_cod != CROS_SUCCESS_ERR_PACK)
        return EXIT_FAILURE;

    // Function exit_deamon_handler() will be called when Ctrl-C is pressed or kill is executed
    set_signal_handler();

    // Run the main loop until exit_flag is 1
    err_cod = cRosNodeStart( node, CROS_INFINITE_TIMEOUT, &exit_flag );
    if(err_cod != CROS_SUCCESS_ERR_PACK)
      cRosPrintErrCodePack(err_cod, "cRosNodeStart() returned an error code");

    // All done: free memory and unregister from ROS master
    err_cod=cRosNodeDestroy( node );
    if(err_cod != CROS_SUCCESS_ERR_PACK)
    cRosPrintErrCodePack(err_cod, "cRosNodeDestroy() failed; Error unregistering from ROS master");

    return EXIT_SUCCESS;
}
