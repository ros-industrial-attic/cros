/*! \file parameters-test.c
 *  \brief The file is an example of cROS usage implementing a parameter subscriber. This sample
 *         can be used together with rosparam or MATLAB to prove the operation of parameter sharing
 *         between two nodes. If MATLAB is used, it must not be used as ROS master node since it seems that
 *         it does not notify parameter updates.
 *
 *  It creates a parameter subscriber to the key /testparam. Each time this parameter is updated by
 *  other node the master node informs about it and the callback function getNodeStatusCallback() is executed.
 *  This function checks if the parameters /testparam/x, /testparam/y and /testparam/z have been updated.
 *  When all of these parameters are updated by other node, the function changes their value to
 *  {x: 5, y: hello, z: [0,1,2,4]} and continues checking again when all the them change.
 *  - With ROS core it can be tested by running:
 *  $ rosparam set /testparam "{x: 3, y: ciao, z: [1,2,3]}"
 *  - With MATLAB R2015a (or later) it can be tested by running:
 *  >> rosinit
 *  >> rosparam('set', '/testparam/x', int32(3))
 *  >> rosparam('set', '/testparam/y', 'ciao')
 *  >> rosparam('set', '/testparam/z', {int32(1),int32(2),int32(3)})
 *  >> rosparam('get', '/testparam')
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

#include "cros_log.h"
#include "cros_node.h"
#include "cros_api.h"
#include "cros_clock.h"

CrosNode *node;

int paramx_updated = 0;
int paramy_updated = 0;
int paramz_updated = 0;
static unsigned char exit_flag = 0; //! ROS-node loop exit flag. When it is set to 1 the cRosNodeStart() function exits

static void getParamNamesCallback(int callid, GetParamNamesResult *result, void *context)
{
  // OK
}

static void setParamCallback(int callid, SetParamResult *result, void *context)
{
  // OK
}

static void getNodeStatusCallback(CrosNodeStatusUsr *status, void* context)
{
/*
  XmlrpcParam *param0;
  XmlrpcParam *param1;
  XmlrpcParam *param2;

  if (status->parameter_value->array_n_elem == 3)
  {
    param0 = &status->parameter_value->data.as_array[0];
    param1 = &status->parameter_value->data.as_array[1];
    param2 = &status->parameter_value->data.as_array[2];
  }
*/

  if (status->state == CROS_STATUS_PARAM_UPDATE)
  {
    ROS_INFO(node, "Subscribed parameter updated: %s\n", status->parameter_key);

    if (strcmp(status->parameter_key, "/testparam/x/") == 0)
      paramx_updated = 1;

    if (strcmp(status->parameter_key, "/testparam/y/") == 0)
      paramy_updated = 1;

    if (strcmp(status->parameter_key, "/testparam/z/") == 0)
      paramz_updated = 1;

    if (paramx_updated && paramy_updated && paramz_updated)
    {
      cRosErrCodePack err_cod;

      XmlrpcParam param;
      xmlrpcParamInit(&param);
      xmlrpcParamSetStruct(&param);

      xmlrpcParamStructPushBackInt(&param, "x", 5);
      xmlrpcParamStructPushBackString(&param, "y", "hello");
      XmlrpcParam *array = xmlrpcParamStructPushBackArray(&param, "z");
      xmlrpcParamArrayPushBackInt(array, 0);
      xmlrpcParamArrayPushBackInt(array, 1);
      xmlrpcParamArrayPushBackInt(array, 2);
      xmlrpcParamArrayPushBackInt(array, 4);

      ROS_INFO(node, "x, y, and z parameter values changed: Modifying their value\n");
      err_cod = cRosApiSetParam(node, "/testparam", &param, setParamCallback, NULL, NULL);
      if (err_cod != CROS_SUCCESS_ERR_PACK)
        cRosPrintErrCodePack(err_cod, "cRosApiSetParam() failed");

      paramx_updated = 0;
      paramy_updated = 0;
      paramz_updated = 0;
      xmlrpcParamRelease(&param);
    }
  }
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
  const char *default_node_name = "/Gripper",
       *node_name = default_node_name;
  const char *default_host = "127.0.0.1",
       *node_host = default_host,
       *roscore_host = default_host;
  unsigned short roscore_port = 11311;
  char path[4097];
  cRosErrCodePack err_cod;

  getcwd(path, sizeof(path));
  strncat(path, DIR_SEPARATOR_STR"rosdb", sizeof(path) - strlen(path) - 1);
  node = cRosNodeCreate(node_name, node_host, roscore_host, roscore_port, path);

  err_cod = cRosApiSubscribeParam(node, "/testparam", getNodeStatusCallback, NULL, NULL);
  if (err_cod != CROS_SUCCESS_ERR_PACK)
  {
    cRosPrintErrCodePack(err_cod, "cRosApiSubscribeParam() failed");
    cRosNodeDestroy( node );
    return EXIT_FAILURE;
  }

  printf("Node XMLRPC port: %i\n", node->xmlrpc_port);

  err_cod = cRosApiGetParamNames(node, getParamNamesCallback, NULL, NULL);
  if (err_cod != CROS_SUCCESS_ERR_PACK)
  {
    cRosPrintErrCodePack(err_cod, "cRosApiGetParamNames() failed");
    cRosNodeDestroy( node );
    return EXIT_FAILURE;
  }

  // Function exit_deamon_handler() will be called when Ctrl-C is pressed or kill is executed
  set_signal_handler();

  // Run the main loop until exit_flag is 1
  err_cod = cRosNodeStart( node, CROS_INFINITE_TIMEOUT, &exit_flag );
  if(err_cod != CROS_SUCCESS_ERR_PACK)
    cRosPrintErrCodePack(err_cod, "cRosNodeStart() returned an error code");

  // Free memory and unregister
  err_cod=cRosNodeDestroy( node );
  if(err_cod != CROS_SUCCESS_ERR_PACK)
  {
    cRosPrintErrCodePack(err_cod, "cRosNodeDestroy() failed; Error unsubscribing in ROS master");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
