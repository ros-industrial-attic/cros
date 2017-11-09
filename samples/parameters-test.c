#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <time.h>

#include <cros_defs.h>
#include "cros_node.h"
#include "cros_api.h"
#include "cros_clock.h"

#include <unistd.h>

CrosNode *node;
uint64_t start_timer = 0;
int paramx_updated = 0;
int paramy_updated = 0;
int paramz_updated = 0;

void getParamNamesCallback(int callid, GetParamNamesResult *result, void *context)
{
  // OK
}

void setParamCallback(int callid, SetParamResult *result, void *context)
{
  // OK
}

// Testato con:
// $ rosparam set /testparam "{x: 3, y: ciao, z: [1,2,3]}"

static void getNodeStatusCallback(CrosNodeStatusUsr *status, void* context)
{
  XmlrpcParam *param0;
  XmlrpcParam *param1;
  XmlrpcParam *param2;

  if (status->parameter_value->array_n_elem == 3)
  {
    param0 = &status->parameter_value->data.as_array[0];
    param1 = &status->parameter_value->data.as_array[1];
    param2 = &status->parameter_value->data.as_array[2];
  }

  if (status->state == CROS_STATUS_PARAM_UPDATE)
  {
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

      err_cod = cRosApiSetParam(node, "/testparam", &param, setParamCallback, NULL);
      if (err_cod != CROS_SUCCESS_ERR_PACK)
        cRosPrintErrCodePack(err_cod, "cRosApiSetParam() failed");

      paramx_updated = 0;
      paramy_updated = 0;
      paramz_updated = 0;
      xmlrpcParamRelease(&param);
    }
  }
}

int main(int argc, char **argv)
{
  char *default_node_name = "/Gripper",
       *node_name = default_node_name;
  char *default_host = "127.0.0.1",
       *node_host = default_host,
       *roscore_host = default_host;
  unsigned short roscore_port = 11311;
  char path[1024];
  cRosErrCodePack err_cod;

  srand(time(NULL));

  getcwd(path, sizeof(path));
  strncat(path, "/rosdb", sizeof(path));
  node = cRosNodeCreate(node_name, node_host, roscore_host, roscore_port, path, NULL);

  err_cod = cRosApiSubscribeParam(node,"/testparam", getNodeStatusCallback, NULL, NULL);
  if (err_cod != CROS_SUCCESS_ERR_PACK)
  {
    cRosPrintErrCodePack(err_cod, "cRosApiSubscribeParam() failed");
    cRosNodeDestroy( node );
    return EXIT_FAILURE;
  }

  err_cod = cRosApiGetParamNames(node, getParamNamesCallback, NULL);
  if (err_cod != CROS_SUCCESS_ERR_PACK)
  {
    cRosPrintErrCodePack(err_cod, "cRosApiGetParamNames() failed");
    cRosNodeDestroy( node );
    return EXIT_FAILURE;
  }


  unsigned char exit = 0;
  cRosNodeStart( node, &exit );
  cRosNodeDestroy( node );

  return EXIT_SUCCESS;
}
