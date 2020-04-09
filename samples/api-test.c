#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <time.h>

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

// TODO Signal handler

static void printHelp( char *cmd_name )
{
  printf("Usage: %s [OPTION] ... \n", cmd_name);
  printf("Options:\n");
  printf("\t-name  <node_host>       Set the node name (default: /test_node)\n");
  printf("\t-host  <node_host>       Set the node host (default: 127.0.0.1)\n");
  printf("\t-chost <roscore host>    Set the roscore host (default: 127.0.0.1)\n");
  printf("\t-cport <roscore port>    Set the roscore port (default: 11311)\n");
  printf("\t-sub                     run as a subscriber (default mode)\n");
  printf("\t-pub                     run as a publisher\n");
  printf("\t-h                       Print this help\n");
}

CrosNode *node;

static CallbackResponse gripperstatus_sub_callback(cRosMessage *message, void* data_context)
{
  CallbackResponse cb_resp;
  cRosMessageField *PowerStatus_field = cRosMessageGetField(message, "PowerStatus");
  cRosMessageField *ClampStatus_field = cRosMessageGetField(message, "ClampStatus");
  cRosMessageField *AlarmCodes_field = cRosMessageGetField(message, "AlarmCodes");
  cRosMessageField *Configuration_field = cRosMessageGetField(message, "Configuration");

  uint32_t PowerStatus = PowerStatus_field->data.as_uint32;
  uint32_t NAlarmCodes = AlarmCodes_field->array_size;
  uint32_t Configuration = Configuration_field->data.as_uint32;
  uint32_t ClampStatus[4];
  ClampStatus[0] = *cRosMessageFieldArrayAtUInt32(ClampStatus_field, 0);
  ClampStatus[1] = *cRosMessageFieldArrayAtUInt32(ClampStatus_field, 1);
  ClampStatus[2] = *cRosMessageFieldArrayAtUInt32(ClampStatus_field, 2);
  ClampStatus[3] = *cRosMessageFieldArrayAtUInt32(ClampStatus_field, 3);

  uint32_t *AlarmCodes = (uint32_t *)malloc(NAlarmCodes*sizeof(uint32_t));
  if (AlarmCodes != NULL)
  {
    uint32_t it;
    for (it = 0; it < NAlarmCodes; it++)
    {
      AlarmCodes[it] = *cRosMessageFieldArrayAtUInt32(AlarmCodes_field, it);
    }

    printf("GripperStatus: PowerStatus: %d\n", PowerStatus);
    printf("GripperStatus: ClampStatus: [%d %d %d %d]\n", ClampStatus[0], ClampStatus[1],
      ClampStatus[2], ClampStatus[3]);
    printf("GripperStatus: AlarmCodes: [");
    int first = 1;

    for (it = 0; it < NAlarmCodes; it++)
    {
      if (first)
        first = 0;
      else
        printf(" ");

      printf("%d", AlarmCodes[it]);
    }
    printf("]\n");
    printf("GripperStatus: Configuration: %d\n", Configuration);
    fflush(stdout);

    free(AlarmCodes);
    cb_resp = 0; //success
  }
  else
  {
    ROS_ERROR(node, "Error allocating memory for AlarmCodes\n");
    cb_resp = 1; //fail
  }

  return(cb_resp);
}

static CallbackResponse gripperjoints_sub_callback(cRosMessage *message, void* context)
{
  cRosMessageField *Position_field = cRosMessageGetField(message, "Position");
  double Position[9];
  Position[0] = *cRosMessageFieldArrayAtFloat64(Position_field, 0);
  Position[1] = *cRosMessageFieldArrayAtFloat64(Position_field, 1);
  Position[2] = *cRosMessageFieldArrayAtFloat64(Position_field, 2);
  Position[3] = *cRosMessageFieldArrayAtFloat64(Position_field, 3);
  Position[4] = *cRosMessageFieldArrayAtFloat64(Position_field, 4);
  Position[5] = *cRosMessageFieldArrayAtFloat64(Position_field, 5);
  Position[6] = *cRosMessageFieldArrayAtFloat64(Position_field, 6);
  Position[7] = *cRosMessageFieldArrayAtFloat64(Position_field, 7);
  Position[8] = *cRosMessageFieldArrayAtFloat64(Position_field, 8);

  printf("GripperJoints: Position: [%f %f %f %f %f %f %f %f %f]\n", Position[0], Position[1], Position[2],
    Position[3], Position[4], Position[5], Position[6], Position[7], Position[8]);

  fflush(stdout);
  return 0;
}

static CallbackResponse callback_pub_gripperstatus(cRosMessage *message, void* data_context)
{
  cRosMessageField *PowerStatus_field = cRosMessageGetField(message, "PowerStatus");
  cRosMessageField *ClampStatus_field = cRosMessageGetField(message, "ClampStatus");
  cRosMessageField *AlarmCodes_field = cRosMessageGetField(message, "AlarmCodes");
  cRosMessageField *Configuration_field = cRosMessageGetField(message, "Configuration");

  PowerStatus_field->data.as_uint32 = 1;

  *cRosMessageFieldArrayAtUInt32(ClampStatus_field, 0) = 0;
  *cRosMessageFieldArrayAtUInt32(ClampStatus_field, 1) = 1;
  *cRosMessageFieldArrayAtUInt32(ClampStatus_field, 2) = 2;
  *cRosMessageFieldArrayAtUInt32(ClampStatus_field, 3) = 3;

  cRosMessageFieldArrayClear(AlarmCodes_field);
  cRosMessageFieldArrayPushBackUInt32(AlarmCodes_field, 1);
  cRosMessageFieldArrayPushBackUInt32(AlarmCodes_field, 8);

  Configuration_field->data.as_uint32 = 3;

  return 0;
}

static CallbackResponse callback_pub_gripperjoints(cRosMessage *message, void* context)
{
  cRosMessageField *Position_field = cRosMessageGetField(message, "Position");

  *cRosMessageFieldArrayAtFloat64(Position_field, 0) = 8;
  *cRosMessageFieldArrayAtFloat64(Position_field, 1) = 7;
  *cRosMessageFieldArrayAtFloat64(Position_field, 2) = 6;
  *cRosMessageFieldArrayAtFloat64(Position_field, 3) = 5;
  *cRosMessageFieldArrayAtFloat64(Position_field, 4) = 4;
  *cRosMessageFieldArrayAtFloat64(Position_field, 5) = 3;
  *cRosMessageFieldArrayAtFloat64(Position_field, 6) = 2;
  *cRosMessageFieldArrayAtFloat64(Position_field, 7) = 1;
  *cRosMessageFieldArrayAtFloat64(Position_field, 8) = 0;

  return 0;
}

/*
 * Service callbacks definition
 */

static CallbackResponse callback_srv_close_gripper(cRosMessage *request, cRosMessage *response, void* context)
{
  srand((unsigned int)time(NULL));
  uint8_t response_val = rand() % 2;

  cRosMessageField *closed_field = cRosMessageGetField(response, "closed");
  closed_field->data.as_uint8 = response_val;

  printf("Service close_clamps. Arguments: none . Response %d \n", response_val);

  fflush(stdout);
  return 0;
}

static CallbackResponse callback_srv_open_gripper(cRosMessage *request, cRosMessage *response, void* context)
{
  srand((unsigned int)time(NULL));
  uint8_t response_val = rand() % 2;

  cRosMessageField *opened_field = cRosMessageGetField(response, "opened");
  opened_field->data.as_uint8 = response_val;

  printf("Service open_clamps. Arguments: none . Response %d \n", response_val);
  fflush(stdout);

  return 0;
}

static CallbackResponse callback_srv_rest(cRosMessage *request, cRosMessage *response, void* context)
{
  srand((unsigned int)time(NULL));
  uint8_t response_val = rand() % 2;

  cRosMessageField *parked_field = cRosMessageGetField(response, "parked");
  parked_field->data.as_uint8 = response_val;

  printf("Service park. Arguments: none . Response %d \n", response_val);
  fflush(stdout);

  return 0;
}

static CallbackResponse callback_srv_reconfigure(cRosMessage *request, cRosMessage *response, void* context)
{
  cRosMessageField *id_field = cRosMessageGetField(request, "id");
  int64_t reconfigure_id;
  reconfigure_id = id_field->data.as_int64;

  srand((unsigned int)time(NULL));
  uint8_t response_val = rand() % 2;

  cRosMessageField *reconfigured_field = cRosMessageGetField(response, "reconfigured");
  reconfigured_field->data.as_uint8 = response_val;

  printf("Service reconfigure. Arguments: %lld. Response %u \n", (long long)reconfigure_id, response_val);
  fflush(stdout);

  return 0;
}

static CallbackResponse callback_srv_move_arm(cRosMessage *request, cRosMessage *response, void* context)
{
  cRosMessageField *arm_field = cRosMessageGetField(request, "arm");
  cRosMessageField *rad_field = cRosMessageGetField(request, "rad");
  cRosMessageField *velocity_field = cRosMessageGetField(request, "velocity");

  int32_t arm = arm_field->data.as_int32;
  int32_t rad = arm_field->data.as_int32;
  int32_t velocity = arm_field->data.as_int32;

  srand((unsigned int)time(NULL));
  uint8_t response_val = rand() % 2;

  cRosMessageField *positioned_field = cRosMessageGetField(response, "positioned");
  positioned_field->data.as_uint8 = response_val;

  printf("Service reconfigure. Arguments: {arm: %d, rad: %d, velocity: %d} . Response %d \n", arm, rad, velocity, response_val);
  fflush(stdout);

  return 0;
}

static CallbackResponse callback_srv_transfer(cRosMessage *request, cRosMessage *response, void* context)
{
  srand((unsigned int)time(NULL));
  uint8_t response_val = rand() % 2;

  cRosMessageField *transfered_field = cRosMessageGetField(response, "transfered");
  transfered_field->data.as_uint8 = response_val;

  printf("Service transfer. Arguments: none . Response %d \n", response_val);
  fflush(stdout);

  return 0;
}

int main(int argc, char **argv)
{
  const char *def_pub_node_name = "/Gripper_pub";
  const char *def_sub_node_name = "/Gripper_sub";
  const char *node_name;
  const char *default_host = "127.0.0.1",
       *node_host = default_host,
       *roscore_host = default_host;
  unsigned short roscore_port = 11311;
  int run_as_publisher = 0;

  node_name = NULL;
  int i;
  for(i = 1; i < argc; i++)
  {
    if( strcmp(argv[i],"-name") == 0)
      node_name = argv[++i];
    else if (strcmp(argv[i], "-sub") == 0)
      run_as_publisher = 0;
    else if (strcmp(argv[i], "-pub") == 0)
      run_as_publisher = 1;
    else if (strcmp(argv[i], "-h") == 0)
    {
      printHelp(argv[0]);
      return(EXIT_SUCCESS);
    }
    else if( strcmp(argv[i],"-host") == 0)
      node_host = argv[++i];
    else if( strcmp(argv[i],"-chost") == 0)
      roscore_host = argv[++i];
    else if( strcmp(argv[i],"-cport") == 0)
    {
      int i_port = atoi(argv[++i]);
      if( i_port < 0 || i_port > USHRT_MAX )
      {
        fprintf(stderr,"Invalid port value %d.\n",i_port);
        return(EXIT_FAILURE);
      }
      roscore_port = (unsigned short)i_port;
    }
    else
    {
      fprintf(stderr,"Invalid option %s\n",argv[i]);
      return(EXIT_FAILURE);
    }
  }

  if (node_name == NULL) // Node name has not been specified, so use the default one
  {
    if (run_as_publisher)
      node_name = def_pub_node_name;
    else
      node_name = def_sub_node_name;
  }

  printf("Running node \"%s\" with host : %s, roscore host : %s and roscore port : %d\n",
          node_name, node_host, roscore_host, roscore_port );
  printf("To set a different node/host/port, take a look at the options: ");
  printf("%s -h\n", argv[0]);

  char path[4096];
  getcwd(path, sizeof(path));
  strncat(path, DIR_SEPARATOR_STR"rosdb", sizeof(path) - strlen(path) - 1);

  srand((unsigned int)time(NULL));

  node = cRosNodeCreate(node_name, node_host, roscore_host, roscore_port, path);

  cRosErrCodePack err_cod;
  ROS_INFO(node, "cROS Node created!\n");

  if (run_as_publisher)
  {
    ROS_INFO(node, "Example running as publisher\n");

    // Topics
    err_cod = cRosApiRegisterPublisher(node, "/gripperstatus", "gripping_robot/GripperStatus", 1000,
        callback_pub_gripperstatus, NULL, NULL, NULL);
    if (err_cod != CROS_SUCCESS_ERR_PACK)
      return EXIT_FAILURE;

    err_cod = cRosApiRegisterPublisher(node, "/gripperjoints", "gripping_robot/GripperJoints", 1000,
      callback_pub_gripperjoints, NULL, NULL, NULL);
    if (err_cod != CROS_SUCCESS_ERR_PACK)
      return EXIT_FAILURE;

    // Services
    err_cod = cRosApiRegisterServiceProvider(node, "/close_clamps", "gripping_robot/CloseGripper",
      callback_srv_close_gripper, NULL, NULL, NULL);
    if (err_cod != CROS_SUCCESS_ERR_PACK)
      return EXIT_FAILURE;

    err_cod = cRosApiRegisterServiceProvider(node, "/open_clamps", "gripping_robot/OpenGripper",
      callback_srv_open_gripper, NULL, NULL, NULL);
    if (err_cod != CROS_SUCCESS_ERR_PACK)
      return EXIT_FAILURE;

    err_cod = cRosApiRegisterServiceProvider(node, "/park_configuration", "gripping_robot/RestPosition",
      callback_srv_rest, NULL, NULL, NULL);
    if (err_cod != CROS_SUCCESS_ERR_PACK)
      return EXIT_FAILURE;

    err_cod = cRosApiRegisterServiceProvider(node, "/reconfigure", "gripping_robot/Reconfigure",
      callback_srv_reconfigure, NULL, NULL, NULL);
    if (err_cod != CROS_SUCCESS_ERR_PACK)
      return EXIT_FAILURE;

    err_cod = cRosApiRegisterServiceProvider(node, "/transfer", "gripping_robot/Transfer",
      callback_srv_transfer, NULL, NULL, NULL);
    if (err_cod != CROS_SUCCESS_ERR_PACK)
      return EXIT_FAILURE;

    err_cod = cRosApiRegisterServiceProvider(node, "/move_arm", "gripping_robot/MoveArm",
      callback_srv_move_arm, NULL, NULL, NULL);
    if (err_cod != CROS_SUCCESS_ERR_PACK)
      return EXIT_FAILURE;
  }
  else
  {
    ROS_INFO(node, "Example running as subscriber\n");

    err_cod = cRosApiRegisterSubscriber(node, "/gripperstatus", "gripping_robot/GripperStatus",
      gripperstatus_sub_callback, NULL, NULL, 0, NULL);
    if (err_cod != CROS_SUCCESS_ERR_PACK)
      return EXIT_FAILURE;

    err_cod = cRosApiRegisterSubscriber(node, "/gripperjoints", "gripping_robot/GripperJoints",
      gripperjoints_sub_callback, NULL, NULL, 0, NULL);
    if (err_cod != CROS_SUCCESS_ERR_PACK)
      return EXIT_FAILURE;
  }

  unsigned char exit_flag = 0;

  err_cod = cRosNodeStart( node, CROS_INFINITE_TIMEOUT, &exit_flag );
  if(err_cod != CROS_SUCCESS_ERR_PACK)
    cRosPrintErrCodePack(err_cod, "cRosNodeStart() returned error code(s)");

  // All done: free memory and unregister from ROS master
  err_cod=cRosNodeDestroy( node );
  if(err_cod != CROS_SUCCESS_ERR_PACK)
    cRosPrintErrCodePack(err_cod, "cRosNodeDestroy() failed; Error unregistering from ROS master");

  return EXIT_SUCCESS;
}
