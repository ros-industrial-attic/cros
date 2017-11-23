//
// Created by nico on 26/05/16.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <time.h>
#include <cros.h>

#include <unistd.h>

#define SUBSCRIBER

// TODO Signal handler

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

static uint64_t start_timer = 0;
CrosNode *node;

static CallbackResponse jointstates_sub_callback(cRosMessage *message, void* data_context)
{
    cRosMessageField *field_name = cRosMessageGetField(message, "name");
    cRosMessageField *field_position = cRosMessageGetField(message, "position");
    cRosMessageField *field_velocity = cRosMessageGetField(message, "velocity");
    cRosMessageField *field_effort = cRosMessageGetField(message, "effort");

    uint32_t joints_size = field_name->array_size;
    int i;
    printf("Joint names:");
    for(i = 0; i < joints_size; i++)
    {
        char* joint_name;
        joint_name = cRosMessageFieldArrayAtStringGet(field_name,i);
        printf(" %s", joint_name);
    }
    printf("\n");

    return 0;
}

int main(int argc, char **argv)
{
    char *default_node_name = "/robot_listener",
            *node_name = default_node_name;
    char *default_host = "127.0.0.1",
            *node_host = default_host,
            *roscore_host = default_host;
    unsigned short roscore_port = 11311;

    if(argc == 2)
    {
        printHelp( argv[0] );
        return EXIT_SUCCESS;
    }

    srand (time(NULL));

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

    char path[1024];
    getcwd(path, sizeof(path));
    strncat(path, "/rosdb", sizeof(path));
    node = cRosNodeCreate(node_name, node_host, roscore_host, roscore_port, path, NULL);

    cRosErrCodePack err_cod;
    ROS_INFO(node, "cROS Node (version %.2f) created!\n", 0.9);

    err_cod = cRosApiRegisterSubscriber(node, "/arm/joint_states", "sensor_msgs/JointState",
                                   jointstates_sub_callback, NULL, NULL, 0, NULL);
    if (err_cod != CROS_SUCCESS_ERR_PACK)
        return EXIT_FAILURE;

    if (err_cod != CROS_SUCCESS_ERR_PACK)
        return EXIT_FAILURE;

    unsigned char exit_flag = 0;

    err_cod = cRosNodeStart( node, CROS_INFINITE_TIMEOUT, &exit_flag );
    if(err_cod != CROS_SUCCESS_ERR_PACK)
      cRosPrintErrCodePack(err_cod, "cRosNodeStart() returned an error code");

    // All done: free memory and unregister from ROS master
    err_cod=cRosNodeDestroy( node );
    if(err_cod != CROS_SUCCESS_ERR_PACK)
    cRosPrintErrCodePack(err_cod, "cRosNodeDestroy() failed; Error unregistering from ROS master");

    return EXIT_SUCCESS;
}
