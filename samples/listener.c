#include <cros.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

CrosNode *node;

// This callback will be invoked when we receive a message
static CallbackResponse callback_sub(cRosMessage *message, void* data_context)
{
  cRosMessageField *data_field = cRosMessageGetField(message, "data");
  if(data_field)
  {
    ROS_INFO(node, "I heard: [%s]", data_field->data.as_string);
  }
  return 0;
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
  if(cRosApiRegisterSubscriber(node, "/chatter","std_msgs/String",
                               callback_sub, NULL, NULL) < 0)
  {
    printf("cRosApiRegisterSubscriber failed; did you run this program one directory above 'rosdb'?\n");
    return EXIT_FAILURE;
  }
  // Run the main loop
  unsigned char exit = 0;
  cRosNodeStart( node, &exit );
  // All done
  cRosNodeDestroy( node );
  return EXIT_SUCCESS;
}
