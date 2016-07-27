#include <cros.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

CrosNode *node;

// This callback will be invoked when it's our turn to publish a new message
static CallbackResponse callback_pub(cRosMessage *message, void* data_context)
{
  static int count = 0;
  char buf[1024];
  // We need to index into the message structure and then assign to fields
  cRosMessageField *data_field = cRosMessageGetField(message, "data");
  if(data_field)
  {
    snprintf(buf, sizeof(buf), "hello world %d", count);
    if(cRosMessageSetFieldValueString(data_field, buf) == 0)
    {
      ROS_INFO(node, "%s", buf);
    }
  }
  ++count;
  return 0;
}

int main(int argc, char **argv)
{
  // We need to tell our node where to find the .msg files that we'll be using
  char path[1024];
  getcwd(path, sizeof(path));
  strncat(path, "/rosdb", sizeof(path));
  // Create a new node and tell it to connect to roscore in the usual place
  node = cRosNodeCreate("/talker", "127.0.0.1", "127.0.0.1", 11311, path, NULL);
  // Create a publisher and request that the associated callback be invoked every 100ms (10Hz)
  if(cRosApiRegisterPublisher(node, "/chatter","std_msgs/String", 100,
                                callback_pub, NULL, NULL) < 0)
  {
    printf("cRosApiRegisterPublisher failed; did you run this program one directory above 'rosdb'?\n");
    return EXIT_FAILURE;
  }
  // Run the main loop
  unsigned char exit = 0;
  cRosNodeStart( node, &exit );
  // All done
  cRosNodeDestroy( node );
  return EXIT_SUCCESS;
}
