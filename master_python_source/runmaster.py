"""!@brief This Python script runs the ROS Master node, which is needed to use a ROS network.

This script does not requires that a ROS distribution is installed.

@date 5 Mar 2019
"""

#import sys
#sys.path.append('.')
import rosmaster

try:
    input = raw_input
except NameError:
    pass

ros_master=rosmaster.master.Master()
ros_master.start()

ros_master_ok=ros_master.ok()
print("Master is running: {}".format(ros_master_ok))

input("Press Enter to shutdown ROS Master and exit...")

ros_master.stop()
