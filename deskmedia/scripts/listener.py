#!/usr/bin/env python
import rospy
import rosnode
import subprocess
from std_msgs.msg import String


class launch_task:
    def __init__(self, cmd=None):
        self.cmd = cmd
 
    def launch(self):
        self.child = subprocess.Popen(self.cmd)
        return True
 
    def shutdown(self):
        self.child.terminate()
        self.child.wait()
        return True

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # if (data.data.find("mapping"))
    # launchTask(["roslaunch", "turn_on_wheeltec_robot", "navigation.launch"])
    # rosnode.kill_nodes(['listener'])
    # child = subprocess.Popen(["roslaunch", "turn_on_wheeltec_robot", "mapping_for_dm.launch"]) 
    task = launch_task(["roslaunch", "turn_on_wheeltec_robot", "mapping_for_dm.launch"])
    task.launch()

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("dm_control", String, callback)

    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        r.sleep()