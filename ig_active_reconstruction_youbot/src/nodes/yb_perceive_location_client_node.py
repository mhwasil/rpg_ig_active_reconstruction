#! /usr/bin/env python
import rospy
import roslib
import actionlib
from std_msgs.msg import Int32

from mir_yb_action_msgs.msg import PerceiveLocationAction, PerceiveLocationGoal
from ig_active_reconstruction_msgs.srv import youbotObjectDetector


def detect_object(req):  
    rospy.loginfo("Object detection is requested with obj numb: %s", req.obj_number)
    client = actionlib.SimpleActionClient('perceive_location_server', PerceiveLocationAction)
    client.wait_for_server()
    goal = PerceiveLocationGoal()
    goal.location = "ws-05"
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(15.0))
    rospy.loginfo("Result ")
    print client.get_result()
    result = len(client.get_result().object_list)
    print result
    #publish_obj_list(result)
    #rsp = youbotObjectDetector()
    #req.result = result
    rsp = Int32()
    rsp.data = result
    youbotObjectDetector.result = rsp
    return youbotObjectDetector.result

def object_detector_server():
    rospy.init_node('yb_perceive_location_client_node')
    service_server = rospy.Service('youbot/object_detector', youbotObjectDetector, detect_object)
    rospy.loginfo("youbot/object_detector service is ready.")
    rospy.spin()

if __name__ == '__main__':
    object_detector_server()
