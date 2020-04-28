#!/usr/bin/env python

__author__ = 'Simon ERNST & Thomas CURE'
import rospy

from meta_lib.LTPerception import LTPerception
from std_msgs.msg import String

class Test():
    
    def __init__(self):

        rospy.init_node("test_LTPerception")
        self._lt_perception = LTPerception()



        self.sub = rospy.Subscriber("/activationDarknet",String,self.callback_sb)
        self.list_obj = ['bleach','chips', 'coffee', 'cracker' ,'jello', 'mustard', 'pitcher', 'pottedmeat', 'pudding', 'sugar', 'tomatosoup', 'tuna', 'windex']
        
        self.timeout=10.0

    def callback_sb(self,req):
        if req.data == "1":
            path = "/home/student/Bureau/TESTS_ws/src/darknet_ros/darknet_ros/src/test.jpg"
            response = self._lt_perception.detect_objects_with_given_sight_from_img_path(self.list_obj,path,self.timeout)
            rospy.loginfo("{class_name} : RESULT DARKNET 1 " + str(response))


            # path = "/home/student/Bureau/TESTS_ws/src/darknet_ros/darknet_ros/src/test2.jpg"
            # response = self._lt_perception.__detect_objects_with_given_sight_from_img_path(self.list_obj,path,self.timeout)
            # rospy.loginfo("{class_name} : RESULT DARKNET 2 " + str(response))


        # else:
        #     response = self._lt_perception.detect_objects_with_given_sight_from_img_topic(['sugar','windex'],self.NO_TIMEOUT)
        #     rospy.loginfo("{class_name} : RESULT DARKNET 2 " + str(response))




if __name__ == "__main__":
    a=Test()
    while not rospy.is_shutdown():
        rospy.spin()