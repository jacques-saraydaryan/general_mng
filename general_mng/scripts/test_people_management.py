#!/usr/bin/env python

import rospy
from meta_lib.LTPerception import LTPerception
from ros_people_mng_msgs.msg import PeopleMetaInfoDetails, PeopleMetaInfo, PeopleMetaInfoList
from cv_bridge import CvBridge, CvBridgeError
import cv2
class Test:

    def __init__(self):

        rospy.init_node("test_people")

        self._lt_perception = LTPerception()
        self._bridge = CvBridge()
        # self.sub = rospy.Subscriber("/people_meta_info",PeopleMetaInfoList,self.handle_detection)

        # response = self._lt_perception.learn_people_meta_from_img_topic(name="Thomas",timeout=10)
        response = self._lt_perception.learn_people_meta_from_img_path("/home/student/Bureau/global_palbator/src/robocup-main/robocup_pepper-people_mng/people_mng/ros_people_mng_node/data/test_Thomas.jpg","Robert",10)

        rospy.sleep(5)

        response = self._lt_perception.get_people_name_from_img_path("/home/student/Bureau/global_palbator/src/robocup-main/robocup_pepper-people_mng/people_mng/ros_people_mng_node/data/test_Thomas.jpg",10)
        rospy.loginfo("RESPONSE : %s",str(response.payload.peopleMetaList.peopleList))
        rospy.logwarn("RESPONSE MSG : %s",str(response.msg))

        # img_cv2 = self._bridge.imgmsg_to_cv2(response.payload.peopleMetaList.img, "bgr8")
        # cv2.imshow("test",img_cv2)

        # response = self._lt_perception.detect_meta_people_from_img_topic(timeout=10)
        # rospy.loginfo("RESPONSE : %s",str(response.payload))

    def handle_detection(self,req):
        peopleList = req.peopleList

        for item in peopleList:
            rospy.loginfo(item)
            rospy.loginfo("----------")


if __name__ == "__main__":
    a = Test()
    while not rospy.is_shutdown():
        rospy.spin()

