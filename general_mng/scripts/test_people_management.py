#!/usr/bin/env python

import rospy
from meta_lib.LTPerception import LTPerception
from meta_lib.LTNavigation import LTNavigation
from tf import TransformListener
from geometry_msgs.msg import PointStamped
import os 

class Test:

    def __init__(self):

        rospy.init_node("test_people")

        self._lt_perception = LTPerception()
        # self._lt_navigation = LTNavigation()
        # listener = TransformListener()
        
        response = self._lt_perception.learn_people_meta_from_img_path("/home/student/Bureau/global_palbator/src/fakePkgForTabletPalbator/tablet_code/robocup_palbator-hri_js/public/img/people/Guest_1.png","Bill",10)
        
        # itp_name = "test_People_John"
        # self._lt_navigation.send_nav_order("NP", "CRRCloseToGoal",itp_name , 90.0)
        # self.path_folder_to_save_imgs = "../../../../fakePkgForTabletPalbator/tablet_code/robocup_palbator-hri_js/public"
        
        # dir_path = os.path.dirname(os.path.realpath(__file__))
        # rospy.logwarn("DIR PATH : %s",dir_path)
        # img_path = os.path.join(dir_path,self.path_folder_to_save_imgs)
        # img_path = os.path.join(img_path,"img/people/G1_test.png")
        # self._lt_perception.take_picture_and_save_it_Palbator(img_path)
        
        # response = self._lt_perception.detect_meta_people_from_img_topic(timeout=10)

        # rospy.logwarn("RESPONSE : %s",str(response.payload))

        # payload = response.payload

        # if payload == {}:
        #     rospy.logerr("NADAAAAA")
        # else:
        #     detection = response.payload.peopleMetaList.peopleList
        #     rospy.loginfo("DETECTION : %s",str(detection))

        # pose = response.payload.peopleMetaList.peopleList[0].pose

        
        # now = rospy.Time(0)
        # object_point = PointStamped()
        # object_point.header.frame_id = "palbator_arm_kinect_link"
        # object_point.header.stamp = now
        # object_point.point.x = pose.position.x
        # object_point.point.y = pose.position.y
        # object_point.point.z = pose.position.z

        # rospy.loginfo("{class_name} : Object coords in palbator_arm_kinect_link : %s".format(class_name=self.__class__.__name__),str(object_point))
        # listener.waitForTransform("map", "/palbator_arm_kinect_link", now, rospy.Duration(20))
        # target = listener.transformPoint("/map",object_point)

        # rospy.loginfo("{class_name} : Object coords in map : %s".format(class_name=self.__class__.__name__),str(target))


        # self._bridge = CvBridge()
        # self.sub = rospy.Subscriber("/people_meta_info",PeopleMetaInfoList,self.handle_detection)

        # response = self._lt_perception.learn_people_meta_from_img_topic(name="Thomas",timeout=10)
        # response = self._lt_perception.learn_people_meta_from_img_path("/home/student/Bureau/global_palbator/src/robocup-main/robocup_pepper-people_mng/people_mng/ros_people_mng_node/data/test_Thomas.jpg","Robert",10)

        # rospy.sleep(5)

        # response = self._lt_perception.get_people_name_from_img_path("/home/student/Bureau/global_palbator/src/robocup-main/robocup_pepper-people_mng/people_mng/ros_people_mng_node/data/test_Thomas.jpg",10)
        # rospy.loginfo("RESPONSE : %s",str(response.payload.peopleMetaList.peopleList))
        # rospy.logwarn("RESPONSE MSG : %s",str(response.msg))

        # response = self._lt_perception.take_picture_and_save_it_Palbator("/home/student/Bureau/global_palbator/src/robocup-main/robocup_pepper-general_mng/general_mng/scripts/test.png")

        # rospy.loginfo(response)

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
    # while not rospy.is_shutdown():
    #     rospy.spin()

