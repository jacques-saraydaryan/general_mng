#!/usr/bin/env python
__author__ = 'Thomas CURE'

from abc import ABCMeta, abstractmethod
from meta_lib.LTAbstract import LTAbstract

from meta_lib.LTPerception import LTPerception
from meta_lib.LTNavigation import LTNavigation
from meta_lib.LTMotion import LTMotionPalbator
import rospy
import json

import math
from tf import TransformListener

class LTHighBehaviour(LTAbstract):

    _enableHighBehaviour = True

    def __init__(self):
        """
        Initializes the LTHighBehaviour API for Palbator. It will manage high-level behaviours which combine several low-level APIs 
        (LTPerception + LTMotion to detect object + turn around for instance)
        """
        self.configure_intern()

        #Inform configuration is ready
        self.configurationReady = True
        rospy.loginfo("{class_name}: High Level Behaviours API initialized".format(class_name=self.__class__.__name__))

    #######################################
    # CONFIGURATION
    ######################################

    def configure_intern(self):
        """
        Loads the configuration needed to use correctly every high-level function.
        """
        rospy.loginfo("{class_name}: Initiating High Level Behaviours API ... ".format(class_name=self.__class__.__name__))

        self._lt_perception = LTPerception()
        self._lt_navigation = LTNavigation()
        self._lt_motion_palbator = LTMotionPalbator()
        

    def reset(self):
        """
        Reloads the configuration needed to use correctly every high-level function.
        """
        self.configure_intern()

    #######################################
    # HIGH LEVEL BEHAVIOURS 
    ######################################

    def turn_around_and_detect_objects(self, room_to_inspect, number_of_rotation, nav_timeout):
        """
        Will send a navigation order to turn around with a specified number of rotations and after that, the system will send a perception order to get objects list in the room.
        Returns the closest object in the specified room.

        :param room_to_inspect: name of the room to inspect
        :type room_to_inspect: string
        :param number_of_rotation: number of rotations to describe a circle
        :type number_of_rotation: int
        :param nav_timeout: maximum time to realize the rotation
        :type nav_timeout: float
        """
        try:
            for i in range(0,number_of_rotation):
                rotation_angle = (2*math.pi)/float(number_of_rotation)
                rospy.loginfo("{class_name}: ROTATION %s of %s radians".format(class_name=self.__class__.__name__),str(i),str(rotation_angle))
                response_nav = self._lt_navigation.send_nav_rotation_order("NT", rotation_angle , nav_timeout)
                rospy.sleep(2)

            response = self._lt_perception.get_object_in_room(room_to_inspect)
            objects_list = response.payload
            rospy.logwarn("{class_name}: OBJECTS IN ROOM %s".format(class_name=self.__class__.__name__),str(objects_list))

            if len(objects_list) != 0:
                closest_object = self.get_closest_object(objects_list)
                return closest_object
            else:
                return None

        except Exception as e:
            rospy.logwarn("{class_name}: ###### TURN AROUND AND DETECT OBJECT FAILURE , State: %s".format(class_name=self.__class__.__name__), str(e))
            return None

    
    def get_closest_object(self,objects_list):
        """
        Will get the closest object to the robot from an object list.
        Return the data of the closest object.

        :param objects_list: list of objects detected in a room
        :type objects_list: list
        """
        try:
            tflistener = TransformListener()
            now = rospy.Time(0)
            tflistener.waitForTransform("/map", "/base_footprint", now, rospy.Duration(2.0))
            (trans, rot) = tflistener.lookupTransform("/map", "/base_footprint", now)


            minimum_distance = 0
            list_distance_objects = []
            choosen_item = None
            for item in objects_list:
                data_item = json.loads(item)

                x_data_item = data_item["pose"]["position"]["x"]
                y_data_item = data_item["pose"]["position"]["y"]

                item_distance = math.sqrt(pow(x_data_item-trans[0],2)+pow(y_data_item-trans[1],2))
                
                if minimum_distance == 0:
                    minimum_distance = item_distance
                    choosen_item = item
                else:
                    if item_distance < minimum_distance:
                        minimum_distance = item_distance
                        choosen_item = item

            return choosen_item

        except Exception as e:
            rospy.logerr("{class_name}: can not complete the function get closest object : %s".format(class_name=self.__class__.__name__),e)
            return objects_list



    def point_an_object(self,object_label):
        """
        Will send a motion order to point a specified object and if it's necessary, will send a navaigation to rotate 
        in order to have the best orientation to point the object.

        :param object_label: label of the target object
        :type object_label: string
        """
        
        response = self._lt_motion_palbator.point_at_object(object_label)
        rospy.logwarn("{class_name}: RESPONSE : %s".format(class_name=self.__class__.__name__),str(response))


        result = response.result
        rospy.logwarn("{class_name}: RESULT %s".format(class_name=self.__class__.__name__),str(result))
        if result.action_output != '':
            result_in_json = json.loads(result.action_output)
            if "rotationNeed" in result_in_json.keys():
                angle_rotation = float(result_in_json["rotationNeed"])
                self._lt_navigation.send_nav_rotation_order("NT", angle_rotation, 90.0)
                response = self._lt_motion_palbator.point_at_object(object_label)

                rospy.loginfo("{class_name}: RESULT %s".format(class_name=self.__class__.__name__),str(response.result))


    def turn_around_and_detect_someone(self,people_name):

        rospy.logwarn("{class_name} : I WILL TRY TO DETECT %s".format(class_name=self.__class__.__name__),people_name)
        self.people_detected = False
        response = self._lt_perception.detect_meta_people_from_img_topic(timeout=10)
        result = response.payload
        rospy.logwarn("RESULT : %s",str(result))

        if result != None and result != {}:
            detection = result.peopleMetaList.peopleList
            rospy.logwarn("DETECTION : %s",str(detection))
            for people in detection:
                if people.label_id == people_name:
                    self.people_detected = True
                    pose = people.pose
                    rospy.logerr("fbhzebunrvineriob,eirb,ioerb,roe,bioer,bpoer;plgpkerojgeorgkoerkgoerkgoerger")
                    break
        
        cp = 0
        while self.people_detected == False or cp<8:
            cp = cp + 1
            rotation_angle = math.pi/4.0
            response_nav = self._lt_navigation.send_nav_rotation_order("NT", rotation_angle , 90.0)
            response = self._lt_perception.detect_meta_people_from_img_topic(timeout=10)
            result = response.payload
            rospy.logwarn("RESULT : %s",str(result))
            if result != None and result != {}:
                detection = result.peopleMetaList.peopleList
                rospy.logwarn("DETECTION : %s",str(detection))
                    for people in detection:
                        if people.label_id == people_name:
                            self.people_detected = True
                            pose = people.pose
                            rospy.logerr("fbhzebunrvineriob,eirb,ioerb,roe,bioer,bpoer;plgpkerojgeorgkoerkgoerkgoerger")
                            break
        
        if self.people_detected == False:
            rospy.logerr("I can not detect %s",people_name)
            return "NO DETECTION"
        else:
            now = rospy.Time(0)
            object_point = PointStamped()
            object_point.header.frame_id = "palbator_arm_kinect_link"
            object_point.header.stamp = now
            object_point.point.x = pose.position.x
            object_point.point.y = pose.position.y
            object_point.point.z = pose.position.z
            rospy.loginfo("{class_name} : Object coords in palbator_arm_kinect_link : %s".format(class_name=self.__class__.__name__),str(object_point))
            listener.waitForTransform("/base_footprint", "/palbator_arm_kinect_link", now, rospy.Duration(20))
            target = listener.transformPoint("/base_footprint",object_point)

            rospy.loginfo("{class_name} : Object coords in base_footprint : %s".format(class_name=self.__class__.__name__),str(target))

            alpha = np.arctan(target.point.y/target.point.x)

            rospy.logerr("ALPHA : %s",str(alpha))
            rospy.logerr("ALPHA DEGRES : %s",str((alpha*360)/(2*math.pi)))
            response_nav = self._lt_navigation.send_nav_rotation_order("NT", alpha , 90.0) 
            return "PEOPLE DETECTED"


