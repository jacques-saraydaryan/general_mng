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
        self.configure_intern()

        #Inform configuration is ready
        self.configurationReady = True
        rospy.loginfo("High Level Behaviours API initialized")

    #######################################
    # CONFIGURATION
    ######################################

    def configure_intern(self):

        rospy.loginfo("Initiating High Level Behaviours API ... ")

        self._lt_perception = LTPerception()
        self._lt_navigation = LTNavigation()
        self._lt_motion_palbator = LTMotionPalbator()
        

    def reset(self):
        self.configure_intern()

    #######################################
    # HIGH LEVEL BEHAVIOURS 
    ######################################

    def turn_around_and_detect_objects(self, room_to_inspect, nav_timeout):
        try:
            for i in range(0,8):
                rotation_angle = math.pi/4.0
                rospy.loginfo("ROTATION %s of %s radians",str(i),str(rotation_angle))
                response_nav = self._lt_navigation.send_nav_rotation_order("NT", rotation_angle , nav_timeout)

            response = self._lt_perception.get_object_in_room(room_to_inspect)
            objects_list = response.payload
            rospy.logwarn("OBJECTS IN ROOM %s",str(objects_list))

            if len(objects_list) != 0:
                closest_object = self.get_closest_object(objects_list)
                return closest_object
            else:
                return None

        except Exception as e:
            rospy.logwarn("###### TURN AROUND AND DETECT OBJECT FAILURE , State: %s", str(e))
            return None

    
    def get_closest_object(self,objects_list):
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
            rospy.logerr(e)
            return objects_list



    def point_an_object(self,object_label):
        
        response = self._lt_motion_palbator.point_at_object(object_label)
        rospy.logwarn("RESPONSE : %s",str(response))


        result = response.result
        rospy.logwarn(result)
        if result.action_output != '':
            result_in_json = json.loads(result.action_output)
            if "rotationNeed" in result_in_json.keys():
                angle_rotation = float(result_in_json["rotationNeed"])
                self._lt_navigation.send_nav_rotation_order("NT", angle_rotation, 90.0)
                response = self._lt_motion_palbator.point_at_object(object_label)

                rospy.loginfo(response.result)



