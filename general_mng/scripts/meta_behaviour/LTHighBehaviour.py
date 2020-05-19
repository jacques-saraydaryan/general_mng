#!/usr/bin/env python
__author__ = 'Thomas CURE'

from abc import ABCMeta, abstractmethod
from meta_lib.LTAbstract import LTAbstract

from meta_lib.LTPerception import LTPerception
from meta_lib.LTNavigation import LTNavigation
from meta_lib.LTMotion import LTMotionPalbator
import rospy
import json

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

    def turn_around_and_detect_objects(self, perception_objects_labels, nb_max_detection, nav_timeout):

        detection_list = []
        cp_detection = 1
        try:
            while len(detection_list) == 0 and cp_detection < nb_max_detection:
                
                rotation_angle = float((2*math.pi)/float(nb_max_detection))

                rospy.logwarn("ROTATION %s de %s degres",str(cp_detection),str((rotation_angle*360)/(2*math.pi)))

                response_nav = self._lt_navigation.send_nav_rotation_order("NT", rotation_angle , 90.0)

                response_perception = self._lt_perception.detect_objects_with_given_sight_from_img_topic(perception_objects_labels,NO_TIMEOUT)
                
                detection_list = response_perception.payload.labelList
                
                cp_detection = cp_detection + 1
            
            if len(detection_list) == 0:
                rospy.loginfo("NO OBJECTS DETECTED AFTER TURNING AROUND") 
            else:
                rospy.loginfo("OBJECTS DETECTED "+str(detection_list))

            return detection_list           

        except Exception as e:
            rospy.logwarn("###### TURN AROUND AND DETECT OBJECT FAILURE , State: %s", str(e))
            return None

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



