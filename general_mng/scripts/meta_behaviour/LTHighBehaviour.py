__author__ = 'Jacques Saraydaryan'

from abc import ABCMeta, abstractmethod
from LTAbstract import LTAbstract

from meta_lib.LTPerception import LTPerception
from meta_lib.LTNavigation import LTNavigation
import rospy

class LTHighBehaviour(LTAbstract):

    _enableHighBehaviour = True

    def __init__(self):
        self.configure_intern()

        #Inform configuration is ready
        self.configurationReady = True

    #######################################
    # CONFIGURATION
    ######################################

    def configure_intern(self):

        rospy.loginfo("Initiating High Level Behaviours API ... ")

        self._lt_perception = LTPerception()
        self._lt_navigation = LTNavigation()

        rospy.loginfo("High Level Behaviours API initialized")

    def reset(self):
        self.configure_intern()

    #######################################
    # HIGH LEVEL BEHAVIOURS 
    ######################################

    def turn_around_and_detect_objects(self, perception_objects_labels, nb_max_detection, nav_timeout):

        detection_list = []
        cp_detection = 0
        try:
            while len(detection_list) == 0 and cp_detection < nb_max_detection:

                response_nav = self._lt_navigation.send_nav_order("NT","","",nav_timeout)

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


