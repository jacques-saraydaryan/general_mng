#!/usr/bin/env python
__author__ = 'Florian DUPUIS'
import rospy

from AbstractScenario import AbstractScenario
from meta_lib.LTPerception import LTPerception
from meta_lib.LTNavigation import LTNavigation
from meta_lib.LTMotion import LTMotionPalbator

from meta_behaviour.LTHighBehaviour import LTHighBehaviour

import json
import time
from copy import deepcopy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus
from convert_2d_to_3d.srv import SwitchMode
from object_management.srv import ConfigureFLowSwitcherService
from tf import TransformListener

class Robocup_simu_scenario(AbstractScenario):
    DEFAULT_TIMEOUT = 5.0
    NO_TIMEOUT = -1.0

    def __init__(self,config,scenario_path_folder):
        """
        Initializes the scenario Robocup_simu and receives the needed parameters to run the scenario.
        :param config: contains all the data of the scenario : steps list, parameters etc ...
        :type config: dict
        :param scenario_path_folder: path where is stored the JSON scenario file
        :type scenario_path_folder: string
        """

        self._scenario_path_folder = scenario_path_folder

        self._scenario=config
        self._nav_strategy = {
            "action": "NP",
            "mode": "CRRCloseToGoal",
            "timeout": 90.0
        }

        self.allow_perception = True
        self.allow_navigation = True
        self.allow_highbehaviour = True
        self.allow_motion = True
        self.allow_simulation = True

        if self.allow_perception:
            self._lt_perception = LTPerception()

        if self.allow_navigation:
            self._lt_navigation = LTNavigation()

        if self.allow_highbehaviour:
            if self.allow_simulation:
                self._lt_high_behaviour = LTHighBehaviour(execution_mode="simulation")
            else:
                self._lt_high_behaviour = LTHighBehaviour(execution_mode="real")


        if self.allow_motion:
            self._lt_motion = LTMotionPalbator()

        self._locations = self._scenario['imports']['locations']
        self._objects = self._scenario['imports']['objects']

        self.detected_object = None
        self.configuration_ready = True

        service_name ="merge_register_data_switch_config"
        self.switch_config = rospy.ServiceProxy(service_name, SwitchMode)
        self.switch_config(register_or_grap_mode = 0, category_filter_tag_list="*")
        service_name_2 ="merge_flow_config_service"
        self.switch_camera = rospy.ServiceProxy(service_name_2, ConfigureFLowSwitcherService)
        self.switch_camera(flow_list =["/camera/color/image_raw"], switch_period = 1)

        self.listener = TransformListener()

        self.zone = 'Room_1'
        self.current_itp = ''

    def start_scenario(self):   

        self.current_index_scenario=0
        self.scenario_end = False

        while self.scenario_end == False and not rospy.is_shutdown():
            i = 1
            while self.detected_object == None and i<4:
                self.go_To('Perception_3_'+str(i))
                self.find_object()
                i+=1
            self.catch_object("Catch XYZ")
            self.go_To('Left_Person')
            self.scenario_end = True

    
    def go_To(self, Itp):

        rospy.loginfo("{class_name} ACTION GO TO %s".format(class_name=self.__class__.__name__), Itp)
        
        if self.allow_motion:
            self._lt_motion.set_palbator_ready_to_travel()
            rospy.logwarn("{class_name}: MOTION SET".format(class_name=self.__class__.__name__))

        if self.allow_navigation:
            rospy.logwarn("{class_name}: SEND NAV".format(class_name=self.__class__.__name__))
            self._lt_navigation.send_nav_order(self._nav_strategy['action'], self._nav_strategy['mode'], Itp, self._nav_strategy['timeout'])
            self.current_itp = Itp
        else:
            rospy.logwarn("{class_name}: NAV GOAL TO : ".format(class_name=self.__class__.__name__) + Itp + " ACTION "+self._nav_strategy['action'] +" MODE "+self._nav_strategy['mode'] + " itp " + Itp + " timeout "+ str(self._nav_strategy['timeout']))    
            time.sleep(2)

    def find_object(self):
        """
        Function dealing with the find action. The robot finds something or someone.
        """
        rospy.loginfo("{class_name} ACTION FOUND OBJECT".format(class_name=self.__class__.__name__))

        ############################# ACTION CLIENT DETECTION OBJET ICI

        detection = ''
        if self.allow_perception:
            #Filtre pour le retour en simulation
            category_filter = '*'
            response = self._lt_perception.get_object_in_room(self.current_zone(), category_filter)
            objects_list = response.payload
            objects_names_list = []
            for obj in objects_list:
                objects_names_list.append(obj.type)
            rospy.logwarn("{class_name}: OBJECTS IN ROOM %s".format(class_name=self.__class__.__name__),str(objects_names_list))

        else:
            objects_list = []

        if self.allow_highbehaviour:

            # if len(objects_list) < 2:
                number_of_rotation = 1
                if self.allow_perception and self.allow_navigation:
                    rospy.sleep(2)
                    detection_result = None
                    #self._lt_navigation.send_nav_order_to_pt('RN', 'CloseToObject', 2.42, 4.68, self._nav_strategy['timeout'])
                    for mouvements in range(1,3):
                        tentatives = 0
                        while detection_result == None and tentatives < 2:
                            tentatives += 1
                            detection_result = self._lt_high_behaviour.turn_around_and_detect_objects(self.zone, number_of_rotation, self._nav_strategy['timeout'])
                            if not detection_result is None:
                                rospy.loginfo("{class_name}: DETECTION RESULT %s".format(class_name=self.__class__.__name__),detection_result)
                                #object_label = detection_result.label+'_TF'
                                for item in self._objects:
                                    if item['id'] in detection_result.type:
                                        detection = item['id']
                                self.detected_object_TF = detection_result.uuid+'_TF'
                                self.detected_object = detection_result.type
                                self.detected_object_coord_x = detection_result.pose.position.x
                                self.detected_object_coord_y = detection_result.pose.position.y
                                self.detected_object_coord_z = detection_result.pose.position.z 
                            else:
                                if tentatives < 2:
                                    rospy.logwarn("{class_name}: NO OBJECTS DETECTED IN %s".format(class_name=self.__class__.__name__),self.zone)
                                    rospy.logwarn("{class_name}: NEW TRY FAILED TRY NUM %s", tentatives)
                                    self.detected_object = None
                        #self._lt_navigation.send_nav_order_to_pt('RES', 'CloseToObject', 2.42, 4.68, self._nav_strategy['timeout'])
                else:
                    rospy.logwarn("{class_name} : Can't use the turn around and detect object behaviour".format(class_name=self.__class__.__name__))

    def current_zone(self):

        for location in self._locations:
                if location['name'] == self.current_itp:
                    self.zone = location['zone']
        return self.zone

    def catch_object(self, action):
        """
        Function dealing with an Object action. The robot will accomplish an action related to an object.
        """

        if action == "Catch XYZ":
            rospy.logwarn("CATCHING OBJECT")
            result = {"status":""}
            tentatives = 0
            self.switch_config(register_or_grap_mode = 1, category_filter_tag_list="*")
            while result["status"] != "Success" and tentatives < 5:
                if tentatives != 0:
                    nav_strategy = 'RES'
                    # coord_x = trans[0]
                    # coord_y = trans[1]
                else:
                    nav_strategy = self._nav_strategy['action']
                    coord_x = self.detected_object_coord_x
                    coord_y = self.detected_object_coord_y
                    coord_z = self.detected_object_coord_z
                self._lt_motion.set_palbator_ready_to_travel()
                try:
                    self._lt_navigation.send_nav_order_to_pt(nav_strategy, 'CloseToObject', coord_x, coord_y, self._nav_strategy['timeout'])
                    self._lt_motion.look_at_object(self.detected_object_TF)
                    #self.switch_camera(flow_list =["/camera/color/image_raw"], switch_period = 1)
                    # self.listener.waitForTransform("map", 'Target_TF_0', rospy.Time(0), rospy.Duration(5))
                    # (trans, rot) = self.listener.lookupTransform("map", 'Target_TF_0', rospy.Time(0))
                    #result = self._lt_motion.catch_object_XYZ(trans[0],trans[1],trans[2])
                    dimensions = []
                    for item in self._objects:
                        if item['id'] == self.detected_object:
                            dimensions = item['dimensions']
                    result = self._lt_motion.catch_object_XYZ(coord_x,coord_y, coord_z, dimensions)
                    result = json.loads(result.result.action_output)
                except rospy.ROSInterruptException:
                    rospy.loginfo("{class_name} ACTION FAILED, NEW TRY")
                tentatives+=1
            self.switch_config(register_or_grap_mode = 0, category_filter_tag_list="*")
           
        if action == "Catch Label":
            rospy.logwarn("CATCHING OBJECT")
            self.switch_config(register_or_grap_mode = 1, category_filter_tag_list="*")
            self._lt_navigation.send_nav_order_to_pt(self._nav_strategy['action'], self._nav_strategy['mode'],self.detected_object_coord_x,self.detected_object_coord_y, self._nav_strategy['timeout'])
            self._lt_motion.catch_object_XYZ(self.detected_object_coord_x, self.detected_object_coord_y, self.detected_object_coord_z)
            self.switch_config(register_or_grap_mode = 0, category_filter_tag_list="*")

        if action == "Dropping Label":
            rospy.logwarn("DROPPING OBJECT")
            self._lt_navigation.send_nav_order(self._nav_strategy['action'], self._nav_strategy['mode'],"GreenBac", self._nav_strategy['timeout'])
            self._lt_motion.dropping_label("GreenBac")

        if action == "Dropping XYZ":
            rospy.logwarn("DROPPING OBJECT")
            self._lt_motion.dropping_XYZ(1, -1.3 , 0.522383)
        # result = {
        #     "NextIndex": stepIndex+1
        # }
        return result


if __name__ == '__main__':
    
    scenario = Robocup_simu_scenario()
    scenario.scenario_launcher()
    rospy.spin()
