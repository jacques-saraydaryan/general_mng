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
import math
from std_msgs.msg import String
from convert_2d_to_3d.srv import SwitchMode
from object_management.srv import ConfigureFLowSwitcherService
from robocup_launcher.srv import MessageParserSrv

from tf import TransformListener
from std_msgs.msg import Int16
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from world_manager.srv import getNamoEntity

class Robocup_Cleanup(AbstractScenario):

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

        # META LIB

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

        # Scenario Infos

        self._locations = self._scenario['imports']['locations']
        self._objects = self._scenario['imports']['objects']

        # Services
        service_name ="merge_register_data_switch_config"
        self.switch_config = rospy.ServiceProxy(service_name, SwitchMode)
        self.switch_config(register_or_grap_mode = 0, category_filter_tag_list="*")
        service_name_2 ="merge_flow_config_service"
        self.switch_camera = rospy.ServiceProxy(service_name_2, ConfigureFLowSwitcherService)
        self.switch_camera(flow_list =["/fake_camera"], switch_period = 1)

        # namo services
        self._makePlan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        self._getNamoEntity = rospy.ServiceProxy('/get_namo_entity', getNamoEntity)

        self.message_parser = rospy.ServiceProxy("/message_parser", MessageParserSrv)

        self.listener = TransformListener()

        # Declare attributes
        self.zone = 'Room_1'
        self.current_itp = ''
        
        self.objets_ponderes = ['mustard', 'tomatosoup', 'pottedmeat', 'sugar', 'coffee', 'bleach', 'mug', 'cup', 'cracker']
        self.detection_result = []
        self.grasp_message = False
        self.detected_object = ''


        self.configuration_ready = True

    def start_scenario(self):   

        self.current_index_scenario=0
        self.scenario_end = False

        while self.scenario_end == False and not rospy.is_shutdown():

            self.perception_phase('Table_Perception')
            self.return_objects()
            if self.detection_result != [] :
                grasp = self.get_closest_object(self.detection_result)
                self.actualise_detected_obj(grasp)
                self.go_To('Table_Grasp')
                self.catch_object()

            self.perception_phase('Perception_2')
            self.perception_phase('Room_1')
            self.perception_phase('Perception_0')

            self.grasping_pondere()
            self.scenario_end = True

    
    def go_To(self, Itp):

        rospy.loginfo("{class_name} ACTION GO TO %s".format(class_name=self.__class__.__name__), Itp)
        
        if self.allow_motion:
            self._lt_motion.set_palbator_ready_to_travel()
            rospy.logwarn("{class_name}: MOTION SET".format(class_name=self.__class__.__name__))

        if self.allow_navigation:
            rospy.logwarn("{class_name}: SEND NAV".format(class_name=self.__class__.__name__))
            result = self._lt_navigation.send_nav_order(self._nav_strategy['action'], self._nav_strategy['mode'], Itp, self._nav_strategy['timeout'])
            self.current_itp = Itp
        else:
            rospy.logwarn("{class_name}: NAV GOAL TO : ".format(class_name=self.__class__.__name__) + Itp + " ACTION "+self._nav_strategy['action'] +" MODE "+self._nav_strategy['mode'] + " itp " + Itp + " timeout "+ str(self._nav_strategy['timeout']))    
            time.sleep(2)
            result = None
        return result

    def return_objects(self):
        response = self._lt_perception.get_object_in_room(self.current_zone(), self.objets_ponderes)
        self.detection_result = response.payload
        for object in self.detection_result:
            if object.pose.position.z < 0.2:
                self.detection_result.remove(object)

        rospy.loginfo("{class_name}: DETECTION RESULT ON TABLE %s".format(class_name=self.__class__.__name__),self.detection_result)

    def find_object(self):
        """
        Function dealing with the find action. The robot finds something or someone.
        """
        rospy.loginfo("{class_name} ACTION FIND OBJECT".format(class_name=self.__class__.__name__))

        ############################# ACTION CLIENT DETECTION OBJET ICI
        self.switch_camera(flow_list =["/camera/color/image_raw","/camera_base/color/image_raw"], switch_period = 2)

        if self.allow_highbehaviour:
                if self.allow_perception and self.allow_navigation:
                    rospy.sleep(6)
                    tentatives = 0
                    while self.detection_result == [] and tentatives < 3:
                        tentatives += 1
                        category_filter = ['mustard', 'tomatosoup', 'pottedmeat', 'sugar', 'coffee', 'cracker']
                        response = self._lt_perception.get_object_in_room(self.current_zone(), category_filter)
                        self.detection_result = response.payload
                        rospy.loginfo("{class_name}: DETECTION RESULT %s".format(class_name=self.__class__.__name__),self.detection_result)
                        if self.detection_result != []:
                            if self.darknet_object_message != 'undefined' and self.darknet_object_message != 'pending':
                                for el in self.detection_result:
                                    if el.type == self.darknet_object_message:
                                        self.actualise_detected_obj(el)
                                        self.grasp_message = True
                                        return
                        else:
                            if tentatives < 2:
                                rospy.logwarn("{class_name}: NO OBJECTS DETECTED IN %s".format(class_name=self.__class__.__name__),self.zone)
                                rospy.logwarn("{class_name}: NEW TRY FAILED TRY NUM %s", tentatives)
                else:
                    rospy.logwarn("{class_name} : Can't use the turn around and detect object behaviour".format(class_name=self.__class__.__name__))

        self.switch_camera(flow_list =["/fake_camera"], switch_period = 1)

    def current_zone(self):

        for location in self._locations:
                if location['name'] == self.current_itp:
                    self.zone = location['zone']
        return self.zone

    def catch_object(self):
        """
        Function dealing with an Object action. The robot will accomplish an action related to an object.
        """
        rospy.logwarn("CATCHING OBJECT")
        result = {"status":""}
        tentatives = 0
        self.switch_config(register_or_grap_mode = 1, category_filter_tag_list="*")
        while result["status"] != "Success" and tentatives < 2:
            self._lt_motion.set_palbator_ready_to_travel()
            if tentatives != 0:
                nav_strategy = 'RES'
            else:
                coord_x = self.detected_object_coord_x
                coord_y = self.detected_object_coord_y
                coord_z = self.detected_object_coord_z
                nav_strategy = self._nav_strategy['action']
            try:
                self._lt_navigation.send_nav_order_to_pt(nav_strategy, 'CloseToObject', coord_x, coord_y, self._nav_strategy['timeout'])
                dimensions = []
                for item in self._objects:
                    if item['id'] == self.detected_object:
                        dimensions = item['dimensions']
                rospy.logwarn("CATCHING OBJECT: %s", self.detected_object)
                result = self._lt_motion.catch_object_XYZ(coord_x,coord_y, coord_z, dimensions)
                result = json.loads(result.result.action_output)
            except rospy.ROSInterruptException:
                rospy.loginfo("{class_name} ACTION FAILED, NEW TRY")
            tentatives+=1
        self.switch_config(register_or_grap_mode = 0, category_filter_tag_list="*")
        return result

    def catch_object_table(self):
        """
        Function dealing with an Object action. The robot will accomplish an action related to an object.
        """
        rospy.logwarn("CATCHING OBJECT")
        result = {"status":""}
        tentatives = 0
        self.switch_config(register_or_grap_mode = 1, category_filter_tag_list="*")
        while result["status"] != "Success" and tentatives < 2:
            if tentatives == 0:
                coord_x = self.detected_object_coord_x
                coord_y = self.detected_object_coord_y
                coord_z = self.detected_object_coord_z
            try:
                dimensions = []
                for item in self._objects:
                    if item['id'] == self.detected_object:
                        dimensions = item['dimensions']
                rospy.logwarn("CATCHING OBJECT: %s", self.detected_object)
                result = self._lt_motion.catch_object_XYZ(coord_x,coord_y, coord_z, dimensions)
                result = json.loads(result.result.action_output)
            except rospy.ROSInterruptException:
                rospy.loginfo("{class_name} ACTION FAILED, NEW TRY")
            tentatives+=1
        self.switch_config(register_or_grap_mode = 0, category_filter_tag_list="*")
        return result

    def actualise_detected_obj(self, detected_obj):
        self.detected_object_TF = detected_obj.uuid+'_TF'
        self.detected_object = detected_obj.type
        self.detected_object_coord_x = detected_obj.pose.position.x
        self.detected_object_coord_y = detected_obj.pose.position.y
        self.detected_object_coord_z = detected_obj.pose.position.z

    def grasping_pondere(self):
        for obj in self.detection_result:
            
            if obj.type in self.objets_ponderes:
                self.actualise_detected_obj(obj)
                result = self.catch_object()
                if result['status'] == 'Success':
                    return True
        return False

    def NAMO(self):
        free = False
        while not free:
            self.go_To('NAMO_Observ')
            # result = self._lt_navigation.send_nav_order(self._nav_strategy['action'], 'Simple', 'Perception_3_2', self._nav_strategy['timeout'])
            start = self.getRobotPose()
            goal = self.getItp()
            rospy.loginfo("{class_name} GOT POSE AND ITP")
            current_plan = self._makePlan(start, goal, 0.2)
            if len(current_plan.plan.poses) == 0:
                # rospy.logerr("GOOD")
                rospy.loginfo("{class_name} PROCESS NAMO")
                self.processNamo()
            else:
                free = True
        
    def processNamo(self):
        entity = self._getNamoEntity(2.0, 3.0, 3.15, 2.0, 0.1, 1, 100).entity
        outEntity = []
        for i in range(len(entity)):
            if entity[i].position.x > 2.64:
                continue
            else:
                outEntity.append(entity[i])
        minY = 10
        selectedEntity = ''
        for e in outEntity:
            if e.position.y < minY:
                minY = e.position.y
                selectedEntity = e
        if selectedEntity != '':
            self._lt_navigation.send_nav_order_to_pt("RN", 'CloseToObject', selectedEntity.position.x, selectedEntity.position.y, self._nav_strategy['timeout'])
            self._lt_motion.namo_XYZ(selectedEntity.position.x, selectedEntity.position.y)

    def getRobotPose(self):
        self.listener.waitForTransform("/base_link", "/map", rospy.Time(0), rospy.Duration(5.0))
        robot_p = PoseStamped()
        robot_p.header.frame_id = "/base_link"
        robot_p.pose.position.x = 0
        robot_p.pose.position.y = 0
        robot_p.pose.position.z = 0
        robotPose = self.listener.transformPose("map", robot_p)
        return robotPose

    def getItp(self):
        self.listener.waitForTransform("map", "Perception_3_2_TF", rospy.Time(0), rospy.Duration(5))
        (trans, rot) = self.listener.lookupTransform("map", "Perception_3_2_TF", rospy.Time(0))

        poseStamped = PoseStamped()
        poseStamped.header.frame_id = "map"
        poseStamped.header.stamp = rospy.Time(0)
        poseStamped.pose.position.x = trans[0]
        poseStamped.pose.position.y = trans[1]
        poseStamped.pose.position.z = trans[2]
        poseStamped.pose.orientation.x = rot[0]
        poseStamped.pose.orientation.y = rot[1]
        poseStamped.pose.orientation.z = rot[2]
        poseStamped.pose.orientation.w = rot[3]

        return poseStamped

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
            choosen_item = None
            for item in objects_list:

                x_data_item = item.pose.position.x
                y_data_item = item.pose.position.y

                item_distance = math.sqrt(pow(x_data_item-trans[0],2)+pow(y_data_item-trans[1],2))
                
                if minimum_distance == 0:
                    minimum_distance = item_distance
                    choosen_item = item
                else:
                    if item_distance < minimum_distance:
                        minimum_distance = item_distance
                        choosen_item = item

            return choosen_item
        except:
            rospy.loginfo("{class_name} COULDN'T GET CLOSEST OBJECT")

    def perception_phase(self, Itp):
        self.go_To(Itp)
        if Itp == "Table_Perception":
            self._lt_motion.look_at_object_XYZ(1.4, 1.9, 0.9)
        rospy.loginfo("{class_name} GETTING OBJECTS TFs")
        self.switch_camera(flow_list = ["/camera/color/image_raw"], switch_period = 1)
        rospy.sleep(15)
        self.switch_camera(flow_list = ["/fake_camera"], switch_period = 1)

