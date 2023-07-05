__author__ = 'Jacques Saraydaryan'
import rospy
import os
import json
from copy import deepcopy
from AbstractScenario import AbstractScenario
from nav_msgs.msg import Path, Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tools.tf_tools import transform_pose

from meta_lib.LTHriManager import LTHriManagerPalbator
from meta_lib.LTNavigation import LTNavigation
from meta_lib.LTPerception import LTPerception
from meta_lib.LTServiceResponse import LTServiceResponse
import time
import math


class ReceptionistScenarioV0(AbstractScenario):
    DEFAULT_TIMEOUT = 5.0
    NO_TIMEOUT = -1.0
    YOLO_CLASS_CHAIR= 'chair'
    YOLO_CLASS_COUCH= 'couch'
    _severalActionPending = {}
    _oneActionPending = None

    def __init__(self,config,scenario_path_folder):
        self.configuration_ready = False
        self.init_scenario(config,scenario_path_folder)

        # get odometry
        #rospy.Subscriber("/odom", Odometry, self.odomCallback)
        rospy.on_shutdown(self.onShutDown)
        

    def init_scenario(self, config,scenario_path_folder):

        self._scenario_path_folder = scenario_path_folder
        self._scenario=config
        self.current_dir = os.path.dirname(os.path.abspath(__file__))

        
        _name_action_server_HRI = self._scenario['parameters']['LTHri_action_server_name']
        self._nav_strategy = self._scenario['parameters']['nav_strategy_parameters']
        debug_variables = self._scenario['parameters']['debug_variables']

        self._drink_list = self._scenario['imports']['drink']
        self._person_list = self._scenario['imports']['person']
        
        self._lm_wrapper = LTHriManagerPalbator(_name_action_server_HRI)
        self._lt_navigation = LTNavigation()
        self._lt_perception = LTPerception()

        
        rospy.loginfo("{class_name}: JSON FILES LOADED.".format(class_name=self.__class__.__name__))

        self.configuration_ready = True
        

    def start_scenario(self):
        """
        Runs the scenario according to the scenario JSON file
        """
        rospy.loginfo("""
        ######################################
        Starting the %s Scenario...
        ######################################
        """%str(self._scenario["name"]))

        # Wait door opening
        #result = self.lt_perception.wait_for_door_to_open()
        #self.print_result(result)

        self.manual_scenario()

    def manual_scenario(self):
        #self.steps = deepcopy(self._scenario["steps"])

        rospy.loginfo("{class_name} : WAITING FOR HRI ACTION SERVER ACTIVATION".format(class_name=self.__class__.__name__))
        self._lm_wrapper.client_action_GmToHri.wait_for_server()

        self._lm_wrapper.restart_hri(self.NO_TIMEOUT)

        self.steps = [
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'WaitToStarts', 'name': 'Wait to start', 'order': 1},
                        {'action': '', 'arguments': {}, 'eta': 60, 'id': 'MoveLoc1', 'name': 'Move to the door', 'order': 2},
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'CheckPeople1', 'name': 'Check People presence', 'order': 3},
                        {'action': '', 'arguments': {}, 'eta': 60, 'id': 'AskPeople1', 'name': 'Ask people name and preference', 'order': 4},
                        {'action': '', 'arguments': {}, 'eta': 60, 'id': 'MoveLoc2', 'name': 'Move to the party', 'order': 5},
                        {'action': '', 'arguments': {}, 'eta': 60, 'id': 'PresentTheGuest', 'name': 'Present new member', 'order': 6},
                        {'action': '', 'arguments': {}, 'eta': 60, 'id': 'FindOtherAndPlace', 'name': 'Ask new member to sit', 'order': 7},

        ]

        #Update HRI Board with tasks list
        self._lm_wrapper.timeboard_send_steps_list(self.steps, self._scenario["name"], self.NO_TIMEOUT)


        #Way Door Opened
        result = self._lm_wrapper.generic_global("WaitToStarts","Wait an opened door",30,"I am waiting an opened door to start",
                        description="I am waiting an opened door to start",
                        wait_answer=False,
                        need_confirmation=False,
                        need_validation=False,
                        media_src="/img/hri/rob2.jpg",
                        media_type="img"
                        )
        result = self._lt_perception.wait_for_door_to_open()
        #rospy.sleep(10)

        # Go to the party area
        result = self._lm_wrapper.generic_global("MoveLoc0","Move to entrance",60,"I am going to the entrance",
                        description="I am going to the entrance",
                        media_src="/img/hri/rob1.jpg", 
                        media_type="img", 
                        )
        result = self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","L_To_E",60)

        
        
        #Check People presence 
        result = self._lm_wrapper.generic_global("CheckPeople1","Check People presence",60,"I'm looking for a new guest !",
                        description="I'm looking for a new guest!",
                        media_src="/img/hri/robot-lookingat2.jpeg",
                        media_type="img",
                        )
        result = self._checkPeopleIsHere(timeout=2)
        if(result.status == result.FAILURE_STATUS):
            self.print_result(result)

        rospy.sleep(2)
        #Learn People Meta (far learn) 
        result = self._lm_wrapper.generic_global("AskPeople1","Ask people name and preference",30,"Hi ! Please wait some seconds I am going to memorize some information",
                        description="Please wait some seconds I am going to memorize some information",
                        media_src="/img/hri/learning-robot.jpg",
                        media_type="img",
                        )
        guest_info_list={}      
        result = self._lt_perception.learn_people_meta_from_img_topic("guest1",2)
        guest_info_list["guest1"]={}
        guest_info_list["guest1"]["MetaInfo"] = result
        rospy.sleep(5)

        #Ask and learn name
        options_drink= self._computeDrinkOptionFromJson(self._drink_list)
        options_user= self._computePeopleNameOptionFromJson(self._person_list)

        result = self._lm_wrapper.generic_global("AskPeople1","Ask people name and preference",30," Please can you come closer and say your name ?",
                        description="Please can you come clother and say your name ?",
                        type="question",
                        wait_answer=True,
                        need_confirmation=True,
                        need_validation=True,
                        options=options_user
                        )
        try:
            guest_info_list["guest1"]["name"]= result.payload['person'][0]
        except KeyError:
            rospy.WARN("Unable to get name from result")
        self.print_result(result)

        result = self._lm_wrapper.generic_global("AskPeople1","Ask people name and preference",30,"I register your data",
                        description="Data Registration",
                        media_src="/img/hri/learning-robot.jpg",
                        media_type="img",
                        )

        result = self._lm_wrapper.generic_global("AskPeople1","Ask people name and preference",30," Please can you select your prefer drink ?",
                        description="Please can you select your prefer drink ?",
                        type="question",
                        wait_answer=True,
                        need_confirmation=True,
                        need_validation=True,
                        options=options_drink
                        )
        
        try:
            guest_info_list["guest1"]["drink"]= result.payload['drink'][0]
        except KeyError:
            rospy.WARN("Unable to get drink from result")

        # Go to the party area
        result = self._lm_wrapper.generic_global("MoveLoc2","Move to the party",60,"Please follow me, I will present you other guests",
                        description="Please follow me,I'm going to the location to meet other guests",
                        media_src="/img/hri/rob1.jpg", 
                        media_type="img", 
                        )
        result = self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","L4",60)

        #Present the Guest 2 to Guest1
        result = self._lm_wrapper.generic_global("PresentTheGuest","Present new member",60,"Hi john, here a new guest " + guest_info_list["guest1"]["name"]+", his prefer drink is "+guest_info_list["guest1"]["drink"],
                        description="Hi john, here a new guest " + guest_info_list["guest1"]["name"]+", his prefer drink is "+guest_info_list["guest1"]["drink"],
                        media_src="/img/hri/person/user.png", 
                        media_type="img", 
                        )
        rospy.sleep(10)
        #FIXME need to check the rotation direction
        self._lt_navigation.send_nav_rotation_order("NT", math.pi / float(2) ,20)
        
        #Present the Guest 1 to Guest2
        result = self._lm_wrapper.generic_global("PresentTheGuest","Present new member",60,guest_info_list["guest1"]["name"] +" , Here it is John, his prefer drink is milk",
                        description=guest_info_list["guest1"]["name"] +" , Here it is John, his prefer drink is milk",
                        media_src="/img/hri/person/user.png", 
                        media_type="img", 
                        )
        
        rospy.sleep(10)
        #FIXME need to check the rotation direction
        self._lt_navigation.send_nav_rotation_order("NT", - math.pi / float(2) ,20)
        #OR navigate to point close to the couch

        rospy.sleep(2)

        # Go to the couch point at pose
        #result = self._lm_wrapper.generic_global("FindOtherAndPlace","Ask new member to sit",60," localize free space",
        #                description="localize free space",
        #                media_src="/img/hri/rob1.jpg", 
        #                media_type="img", 
        #                )
        #result = self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","L5",20)
        

        #Present the free Space
        result = self._lm_wrapper.generic_global("FindOtherAndPlace","Ask new member to sit",30, guest_info_list["guest1"]["name"]+", you can sit on the couch here" ,
                        description=guest_info_list["guest1"]["name"]+", you can sit on the couch here",
                        media_src="/img/hri/person/user.png", 
                        media_type="img", 
                        )

        self.print_result(result)
        
    def onShutDown(self):
        rospy.loginfo(""" SHUTDOWN ASKED Clear action server before ending """)
        self._lm_wrapper.cancel_all_actions_HRI_mng


#******************************************************************************************
#*************************************   Tools   **************************************
#******************************************************************************************


    def _checkPeopleIsHere(self,timeout=20):
        people_detected = False
        start_time = time.time()
        #Until a person is detected
        while(not people_detected and time.time()-start_time < timeout):
            result = self._lt_perception.detect_meta_people_from_img_topic(30);
            if(result.status == result.SUCCESS_STATUS):
                if(len(result.payload.peopleMetaList.peopleList)>0):
                    people_detected = True
                    return result
        response = LTServiceResponse(current_status="FAILURE", current_msg="No people detected, timeout")
        return response

    def _computeDrinkOptionFromJson(self, json_drink_list):
        options=[]
        for item in json_drink_list:
            options.append({'value': item['name'],'media_src': item['pathOnTablet'], 'type':'drink','media_type': 'img'})
        return options
    

    def _computePeopleNameOptionFromJson(self, json_people_list):
        options=[]
        for item in json_people_list:
            options.append({'value': item['name'],'media_src': '/img/hri/person/user.png', 'type':'person','media_type': 'img'})
        return options
    
