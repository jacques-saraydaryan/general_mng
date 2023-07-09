__author__ = 'Jacques Saraydaryan'
import rospy
import os
import json
from AbstractScenario import AbstractScenario

from meta_lib.LTHriManager import LTHriManagerPalbator
from meta_lib.LTNavigation import LTNavigation
from meta_lib.LTPerception import LTPerception
from meta_lib.LTServiceResponse import LTServiceResponse
import time
import math


class CarryMyLuggageV0(AbstractScenario):
    DEFAULT_TIMEOUT = 5.0
    NAVIGATION_TIMEOUT = 6
    NAVIGATION_ROTATION_TIMEOUT = 3
    SIMPLE_SPEECH_TIMEOUT = 1

    # DEFAULT_TIMEOUT = 5.0
    TIMEOUT_NAVIGATION = 6 #60
    TIMEOUT_NAVIGATION_ROTATION = 3 #30
    TIMEOUT_SIMPLE_SPEECH = 1  #10
    TIMEOUT_SIMPLE_ACTION = 3  #30
    TIMEOUT_COMPLEX_ACTION = 6  #60
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
                        {'action': '', 'arguments': {}, 'eta': self.SIMPLE_SPEECH_TIMEOUT, 'id': 'CheckPeople1', 'name': 'Check People presence', 'order': 1},                        
                        {'action': '', 'arguments': {}, 'eta': self.SIMPLE_SPEECH_TIMEOUT, 'id': 'AskPeople1', 'name': 'Ask people name and preference', 'order': 2},
                        {'action': '', 'arguments': {}, 'eta': self.SIMPLE_SPEECH_TIMEOUT, 'id': 'PointAt', 'name': 'Ask to point at the bag', 'order': 3},
                        {'action': '', 'arguments': {}, 'eta': self.SIMPLE_SPEECH_TIMEOUT, 'id': 'LuggageConfirmation', 'name': 'Confirm bag selection', 'order': 4},
                        {'action': '', 'arguments': {}, 'eta': self.SIMPLE_SPEECH_TIMEOUT, 'id': 'MovePickingLoc', 'name': 'Move to the picking place', 'order': 5},
                        {'action': '', 'arguments': {}, 'eta': self.SIMPLE_SPEECH_TIMEOUT, 'id': 'ArmWarning0', 'name': 'Warn of arm movement Deux Machina', 'order': 6},
                        {'action': '', 'arguments': {}, 'eta': self.SIMPLE_SPEECH_TIMEOUT, 'id': 'ArmDeuxMachina', 'name': 'Take bag from operator', 'order': 7},
                        {'action': '', 'arguments': {}, 'eta': self.SIMPLE_SPEECH_TIMEOUT, 'id': 'ArmParking0', 'name': 'Parking the arm for following', 'order': 8},
                        {'action': '', 'arguments': {}, 'eta': self.SIMPLE_SPEECH_TIMEOUT, 'id': 'PrepareFollowing', 'name': 'Prepare the Following', 'order': 9},
                        {'action': '', 'arguments': {}, 'eta': self.SIMPLE_SPEECH_TIMEOUT, 'id': 'Following', 'name': 'Following operator', 'order': 10},
                        #TODO : recovery. x3 ? 
                        {'action': '', 'arguments': {}, 'eta': self.SIMPLE_SPEECH_TIMEOUT, 'id': 'ArmWarning1', 'name': 'Warn of arm movement', 'order': 11},
                        {'action': '', 'arguments': {}, 'eta': self.SIMPLE_SPEECH_TIMEOUT, 'id': 'ArmDeployment1', 'name': 'Give back arm movement', 'order': 12},
                        {'action': '', 'arguments': {}, 'eta': self.SIMPLE_SPEECH_TIMEOUT, 'id': 'ReturnBag', 'name': 'Return the bag to the operator', 'order': 13},
                        {'action': '', 'arguments': {}, 'eta': self.SIMPLE_SPEECH_TIMEOUT, 'id': 'Goodbye', 'name': 'Say Goodbye', 'order': 14},

                        # Bonus
                        # {'action': '', 'arguments': {}, 'eta': self.SIMPLE_SPEECH_TIMEOUT, 'id': 'FindOtherAndPlace', 'name': 'Ask new member to sit', 'order': 15},
                        # {'action': '', 'arguments': {}, 'eta': self.SIMPLE_SPEECH_TIMEOUT, 'id': 'FindOtherAndPlace', 'name': 'Ask new member to sit', 'order': 16},



        ]

        #Update HRI Board with tasks list
        self._lm_wrapper.timeboard_send_steps_list(self.steps, self._scenario["name"], self.NO_TIMEOUT)

        # self._callPointAt()

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

        # Navigate to living room
        result = self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","L_inspect",self.TIMEOUT_NAVIGATION)

        # Navigate to the bedroom
        result = self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","B_inspect",self.TIMEOUT_NAVIGATION)


        #Check People presence 
        result = self._lm_wrapper.generic_global("CheckPeople1","Check People presence",self.SIMPLE_SPEECH_TIMEOUT," I am looking for the operator !",
                        description="I am looking for the operator!",
                        media_src="/img/hri/robot-lookingat2.jpeg",
                        media_type="img",
                        )
        result = self._checkPeopleIsHere(timeout=2)
        if(result.status == result.FAILURE_STATUS):
            self.print_result(result)

        rospy.sleep(2)
        #Learn People Meta (far learn) 
        result = self._lm_wrapper.generic_global("AskPeople1","Ask people name and preference",self.SIMPLE_SPEECH_TIMEOUT,"Hello ! Can you stand in front of me for a few seconds so that I can record some information about you.",
                        description="Hello ! Can you stand in front of me for a few seconds so that I can record some information about you.",
                        media_src="/img/hri/learning-robot.jpg",
                        media_type="img",
                        )
        guest_info_list={}      
        result = self._lt_perception.learn_people_meta_from_img_topic("guest1",2)
        guest_info_list["guest1"]={}
        guest_info_list["guest1"]["MetaInfo"] = result
        rospy.sleep(5)



        # Point at
        result = self._lm_wrapper.generic_global("PointAt","Ask to point at the bag",self.SIMPLE_SPEECH_TIMEOUT,"I am ready to carry your luggage. Can you point at the luggage you want me to carry with you finger ?",
                        description="I am ready to carry your luggage. Can you point at the luggage you want me to carry with you finger ?",
                        media_src="/img/hri/rob1.jpg", 
                        media_type="img", 
                        )
        rospy.sleep(2)

        self._callPointAt()


        # Point at confirmation
        result = self._lm_wrapper.generic_global("LuggageConfirmation","Confirm bag selection",self.SIMPLE_SPEECH_TIMEOUT,"Is this the luggage you want me to carry ? Please confirm by yes or no.",
                        description="Is this the luggage you want me to carry ? Please confirm by yes or no.",
                        wait_answer=True,
                        need_confirmation=False,
                        need_validation=True,
                        media_src="/img/hri/suitcase.jpg", 
                        media_type="img", 
                        )
        rospy.sleep(2)
        

        # Go to the picking place
        result = self._lm_wrapper.generic_global("MovePickingLoc","Move to the picking place",self.SIMPLE_SPEECH_TIMEOUT,"I am going to pick up you bag.",
                        description="I am going to pick up you bag.",
                        media_src="/img/hri/rob1.jpg", 
                        media_type="img", 
                        )
        result = self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","L1",self.NAVIGATION_TIMEOUT)
        rospy.sleep(2)
        
            # Deux Machina

                # Arm Warning
        # result = self._lm_wrapper.generic_global("ArmWarning0","Warn of arm movement Deux Machina ",self.SIMPLE_SPEECH_TIMEOUT,"Please step back while I deploy my arm.",
        #                 description="Please step back while I deploy my arm.",
        #                 wait_answer=False,
        #                 need_confirmation=False,
        #                 need_validation=False,
        #                 media_src="/img/hri/rob2.jpg",
        #                 media_type="img"
        #                 )
        # rospy.sleep(5)
        # result = self._lm_wrapper.generic_global("ArmDeuxMachina","Take bag from operator",self.SIMPLE_SPEECH_TIMEOUT,"I need your help : please give me the bag by passing the handle through my gripper ?  Tell me OK when done.",
        #                 description="I need tour help : can you give me the bag by passing the handle through my gripper ? Tell me OK when done.",
        #                 wait_answer=True,
        #                 need_confirmation=True,
        #                 need_validation=True,
        #                 media_src="/img/hri/rob1.jpg", 
        #                 media_type="img", 
        #                 )
        # # TODO : deploy the arm "deployed pose"
        # result = self._lm_wrapper.generic_global("ArmParking0","Parking the arm for following",self.SIMPLE_SPEECH_TIMEOUT,"I need your help : please give me the bag by passing the handle through my gripper ?",
        #                 description="I need tour help : can you give me the bag by passing the handle through my gripper ?",
        #                 media_src="/img/hri/rob1.jpg", 
        #                 media_type="img", 
        #                 )
        # TODO : park the arm "parked pose"

        rospy.sleep(2)


        # # Following instruction
        # result = self._lm_wrapper.generic_global("PrepareFollowing ","Prepare the Following",self.SIMPLE_SPEECH_TIMEOUT,"I am ready to follow you. Please don't move too fast. Say drop the bag when arrived. Say yes to start.",
        #                 description="I am ready to follow you. Please don't move too fast. Say drop the bag when arrived. Say yes to start.",
        #                 wait_answer=True,
        #                 need_confirmation=True,
        #                 need_validation=True,
        #                 media_src="/img/hri/rob1.jpg", 
        #                 media_type="img", 
        #                 )
        
        # rospy.sleep(5)


        # # Following
        # result = self._lm_wrapper.generic_global("Following","Following operator",self.SIMPLE_SPEECH_TIMEOUT,"We can go. I follow you.",
        #                 description="We can go. I follow you.",
        #                 media_src="/img/hri/rob1.jpg", 
        #                 media_type="img", 
        #                 )
        # # TODO : Following


        rospy.sleep(2)


        # Operator recovery
            # Natural operator recovery


            # Unnatural operator recovery


            # Touching operator recovery


        # Following Done
        # result = self._lm_wrapper.generic_global("MovePickingLoc","Move to the picking place",self.SIMPLE_SPEECH_TIMEOUT,"Are we there yet ?",
        #                 description="Are we there yet ?",
        #                 wait_answer=True,
        #                 need_confirmation=True,
        #                 need_validation=True,
        #                 media_src="/img/hri/rob1.jpg", 
        #                 media_type="img", 
        #                 )


        # Arm warning
        # result = self._lm_wrapper.generic_global("ArmWarning1","Warn of arm movement",self.SIMPLE_SPEECH_TIMEOUT,"I will give you your bag back. Please step back while I deploy my arm.",
        #                 description="I will give you your bag back. Please step back while I deploy my arm.",
        #                 wait_answer=False,
        #                 need_confirmation=False,
        #                 need_validation=False,
        #                 media_src="/img/hri/rob2.jpg",
        #                 media_type="img"
        #                 )
        # rospy.sleep(5)


        # Arm extension
        # result = self._lm_wrapper.generic_global("ArmDeployment1","Give back arm movement",self.SIMPLE_SPEECH_TIMEOUT,"I give you back your bag.",
        #                 description="I give you back your bag.",
        #                 wait_answer=False,
        #                 need_confirmation=False,
        #                 need_validation=False,
        #                 media_src="/img/hri/rob2.jpg",
        #                 media_type="img"
        #                 )
        # TODO : deploy the arm "deployed pose"

        # rospy.sleep(2)


        # # Giving back the bag
        # result = self._lm_wrapper.generic_global("ReturnBag","Return the bag to the operator",self.SIMPLE_SPEECH_TIMEOUT,"Here is your bag. Say 'thank you' when you get your bag back. You can also confirm on my tablet.",
        #                 description="Here is your bag. Say 'thank you' when you get your bag back. You can also confirm on my tablet.",
        #                 media_src="/img/hri/rob1.jpg", 
        #                 media_type="img", 
        #                 )
        
        # rospy.sleep(2)


        # Goodbye
        # result = self._lm_wrapper.generic_global("Goodbye","Say Goodbye",self.SIMPLE_SPEECH_TIMEOUT,"Have a nice day ! ",
        #                 description="Goodbye !",
        #                 media_src="/img/hri/rob1.jpg", 
        #                 media_type="img", 
        #                 )
        # TODO : park the arm "parked pose"



        # # (Bonus) Return navigation
        # result = self._lm_wrapper.generic_global("id","name",self.SIMPLE_SPEECH_TIMEOUT,"Text said",
        #                 description="description",
        #                 media_src="/img/hri/rob1.jpg", 
        #                 media_type="img", 
        #                 )


        # # (Bonus) Queue navigation
        # result = self._lm_wrapper.generic_global("id","Move to the picking place",self.SIMPLE_SPEECH_TIMEOUT,"Text said",
        #                 description="description",
        #                 media_src="/img/hri/rob1.jpg", 
        #                 media_type="img", 
        #                 )


        #Present the free Space
        # result = self._lm_wrapper.generic_global("FindOtherAndPlace","Ask new member to sit",self.SIMPLE_SPEECH_TIMEOUT, guest_info_list["guest1"]["name"]+",You can have a seat on the sofa here" ,
        #                 description=guest_info_list["guest1"]["name"]+", You can have a seat on the sofa here.",
        #                 media_src="/img/hri/person/user.png", 
        #                 media_type="img", 
        #                 )

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

    def _computePeopleNameOptionFromJson(self, json_people_list):
        options=[]
        for item in json_people_list:
            options.append({'value': item['name'],'media_src': '/img/hri/person/user.png', 'type':'person','media_type': 'img'})
        return options
    
    
    def _callPointAt(self):
            from boxes_3D.srv import boxes3D, boxes3DRequest
            rospy.loginfo("{class_name}: Connecting to the yolov8 boxes_3d_services service...".format(class_name=self.__class__.__name__))
            self._boxToCarryYoloSP = rospy.ServiceProxy('box_to_carry_service', boxes3D)
            try:
                self._boxes3dYoloSP_is_up = rospy.wait_for_service('box_to_carry_service', timeout = self.SERVICE_WAIT_TIMEOUT)
                rospy.loginfo("{class_name}: Connected to the box_to_carry_service service.".format(class_name=self.__class__.__name__))
                request = boxes3DRequest()
                self._boxToCarryYoloSP(request)
            except Exception as e:
                rospy.logwarn("{class_name}: Unable to connect to the box_to_carry_service service.".format(class_name=self.__class__.__name__))
