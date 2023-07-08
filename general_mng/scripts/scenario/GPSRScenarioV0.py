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
import actionlib


class GPSRScenarioV0(AbstractScenario):
    TIMEOUT_DEFAULT = 5.0
    TIMEOUT_NAVIGATION = 6 #60
    TIMEOUT_NAVIGATION_ROTATION = 3 #30
    TIMEOUT_SIMPLE_SPEECH = 1  #10
    TIMEOUT_SIMPLE_ACTION = 3  #30
    STT_KEY_WORDS = ["person", "object", "location", "action", "drink", "ack" ]
    STT_KEY_ACTION_NAVIGATE = "navigate"
    STT_KEY_ACTION_NAVIGATE_WORDS = ["navigate","go","move"]
    STT_KEY_ACTION_FIND ="find"
    STT_KEY_ACTION_ASK ="ask"
    STT_KEY_ACTION_MOVE_WAVE ="wave"
    STT_KEY_ACTION_MOVE_POINTAT ="point at"
    STT_KEY_ACTION_MOVE_RAISE ="raise"
    STT_KEY_ACTION_WORDS = []
    for txt in STT_KEY_ACTION_NAVIGATE_WORDS:
       STT_KEY_ACTION_WORDS.append(txt)

    STT_KEY_ACTION_WORDS.append(STT_KEY_ACTION_FIND)
    STT_KEY_ACTION_WORDS.append(STT_KEY_ACTION_ASK)
    STT_KEY_ACTION_WORDS.append(STT_KEY_ACTION_MOVE_WAVE)
    STT_KEY_ACTION_WORDS.append(STT_KEY_ACTION_MOVE_POINTAT)
    STT_KEY_ACTION_WORDS.append(STT_KEY_ACTION_MOVE_RAISE)

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
                        {'action': '', 'arguments': {}, 'eta': self.TIMEOUT_SIMPLE_ACTION, 'id': 'WaitToStarts', 'name': 'Wait to start', 'order': 1},
                        {'action': '', 'arguments': {}, 'eta': self.TIMEOUT_NAVIGATION, 'id': 'MoveLoc0', 'name': 'Move to point', 'order': 2},
                        {'action': '', 'arguments': {}, 'eta': self.TIMEOUT_SIMPLE_ACTION, 'id': 'CheckOperator', 'name': 'check for operator', 'order': 3},
                        {'action': '', 'arguments': {}, 'eta': self.TIMEOUT_SIMPLE_ACTION, 'id': 'ListenOrder', 'name': 'Listen order', 'order': 4},
                        {'action': '', 'arguments': {}, 'eta': self.TIMEOUT_NAVIGATION, 'id': 'ExecuteOrder', 'name': 'Execute order', 'order': 5},
                        {'action': '', 'arguments': {}, 'eta': self.TIMEOUT_NAVIGATION, 'id': 'GoFinalPose', 'name': 'Go to the final pose', 'order': 6},

        ]

        #Update HRI Board with tasks list
        self._lm_wrapper.timeboard_send_steps_list(self.steps, self._scenario["name"], self.NO_TIMEOUT)

        
        #Way Door Opened
        result = self._lm_wrapper.generic_global("WaitToStarts","Wait an opened door",self.TIMEOUT_SIMPLE_ACTION,"I am waiting an opened door to start",
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
        result = self._lm_wrapper.generic_global("MoveLoc0","Move to the point",self.TIMEOUT_NAVIGATION,"I am going to the instruction point",
                        description="I am going to the instruction point",
                        media_src="/img/hri/rob1.jpg", 
                        media_type="img", 
                        )
        #result = self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","L_To_E",60)
        result = self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","L6",self.TIMEOUT_NAVIGATION)

        
        
        #Check Operator presence  
        result = self._lm_wrapper.generic_global("CheckOperator","Check People presence",self.TIMEOUT_SIMPLE_ACTION,"I'm looking for the operator",
                        description="I'm looking for the operator !",
                        media_src="/img/hri/robot-lookingat2.jpeg",
                        media_type="img",
                        )
        #result = self._checkPeopleIsHere(timeout=2)
        #if(result.status == result.FAILURE_STATUS):
        #    self.print_result(result)

        rospy.sleep(10)

        #Listen instruction 
        result = self._lm_wrapper.generic_global("ListenOrder","Listen order",self.TIMEOUT_NAVIGATION,"Dear operator, I am ready for your instruction",
                        description="Dear operator, I am ready for your instruction",
                        media_src="/img/hri/learning-robot.jpg",
                        media_type="img",
                        )
        #-----------------------------------
        # 
        #      Listen operator
        #
        # -----------------------------------
        self._listenAndExecuteInstruction(timeout=20)




        

        #Got to final place
        result = self._lm_wrapper.generic_global("GoFinalPose","Go to the final pose",self.TIMEOUT_NAVIGATION,"I am going to the final pose",
                        description="I am going to the final pose",
                        media_src="/img/hri/rob1.jpg", 
                        media_type="img", 
                        )
        result = self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","L_To_E",self.TIMEOUT_NAVIGATION)
        
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
    

    def _listenAndExecuteInstruction(self,timeout=10):
        choice_list={}
        choice_list['location'] = ['kitchen','livingroom']
        choice_list['action'] = self.STT_KEY_ACTION_WORDS

        LOCATION_TO_IT={'kitchen':"L_To_E",'livingroom': 'L4'}
     
        #TODO et correctly id
        result  = self._listen(choice_list,12,timeout=timeout)
        if result == None:
           rospy.logwarn("[GENERAL MANAGER] Unable to get answer from STT")
           return None
           
        result = result.answer
        current_action = self._getMostProbableData(result.action)
        if current_action in self.STT_KEY_ACTION_NAVIGATE_WORDS:
           current_location = self._getMostProbableData(result.location )
           if current_location in LOCATION_TO_IT.keys():
            #Execute action
            self._navigate_to_point(LOCATION_TO_IT[current_location])
        elif current_action == self.STT_KEY_ACTION_ASK:
            self._answerToOperator("This is a fake answer")


    def _navigate_to_point(self, it_name):
        result = self._lm_wrapper.generic_global("ExecuteOrder","Execute order",self.TIMEOUT_NAVIGATION,"I am going to the location",
                        description="I am going to the location",
                        media_src="/img/hri/rob1.jpg", 
                        media_type="img", 
                        )
        self.print_result(result)
        result = self._lt_navigation.send_nav_order("NP","CRRCloseToGoal",it_name,self.TIMEOUT_NAVIGATION)
        self.print_result(result)

    def _answerToOperator(self, response):
        #Listen instruction 
        result = self._lm_wrapper.generic_global("ExecuteOrder","Execute order",self.TIMEOUT_NAVIGATION,response,
                        description=response,
                        media_src="/img/hri/learning-robot.jpg",
                        media_type="img",
                        )
        self.print_result(result)


    def _listen(self, choice_list,step_id, timeout = 10):
        from stt_nlu_actions.msg import NLExpectationsAction, NLExpectationsGoal, NLExpectationsResult 

        action_stt_client = actionlib.SimpleActionClient("nl_expectations",NLExpectationsAction)
        stt_finished = action_stt_client.wait_for_server(timeout = rospy.Duration(1.0))
        if not stt_finished:
           rospy.logwarn("{class_name} : Connection to STT server Timeout!!! Stop STT and TTS resolution for the current task".format(class_name=self.__class__.__name__))
        
        nul_goal=NLExpectationsGoal()
    
        nul_goal.waitfor.goal_group = str(step_id);
        nul_goal.expected_timeout.data = timeout
        for item in choice_list:
            try:
              if item in self.STT_KEY_WORDS:
                #fixme too dependent of goal message... need to find a way to update dynamically value
                #nul_goal.waitfor.__dict__[item.keys()[0]]=choice_list[0][item.keys()[0]]
                if item =='person':
                  nul_goal.waitfor.person=choice_list[item]
                elif item =='object':
                  nul_goal.waitfor.object=choice_list[item]
                elif item =='drink':
                  nul_goal.waitfor.drink=choice_list[item]
                elif item =='location':
                  nul_goal.waitfor.location=choice_list[item]
                elif item =='action':
                  nul_goal.waitfor.action=choice_list[item]

            except KeyError as e:
              rospy.logwarn("{class_name} : STT malformed choice list "+str(choice_list).format(class_name=self.__class__.__name__))
         
        action_stt_client.send_goal(nul_goal)
        finished = action_stt_client.wait_for_result(timeout = rospy.Duration(timeout))
        if(finished is False):
           rospy.logwarn("[General Manager] timeout calling STT")

        return action_stt_client.get_result() 
        #nul_goal.waitfor.ack.data =True

    def _getMostProbableData(self, list):
       value_max_prob = ""
       max_prob = ""
       for elt in list:
          if max_prob > elt.confidence:
            value_max_prob = elt.confidenceelt.data
       return value_max_prob

          
       
     
    
        
        
    
