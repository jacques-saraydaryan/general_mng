__author__ = 'Simon ERNST & Thomas CURE'
import rospy
import collections

from AbstractScenario import AbstractScenario
from meta_lib.LTHriManager import LTHriManagerPalbator
import sys
import json
import time
import math
from copy import deepcopy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus
import os

class CleanUp2020CPEScenario(AbstractScenario):
    DEFAULT_TIMEOUT = 5.0
    NO_TIMEOUT = -1.0

    def __init__(self,config,scenario_path_folder):

        self.initScenario()
        self._scenario_path_folder = scenario_path_folder

        self._scenario=config
        self._scenario['name']='cleanup'

        _name_action_server_HRI = self._scenario['parameters']['LTHri_action_server_name']
        self._lm_wrapper = LTHriManagerPalbator(_name_action_server_HRI)

        self._locations = self._scenario['imports']['locations']
        self._objects = self._scenario['imports']['objects']
        

        self._path_scenario_infos = self._scenario['variables']['scenarioInfos']
        self.reset_infos_JSON()
        
        rospy.loginfo("{class_name}: JSON FILES LOADED.".format(class_name=self.__class__))

        self.Room_to_clean = None

        # Debug options
        self.allow_navigation = False

        self.configuration_ready = True

    def start_scenario(self):   
        """
        Runs the scenario according to the scenario JSON file
        """
        rospy.loginfo("""
        ######################################
        Starting the {scenario_name} Scenario...
        ######################################
        """.format(scenario_name=self._scenario["name"]))
        self.restart_order=False
        self.steps = deepcopy(self._scenario["steps"])

        rospy.loginfo("SCN : WAITING FOR ACTION SERVER ACTIVATION")
        self._lm_wrapper.client_action_GmToHri.wait_for_server()

        rospy.loginfo("SCN : LOADING CONFIG FOR SCENARIO")
        self._lm_wrapper.timeboard_send_steps_list(self.steps, self._scenario["name"], self.NO_TIMEOUT)


        self.current_index_scenario=0
        while self.current_index_scenario<len(self.steps) and not rospy.is_shutdown():
            
            rospy.loginfo("SCN : CURRENT STEP INDEX : "+str(self.current_index_scenario))
            rospy.loginfo("NEW STEP")

            self.current_step=deepcopy(self.steps[self.current_index_scenario])

            if self.current_step['action']!="":
                result=self.action_parser(self.current_step['action'])
                
                rospy.loginfo("SCN : RESULT FROM PARSER "+str(result))
            else:
                result=self._lm_wrapper.timeboard_set_current_step(self.current_index_scenario,self.NO_TIMEOUT)[1]
                rospy.loginfo("SCN : RESULT WITHOUT PARSER "+str(result))
            
            if result is None:
                rospy.logwarn("SCN : ACTION ABORTED")
                break
            
            elif result != None:
                if 'result' in result and result['result']=="PREEMPTED": 
                    rospy.logwarn("SCN : ACTION ABORTED")
                    break
                
                if result['NextIndex']!="":
                    self.current_index_scenario=deepcopy(result['NextIndex'])

                if 'saveData' in result:
                    self.store_infos(result['saveData'])


    def reset_infos_JSON(self):
        with open(os.path.join(self._scenario_path_folder,self._path_scenario_infos),"w+") as f:
            data_JSON = {}
            json.dumps(data_JSON, f, indent=4)
            f.truncate()

    def store_infos(self,data):
        try:
            with open(os.path.join(self._scenario_path_folder,self._path_scenario_infos),"r") as f:
                data_JSON=json.load(f)
        except:
            rospy.logwarn("INFOS FILE EMPTY")
            data_JSON = {}

        data_JSON[data['what']]={}
        data_JSON[data['what']]['name']=data['where']
        for location in self._locations:
            if location['name'] == data['where']:
                data_JSON[data['what']]['pathOnTablet']=location['pathOnTablet']
                break

        with open(os.path.join(self._scenario_path_folder,self._path_scenario_infos),"w+") as f:
            
            json.dumps(data_JSON, f, indent=4)
            f.truncate()
        
        self._scenario_infos = data_JSON  

    
    def initScenario(self):
        self._enableNavAction = True
        self._enableTtsAction = False
        self._enableDialogueAction = False
        self._enableAddInMemoryAction = False
        self._enableObjectDetectionMngAction = True
        self._enableLookAtObjectMngAction = True
        self._enableMultiplePeopleDetectionAction = False
        self._enableRequestToLocalManagerAction = True
        self._enableLearnPeopleMetaAction = True
        self._enableGetPeopleNameAction = True

        self._enableMoveHeadPoseService = True
        self._enableMoveTurnService = True
        self._enablePointAtService = True
        self._enableResetPersonMetaInfoMapService = True
        self._enableReleaseArmsService = True

    def action_parser(self,stepAction):
        """
        Load a function according to the action name of current step. 

        :param stepAction: action name of current step
        :type stepAction: string
        """
        rospy.loginfo('-------------------------')
        rospy.loginfo('using parser')
        rospy.loginfo('***********')
        switcher = {
        "foundObject": self.gm_find,
        "catchObject": self.gm_catch_object,
        "goTo": self.gm_go_to,
        "storeObject": self.gm_store_object,
        "releaseObject": self.gm_release_object,
        "openDoor": self.gm_open_door,
        # "askRoomToClean" : self.gm_ask_room,
        }
        # Get the function from switcher dictionary
        func = switcher.get(stepAction, lambda: "Invalid action")
        
        return func(self.current_index_scenario)


    # def gm_ask_room(self,stepIndex):
    #     """
    #     Function dealing with the askRoomToClean action. The robot asks to the referee the room to clean.

    #     :param stepIndex: Step index
    #     :type stepIndex: int
    #     """
    #     rospy.loginfo("SCN ACTION FOUND OBJECT")
    #     time.sleep(2)
    #     return self._lm_wrapper.timeboard_set_current_step(stepIndex,self.NO_TIMEOUT)[1]
    
    
    def gm_find(self,stepIndex):
        """
        Function dealing with the find action. The robot finds something or someone.

        :param stepIndex: Step index
        :type stepIndex: int
        """
        rospy.loginfo("SCN ACTION FOUND OBJECT")
        time.sleep(2)
        return self._lm_wrapper.timeboard_set_current_step(stepIndex,self.NO_TIMEOUT)[1]


    def gm_catch_object(self,stepIndex):
        """
        Function dealing with the catchObject action. The robot will catch something.

        :param stepIndex: Step index
        :type stepIndex: int
        """
        rospy.loginfo("SCN ACTION CATCH OBJECT")
        time.sleep(2)
        return self._lm_wrapper.timeboard_set_current_step(stepIndex,self.NO_TIMEOUT)[1]
    
    def gm_go_to(self,stepIndex):
        """
        Function dealing with the goTo action. The robot goes to the choosen interest point depending on the scenario.

        :param stepIndex: Step index
        :type stepIndex: int
        """
        rospy.loginfo("SCN ACTION GO TO OBJECT")
        self._lm_wrapper.timeboard_set_current_step_with_data(stepIndex,deepcopy(self._scenario_infos),self.NO_TIMEOUT)
        time.sleep(2)
        result={
            "NextIndex": stepIndex+1
        }
        return result

    def gm_store_object(self,stepIndex):
        """
        Function dealing with the storeObject action. The robot will store the object he caught before.

        :param stepIndex: Step index
        :type stepIndex: int
        """
        rospy.loginfo("SCN ACTION STORE OBJECT")
        time.sleep(2)
        return self._lm_wrapper.timeboard_set_current_step(stepIndex,self.NO_TIMEOUT)[1] 
    
    def gm_release_object(self,stepIndex):
        """
        Function dealing with the releaseObject action. The robot will release the object he caught before.

        :param stepIndex: Step index
        :type stepIndex: int
        """
        rospy.loginfo("SCN ACTION RELEASE OBJECT")
        time.sleep(2)
        return self._lm_wrapper.timeboard_set_current_step(stepIndex,self.NO_TIMEOUT)[1]

    def gm_open_door(self,stepIndex):
        """
        Function dealing with the openDoor action. The robot will open the door and go to the choosen room.
        :param stepIndex: Step index
        :type stepIndex: int
        """
        rospy.loginfo("SCN ACTION OPEN DOOR OBJECT")
        time.sleep(2)
        return self._lm_wrapper.timeboard_set_current_step(stepIndex,self.NO_TIMEOUT)[1]