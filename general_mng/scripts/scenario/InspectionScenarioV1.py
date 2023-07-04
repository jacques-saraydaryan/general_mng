__author__ = 'Jacques Saraydaryan'
import rospy
import os
import json
from copy import deepcopy
from AbstractScenario import AbstractScenario

from meta_lib.LTHriManager import LTHriManagerPalbator
from meta_lib.LTNavigation import LTNavigation
from meta_lib.LTPerception import LTPerception

class InspectionScenarioV1(AbstractScenario):
    DEFAULT_TIMEOUT = 5.0
    NO_TIMEOUT = -1.0
    _severalActionPending = {}
    _oneActionPending = None

    def __init__(self,config,scenario_path_folder):
        self.configuration_ready = False
        self.init_scenario(config,scenario_path_folder)
        rospy.on_shutdown(self.onShutDown)
        

    def init_scenario(self, config,scenario_path_folder):

        self._scenario_path_folder = scenario_path_folder
        self._scenario=config
        self.current_dir = os.path.dirname(os.path.abspath(__file__))

        
        _name_action_server_HRI = self._scenario['parameters']['LTHri_action_server_name']
        self._nav_strategy = self._scenario['parameters']['nav_strategy_parameters']
        debug_variables = self._scenario['parameters']['debug_variables']
        
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
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'DoorOpen', 'name': 'Wait an opened door', 'order': 1},
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'MoveLoc1', 'name': 'Move to location', 'order': 2},
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'WaitReferee', 'name': 'Wait Referee to continue', 'order': 3},
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'MoveLoc2', 'name': 'Move to location', 'order': 4},

        ]

        #Update HRI Board with tasks list
        self._lm_wrapper.timeboard_send_steps_list(self.steps, self._scenario["name"], self.NO_TIMEOUT)

        #Way Door Opened
        result = self._lm_wrapper.generic_global("DoorOpen","Wait an opened door",30,"I am waiting an opened door to start",
                        description="I am waiting an opened door to start",
                        wait_answer=False,
                        need_confirmation=False,
                        need_validation=False,
                        media_src="/img/hri/rob2.jpg",
                        media_type="img"
                        )
        result = self._lt_perception.wait_for_door_to_open()
        #rospy.sleep(10)
        self.print_result(result)
        
        
        result = self._lm_wrapper.generic_global("MoveLoc1","Move to Loc1",60,"I'm going to the location",
                        description="I'm going to the location",
                        media_src="/img/hri/rob1.jpg",
                        media_type="img",
                        )
        # start navigation
        #result = self._lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", -11.8, 2.45, 30.0)
        result = self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","It1",30)
        result = self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","It2",30)


        result = self._lm_wrapper.generic_global("WaitReferee","Wait Referee Action",60,"Tell me when you are ready to continue ",
                        description="Tell me when you are ready to continue ",
                        wait_answer=True,
                        need_confirmation=False,
                        need_validation=True
                        )

        result = self._lm_wrapper.generic_global("MoveLoc2","Move to Loc2",60,"I'm going to the location",
                        description="I'm going to the location",
                        media_src="/img/hri/rob1.jpg",
                        media_type="img",
                        )
        # start navigation
        result = self._lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", -11.9, -2.25, 30.0)
        self.print_result(result)

    def onShutDown(self):
        rospy.loginfo(""" SHUTDOWN ASKED Clear action server before ending """)
        self._lm_wrapper.cancel_all_actions_HRI_mng
    