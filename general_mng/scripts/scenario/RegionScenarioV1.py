__author__ = 'Jacques Saraydaryan'
import rospy
import os
import json
from copy import deepcopy
from AbstractScenario import AbstractScenario

from meta_lib.LTHriManager import LTHriManagerPalbator
from meta_lib.LTNavigation import LTNavigation
from meta_lib.LTPerception import LTPerception
import math

class RegionScenarioV1(AbstractScenario):
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
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'WaitStart', 'name': 'Wait start order', 'order': 1},
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'MoveLoc1', 'name': 'Move to location 1', 'order': 2},
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'MoveLoc2', 'name': 'Move to location 2', 'order': 3},
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'MoveLoc3', 'name': 'Move to location 3', 'order': 4},
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'End', 'name': 'This is the End', 'order': 5},
        ]

        while not rospy.is_shutdown():
            #Update HRI Board with tasks list
            self._lm_wrapper.timeboard_send_steps_list(self.steps, self._scenario["name"], self.NO_TIMEOUT)

            #Way Start Order
            result = self._lm_wrapper.generic_global("WaitStart","Wait start order'",10,"I am waiting the order to start",
                            description="I am waiting the order to start",
                            wait_answer=True,
                            need_confirmation=False,
                            need_validation=True,
                            media_src="/img/hri/rob2.jpg",
                            media_type="img"
                            )
            self.print_result(result)   
        
            for i in range (0,3):

                result = self._lm_wrapper.generic_global("MoveLoc1","Move to Loc1",60,"I'm going to the location",
                                description="I'm going to the location",
                                media_src="/img/hri/rob1.jpg",
                                media_type="img",
                                )
                # start navigation
                result = self._lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal",0.444, 0.0, 60.0)
                rospy.sleep(3)


                result = self._lm_wrapper.generic_global("MoveLoc2","Move to Loc2",60,"I'm going to the location",
                            description="I'm going to the location",
                            media_src="/img/hri/rob1.jpg",
                            media_type="img",
                            )
                # start navigation
                result = self._lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", -1.97, -2.7, 60.0)
                rospy.sleep(3)

                result = self._lm_wrapper.generic_global("MoveLoc3","Move to Loc3",60,"I'm going to the location",
                            description="I'm going to the location",
                            media_src="/img/hri/rob1.jpg",
                            media_type="img",
                            )
                # start navigation
                # result = self._lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", -5.51, -2.37, 60.0)
                result = self._lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", -2.01, -0.387, 60.0)
                rotation_angle = math.pi
                result = self._lt_navigation.send_nav_rotation_order("NT", rotation_angle , 90.0)

                
                rospy.sleep(3)


            result = self._lm_wrapper.generic_global("End","The End Name",30,"The End Speech",
                        description="The End Description",
                        wait_answer=False,
                        need_confirmation=False,
                        need_validation=False,
                        media_src="/img/hri/theend.gif",
                        media_type="img"
                        )
            rospy.sleep(10)
            self.print_result(result)

    def onShutDown(self):
        rospy.loginfo(""" SHUTDOWN ASKED Clear action server before ending """)
        self._lm_wrapper.cancel_all_actions_HRI_mng
    