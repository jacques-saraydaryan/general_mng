__author__ = 'Jacques Saraydaryan'
import rospy
from AbstractScenario import AbstractScenario

from meta_lib.LTNavigation import LTNavigation

class SimpleNavScenario(AbstractScenario):
    _severalActionPending = {}
    _oneActionPending = None

    def __init__(self,config,scenario_path_folder):
        self.configuration_ready = False
        self.init_scenario(config)

    def init_scenario(self, config):
        self.lt_navigation = LTNavigation()
        
        self.configuration_ready = True

    def start_scenario(self):
        """
        Runs the scenario according to the scenario JSON file
        """
        rospy.loginfo("""
        ######################################
        Starting the TEST NAV Scenario...
        ######################################
        """)

        # Wait door opening
        #result = self.lt_perception.wait_for_door_to_open()
        #self.print_result(result)


        # start navigation A
        result = self.lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", 2.59, -2.18, 60.0)
        self.print_result(result)

        # start navigation A
        result = self.lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", -0.689, 0.386, 60.0)
        self.print_result(result)

        # start navigation A
        result = self.lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", 3.25, 0.0384, 60.0)
        self.print_result(result)

        # start navigation A
        result = self.lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", 2.67, 2.13, 60.0)
        self.print_result(result)
