__author__ = 'Jacques Saraydaryan'
from abc import ABCMeta, abstractmethod
import rospy
from AbstractScenario import AbstractScenario

from meta_lib.LTPerception import LTPerception
from meta_lib.LTNavigation import LTNavigation


class DoorOpenAndNavigScenarioV1(AbstractScenario):
    _severalActionPending = {}
    _oneActionPending = None

    def __init__(self, config):
        self.configurationReady=False


    def startScenario(self):
        rospy.loginfo("")
        rospy.loginfo("######################################")
        rospy.loginfo("Starting the DoorOpenAndNavigScenario Scenario...")
        rospy.loginfo("######################################")

        # Wait door opening
        result = self.lt_perception.wait_for_door_to_open()
        self.print_result(result)


        # start navigation
        result = self.lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", 1.8, 7.4, 60.0)
        self.print_result(result)

        result = self.lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", 1.5, 9.25, 60.0)
        self.print_result(result)

        result = self.lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", 2.0, 1.0, 60.0)
        self.print_result(result)


    def initScenario(self):
        self.lt_perception = LTPerception()
        self.lt_navigation = LTNavigation()

        if self.lt_perception.configurationReady == True and self.lt_navigation.configurationReady == True:
            self.configurationReady = True

