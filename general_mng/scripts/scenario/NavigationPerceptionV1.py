__author__ = 'Jacques Saraydaryan'
from abc import ABCMeta, abstractmethod
import rospy
from AbstractScenario import AbstractScenario

from meta_lib.LTPerception import LTPerception
from meta_lib.LTNavigation import LTNavigation


class NavigationPerceptionV1(AbstractScenario):
    _severalActionPending = {}
    _oneActionPending = None

    def __init__(self, config):
        self.configurationReady=False


    def startScenario(self):
        rospy.loginfo("")
        rospy.loginfo("######################################")
        rospy.loginfo("Starting the NavigationPerceptionV1 Scenario...")
        rospy.loginfo("######################################")

        # Wait door opening
        result = self.lt_perception.wait_for_door_to_open()
        self.print_result(result)


        # start navigation A
        result = self.lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", 1.8, 7.4, 60.0)
        self.print_result(result)

        name="BIG_HERO"

        # learn a people Face associates to a name
        result = self.lt_perception.learn_people_meta_from_img_topic(name, 10.0)
        self.print_result(result)

        # start navigation B
        result = self.lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", 8.8, 1.25, 60.0)
        self.print_result(result)

        # detect people face
        result = self.lt_perception.get_people_name_from_img_topic(10.0)
        self.print_result(result)
        rospy.loginfo(result.payload.peopleNames)
        rospy.loginfo(result.payload.peopleNamesScore)

        # start navigation C
        result = self.lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", 2.0, 1.0, 60.0)
        self.print_result(result)


    def initScenario(self):
        self.lt_perception = LTPerception()
        self.lt_navigation = LTNavigation()

        if self.lt_perception.configurationReady == True and self.lt_navigation.configurationReady == True:
            self.configurationReady = True

