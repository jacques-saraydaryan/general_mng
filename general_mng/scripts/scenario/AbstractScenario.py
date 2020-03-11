__author__ = 'Jacques Saraydaryan'
from abc import ABCMeta, abstractmethod
import rospy
from robocup_msgs.msg import gm_bus_msg
import uuid
from threading import Timer
from meta_lib.LTServiceResponse import LTServiceResponse

class AbstractScenario:

    @abstractmethod
    def startScenario(self): pass

    @abstractmethod
    def initScenario(self): pass


    def print_result(self, data):
        if data.status == LTServiceResponse.FAILURE_STATUS:
            rospy.logwarn(data)
        else:
            rospy.loginfo(data)
