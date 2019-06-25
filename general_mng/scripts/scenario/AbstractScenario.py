__author__ = 'Jacques Saraydaryan'
from abc import ABCMeta, abstractmethod
import rospy
import math
from robocup_msgs.msg import gm_bus_msg
import uuid
from threading import Timer

class AbstractScenario:

    @abstractmethod
    def startScenario(self): pass

    @abstractmethod
    def initScenario(self): pass
