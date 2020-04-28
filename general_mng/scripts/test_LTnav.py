#!/usr/bin/env python

__author__ = 'Simon ERNST & Thomas CURE'
import rospy

from meta_lib.LTNavigation import LTNavigation
from std_msgs.msg import String
import time

class TestNavigation():
    
    def __init__(self):

        rospy.init_node("test_LTNavigation")
        self._lt_navigation = LTNavigation()

        self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","Thomas_entrance", 90.0)

        # self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","Thomas_entrance_table", 90.0)

        self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","Thomas_double_bedroom", 90.0)

        self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","Thomas_simple_bedroom", 90.0)

        self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","Thomas_Kitchen", 90.0)

        self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","Thomas_livingRoom", 90.0)

        self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","Thomas_entrance", 90.0)



if __name__ == "__main__":
    a=TestNavigation()
    