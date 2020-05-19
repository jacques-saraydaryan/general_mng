__author__ = 'Jacques Saraydaryan'

from abc import ABCMeta, abstractmethod
from LTAbstract import LTAbstract
from LTServiceResponse import LTServiceResponse
from actionlib_msgs.msg import GoalStatus

import actionlib
import rospy

from navigation_manager.msg import NavMngGoal, NavMngAction


def singleton(cls):    
    instance = [None]
    def wrapper(*args, **kwargs):
        if instance[0] is None:
            instance[0] = cls(*args, **kwargs)
        return instance[0]

    return wrapper

@singleton
class LTNavigation(LTAbstract):

    _enableNavAction = True

    def __init__(self):
        self.configure_intern()

        #Inform configuration is ready
        self.configurationReady = True

    #######################################
    # CONFIGURATION
    ######################################

    def configure_intern(self):
        rospy.loginfo("Connecting to navigation_manager action server ... ")

        self._actionNavMng_server = actionlib.SimpleActionClient('navigation_manager', NavMngAction)
        finished1 = self._actionNavMng_server.wait_for_server(timeout=rospy.Duration(self.ACTION_WAIT_TIMEOUT))

        if finished1:
            rospy.loginfo("navigation_manager Connected")
        else:
            rospy.logwarn("Unable to connect to navigation_manager action server")

    def reset(self):
        self.configure_intern()

    #######################################
    # NAVIGATION API
    ######################################
    def send_nav_order(self, action, mode, it_point, timeout, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__send_nav_order_action,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for send_nav_order" % (service_mode)
            return response
        else:
            feedback = fct(action, mode, it_point, timeout)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during send_nav_order to it:[%s], mode:[%s], action[%s]" % (
                it_point, mode, action)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes send_nav_order to it:[%s], mode:[%s], action[%s]" % (
                    it_point, mode, action)
                response.payload = feedback
                return response
        return response

    def send_nav_rotation_order(self,action, rotation_angle, timeout, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__send_nav_rotation_order,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for send_nav_order" % (service_mode)
            return response
        else:
            feedback = fct(action, rotation_angle, timeout)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during send_nav_rotation_order to rotation_angle:[%s], action[%s]" % (
                rotation_angle, action)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes send_nav_rotation_order to rotation_angle:[%s], action[%s]" % (
                    rotation_angle, action)
                response.payload = feedback
                return response
        return response

    def send_nav_order_to_pt(self, action, mode, x, y, timeout, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__send_nav_order_to_pt_action,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for send_nav_order" % (service_mode)
            return response
        else:
            feedback = fct(action, mode, x, y, timeout)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during send_nav_order to it:[%s,%s], mode:[%s], action[%s]" % (
                    x, y, mode, action)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success send_nav_order to it:[%s,%s], mode:[%s], action[%s]" % (
                    x, y, mode, action)
                response.payload = feedback
                return response
        return response

    #######################################
    # NAVIGATION ACTION
    ######################################

    def __send_nav_order_action(self, action, mode, itP, timeout):
        try:

            goal = NavMngGoal()
            goal.action = action
            goal.itP = itP
            goal.navstrategy = mode
            rospy.loginfo("### NAV ACTION PENDING : %s", str(goal).replace('\n', ', '))
            self._actionNavMng_server.send_goal(goal)
            self._actionNavMng_server.wait_for_result(rospy.Duration.from_sec(timeout))
            state = self._actionNavMng_server.get_state()
            if state == GoalStatus.ABORTED:
                rospy.logwarn("###### NAV ACTION END , State: %s", self.action_status_to_string(state))
            else:
                rospy.loginfo("###### NAV ACTION END , State: %s", self.action_status_to_string(state))
            return state
        except Exception as e:
            rospy.logwarn("###### NAV ACTION FAILURE , State: %s", str(e))
        return GoalStatus.ABORTED

    def __send_nav_rotation_order(self, action, rotation_angle, timeout):
        try:

            goal = NavMngGoal()
            goal.action = action
            goal.rotation_angle = rotation_angle
            rospy.loginfo("### NAV ROTATION ACTION PENDING : %s", str(goal).replace('\n', ', '))
            self._actionNavMng_server.send_goal(goal)
            self._actionNavMng_server.wait_for_result(rospy.Duration.from_sec(timeout))
            state = self._actionNavMng_server.get_state()
            if state == GoalStatus.ABORTED:
                rospy.logwarn("###### NAV ROTATION ACTION END , State: %s", self.action_status_to_string(state))
            else:
                rospy.loginfo("###### NAV ROTATION ACTION END , State: %s", self.action_status_to_string(state))
            return state
        except Exception as e:
            rospy.logwarn("###### NAV ROTATION ACTION FAILURE , State: %s", str(e))
        return GoalStatus.ABORTED


    def __send_nav_order_to_pt_action(self, action, mode, x, y, timeout):
        try:
            goal = NavMngGoal()
            goal.action = action
            goal.itP = ''
            goal.itP_point.x = x
            goal.itP_point.y = y
            goal.navstrategy = mode
            rospy.loginfo("### NAV ACTION PENDING : %s", str(goal).replace('\n', ', '))
            self._actionNavMng_server.send_goal(goal)
            self._actionNavMng_server.wait_for_result(rospy.Duration.from_sec(timeout))
            state = self._actionNavMng_server.get_state()
            if state == GoalStatus.ABORTED:
                rospy.logwarn("###### NAV ACTION END , State: %s", self.action_status_to_string(state))
            else:
                rospy.loginfo("###### NAV ACTION END , State: %s", self.action_status_to_string(state))
            return state
        except Exception as e:
            rospy.logwarn("###### NAV ACTION FAILURE , State: %s", str(e))
        return GoalStatus.ABORTED
