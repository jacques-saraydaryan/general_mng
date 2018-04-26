__author__ = 'Jacques Saraydaryan'
from abc import ABCMeta, abstractmethod
import rospy
from robocup_msgs.msg import gm_bus_msg
import uuid
from threading import Timer
import actionlib
from navigation_manager.msg import NavMngGoal, NavMngAction
from tts_hri.msg import TtsHriGoal, TtsHriAction

class AbstractScenarioAction:
    _actionNavMng_server=None
    _actionTtsHri_server=None

    def __init__(self,config):
        try:
            rospy.loginfo("Connecting to navigation_manager action server ... ")
            self._actionNavMng_server = actionlib.SimpleActionClient('navigation_manager', NavMngAction)
            finished1 =self._actionNavMng_server.wait_for_server(timeout = rospy.Duration(20.0))
            if finished1:
                rospy.loginfo("navigation_manager Connected")
            else:
                rospy.logwarn("Unable to connect to navigation_manager action server")
            rospy.loginfo("Connecting to tts_hri action server ... ")
            self._actionTtsHri_server = actionlib.SimpleActionClient('tts_hri', TtsHriAction)
            finished2 = self._actionTtsHri_server.wait_for_server(timeout = rospy.Duration(10.0))
            if finished2:
                rospy.loginfo("tts_hri Connected")
            else:
                rospy.logwarn("Unable to connect to tts_hri action server")
        except Exception as e:
            rospy.loginfo("Unable to connect to action server: %s" % e)



    def sendNavOrderAction(self,action,mode,itP,timeout):
        goal = NavMngGoal()
        goal.action=action
        goal.itP=itP
        goal.navstrategy=mode
        rospy.loginfo("### NAV ACTION PENDING : %s",str(goal).replace('\n',', '))
        self._actionNavMng_server.send_goal(goal)
        self._actionNavMng_server.wait_for_result(rospy.Duration.from_sec(timeout))
        state=self._actionNavMng_server.get_state()
        if state ==4:
            rospy.logwarn("###### NAV ACTION END , State: %s",str(state))
        else:
            rospy.loginfo("###### NAV ACTION END , State: %s",str(state))
        return state
    
    def sendNavOrderActionToPt(self,action,mode,x,y,timeout):
        goal = NavMngGoal()
        goal.action=action
        goal.itP=''
        goal.itP_point.x=x
        goal.itP_point.y=y
        goal.navstrategy=mode
        rospy.loginfo("### NAV ACTION PENDING : %s",str(goal).replace('\n',', '))
        self._actionNavMng_server.send_goal(goal)
        self._actionNavMng_server.wait_for_result(rospy.Duration.from_sec(timeout))
        state=self._actionNavMng_server.get_state()
        if state ==4:
            rospy.logwarn("###### NAV ACTION END , State: %s",str(state))
        else:
            rospy.loginfo("###### NAV ACTION END , State: %s",str(state))
        return state

    def sendTtsOrderAction(self,action,txt,mode,lang,timeout):
        goalTts = TtsHriGoal()
        goalTts.action=action
        goalTts.txt=txt
        goalTts.mode=mode
        goalTts.lang=lang
        rospy.loginfo("### TTS ACTION PENDING : %s",str(goalTts).replace('\n',', '))
        self._actionTtsHri_server.send_goal(goalTts)
        self._actionTtsHri_server.wait_for_result(rospy.Duration.from_sec(timeout))
        state=self._actionTtsHri_server.get_state()
        rospy.loginfo("###### TTS ACTION END , State: %s",str(state))
        return state
