__author__ = 'Jacques Saraydaryan'
from abc import ABCMeta, abstractmethod
import rospy
from robocup_msgs.msg import gm_bus_msg
import uuid
from threading import Timer
import actionlib
from navigation_manager.msg import NavMngGoal, NavMngAction
from tts_hri.msg import TtsHriGoal, TtsHriAction
from dialogue_hri_actions.msg import DialogueSendSignalAction,DialogueSendSignalGoal,AddInMemoryAction,AddInMemoryGoal
from object_management.msg import ObjectDetectionAction,ObjectDetectionGoal
from ros_people_mng_actions.msg import ProcessPeopleFromImgAction,ProcessPeopleFromImgGoal

class AbstractScenarioAction:
    _actionNavMng_server=None
    _actionTtsHri_server=None
    _enableNavAction=True
    _enableTtsAction=True
    _enableDialogueAction=True
    _enableAddInMemoryAction=True
    _enableObjectMngAction=False
    _enableMultiplePeopleDetectionAction=False
    _configurationReady=False
    
   

    def __init__(self,config):
       pass

    def configure_intern(self):
        try:
            if self._enableNavAction:
                rospy.loginfo("Connecting to navigation_manager action server ... ")
                self._actionNavMng_server = actionlib.SimpleActionClient('navigation_manager', NavMngAction)
                finished1 =self._actionNavMng_server.wait_for_server(timeout = rospy.Duration(20.0))
                if finished1:
                    rospy.loginfo("navigation_manager Connected")
                else:
                    rospy.logwarn("Unable to connect to navigation_manager action server")
                rospy.loginfo("Connecting to tts_hri action server ... ")

            if self._enableTtsAction:
                self._actionTtsHri_server = actionlib.SimpleActionClient('tts_hri', TtsHriAction)
                finished2 = self._actionTtsHri_server.wait_for_server(timeout = rospy.Duration(10.0))
                if finished2:
                    rospy.loginfo("tts_hri Connected")
                else:
                    rospy.logwarn("Unable to connect to tts_hri action server")

            if self._enableDialogueAction:
                self._actionDialogueHri_server = actionlib.SimpleActionClient('dialogue_hri_signal', DialogueSendSignalAction)
                finished3 = self._actionDialogueHri_server.wait_for_server(timeout = rospy.Duration(10.0))
                if finished3:
                    rospy.loginfo("dialogue_hri Connected")
                else:
                    rospy.logwarn("Unable to connect to dialogue_hri action server")
            
            if self._enableAddInMemoryAction:
                self._actionAddInMemoryHri_server = actionlib.SimpleActionClient('add_in_memory_action', AddInMemoryAction)
                finished4 = self._actionAddInMemoryHri_server.wait_for_server(timeout = rospy.Duration(10.0))
                if finished4:
                    rospy.loginfo("AddInMemoryHri Connected")
                else:
                    rospy.logwarn("Unable to connect to AddInMemoryHri action server")


            if self._enableObjectMngAction:
                self._actioneObjectMng_server = actionlib.SimpleActionClient('object_detection_action', ObjectDetectionAction)
                finished5 = self._actioneObjectMng_server.wait_for_server(timeout = rospy.Duration(10.0))
                if finished5:
                    rospy.loginfo("ObjectMng Connected")
                else:
                    rospy.logwarn("Unable to connect to ObjectMng action server")

            
            if self._enableMultiplePeopleDetectionAction:
                self._actioneMultiplePeopleDetection_server = actionlib.SimpleActionClient('detect_people_meta_action', ProcessPeopleFromImgAction)
                finished6 = self._actioneMultiplePeopleDetection_server.wait_for_server(timeout = rospy.Duration(10.0))
                if finished6:
                    rospy.loginfo("MultiplePeopleDetection Connected")
                else:
                    rospy.logwarn("Unable to connect to MultiplePeopleDetection action server")

                

        except Exception as e:
            rospy.loginfo("Unable to connect to action server: %s" % e)
        self._configurationReady=True


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


    def sendDialogueOrderAction(self,signal_to_emit,signal_to_wait,timeout):
        goalDialogue = DialogueSendSignalGoal()
        # signal send to naoqi
        goalDialogue.signal_to_emit=signal_to_emit
        # signal on wich action server will wait answer
        goalDialogue.signal_to_wait=signal_to_wait
        rospy.loginfo("### DIALOGUE ACTION PENDING : %s",str(goalDialogue).replace('\n',', '))
        # send the current goal to the action server
        self._actionDialogueHri_server.send_goal(goalDialogue)
        # wait action server result
        finished_before_timeout=self._actionDialogueHri_server.wait_for_result(rospy.Duration.from_sec(timeout))
        state=self._actionDialogueHri_server.get_state()
        result=self._actionDialogueHri_server.get_result()
        rospy.loginfo("###### DIALOGUE ACTION END , State: %s",str(state))
        # if timeout cancel all goals on the action server
        if finished_before_timeout:
            self._actionDialogueHri_server.cancel_all_goals()
        # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
        return state,result


    def addInPepperMemory(self,memory_location,json_payload,timeout):
        goalAddInMemory = AddInMemoryGoal()
        goalAddInMemory.memory_location=memory_location
        goalAddInMemory.payload=json_payload
        rospy.loginfo("### ADD IN MEMORY ACTION PENDING : %s",str(goalAddInMemory).replace('\n',', '))

        # send the current goal to the action server
        self._actionAddInMemoryHri_server.send_goal(goalAddInMemory)
        # wait action server result
        finished_before_timeout=self._actionAddInMemoryHri_server.wait_for_result(rospy.Duration.from_sec(timeout))
        state=self._actionAddInMemoryHri_server.get_state()
        result=self._actionAddInMemoryHri_server.get_result()
        rospy.loginfo("###### ADD IN MEMORY ACTION END , State: %s",str(state))
        # if timeout cancel all goals on the action server
        if finished_before_timeout:
            self._actionAddInMemoryHri_server.cancel_all_goals()
        # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
        return state,result
    


    def getObjectInFrontRobot(self,labels,timeout):
        goalObjMng = ObjectDetectionGoal()
        goalObjMng.labels=labels
       
        rospy.loginfo("### OBJECT MNG GET OBJECT ACTION PENDING : %s",str(goalObjMng).replace('\n',', '))

        # send the current goal to the action server
        self._actioneObjectMng_server.send_goal(goalObjMng)
        # wait action server result
        finished_before_timeout=self._actioneObjectMng_server.wait_for_result(rospy.Duration.from_sec(timeout))
        state=self._actioneObjectMng_server.get_state()
        result=self._actioneObjectMng_server.get_result()
        rospy.loginfo("###### OBJECT MNG GET OBJECT ACTION END , State: %s",str(state))
        # if timeout cancel all goals on the action server
        if finished_before_timeout:
            self._actioneObjectMng_server.cancel_all_goals()
        # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
        return state,result


    def detectMetaPeople(self,timeout):
        goalMetaPeople = ProcessPeopleFromImgGoal()
        rospy.loginfo("### DETECT META PEOPLE ACTION PENDING")

        # send the current goal to the action server
        self._actioneMultiplePeopleDetection_server.send_goal(goalMetaPeople)
        # wait action server result
        finished_before_timeout=self._actioneMultiplePeopleDetection_server.wait_for_result(rospy.Duration.from_sec(timeout))
        state=self._actioneMultiplePeopleDetection_server.get_state()
        result=self._actioneMultiplePeopleDetection_server.get_result()
        rospy.loginfo("###### DETECT META PEOPLE ACTION END , State: %s",str(state))
        # if timeout cancel all goals on the action server
        if finished_before_timeout:
            self._actioneMultiplePeopleDetection_server.cancel_all_goals()
        # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
        return state,result
