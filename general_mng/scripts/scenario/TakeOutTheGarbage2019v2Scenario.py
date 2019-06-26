__author__ = 'Jacques Saraydaryan'
from abc import ABCMeta, abstractmethod
import rospy
import uuid
import time
import random
import actionlib


import json

import time

from AbstractScenarioBus import AbstractScenarioBus
from AbstractScenarioAction import AbstractScenarioAction
from AbstractScenario import AbstractScenario
from LocalManagerWrapper import LocalManagerWrapper

from map_manager.srv import getitP_service
from robocup_msgs.msg import gm_bus_msg

from navigation_manager.msg import NavMngGoal, NavMngAction
from tts_hri.msg import TtsHriGoal, TtsHriAction
from dialogue_hri_srvs.srv import MoveTurn, MoveArmHand,PointAt,GoToCarryPose,ReleaseArms

class TakeOutTheGarbage2019v2Scenario(AbstractScenario,AbstractScenarioBus,AbstractScenarioAction):

    _severalActionPending={}
    _oneActionPending=None

    DEFAULT_TIMEOUT = 5.0
    NO_TIMEOUT = -1.0

    def __init__(self,config):

        self._enableNavAction = True
        self._enableTtsAction = True
        self._enableDialogueAction = True
        self._enableAddInMemoryAction = False
        self._enableObjectDetectionMngAction = False
        self._enableLookAtObjectMngAction = False
        self._enableMultiplePeopleDetectionAction = False

        self._enableMoveHeadPoseService = True


        AbstractScenarioBus.__init__(self,config)
        AbstractScenarioAction.__init__(self,config)

   
        # TODO : Remove Hardocoded values and get them from config
        self._lm_wrapper = LocalManagerWrapper("192.168.42.221", 9559, "R2019")

        # with open(config.scenario_filepath) as data:
        with open("/home/jsaraydaryan/ros_robotcupathome_ws/src/data/general_management/jsons/takeOutGarbage/scenario.json") as data:
            self._scenario = json.load(data)
        with open("/home/jsaraydaryan/ros_robotcupathome_ws/src/data/general_management/jsons/locations.json") as data:
            self._location=json.load(data)

        self._getPoint_service = rospy.ServiceProxy('get_InterestPoint', getitP_service)


    def startScenario(self):
        rospy.loginfo("""
        ######################################
        Starting the {scenario_name} Scenario...
        ######################################
        """.format(scenario_name=self._scenario["name"]))

        steps = self._scenario["steps"]

        step_id_to_index = self._lm_wrapper.timeboard_send_steps_list(steps, self._scenario["name"], self.NO_TIMEOUT)[1]
        


        orderState0=self.sendNavOrderAction("NP","CRRCloseToGoal","It0",120.0)

        # Go to the first B
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["GotoB1"], self.NO_TIMEOUT)
        gotolr1_ask_to_b1 = self.find_step(steps, "gotob1_go-to-the-first-bin")
        ##FIXME Error on python side on the pepper --> [WARN ] [Arg Fetcher]: Key not found: said
        #self._lm_wrapper.go_to(gotolr1_ask_to_b1["speech"],"bin1", self.NO_TIMEOUT)

        orderState0=self.sendNavOrderAction("NP","CRRCloseToGoal","It1",120.0)
        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["GotoB1"], self.NO_TIMEOUT)


        ## Start take Garbage
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["TakeB1"], self.NO_TIMEOUT)


        call1r1_take_bin1 = self.find_step(steps, "takeb1_call-human")
        ##FIXME nothing on the screen, need to check also the waittime
        self._lm_wrapper.call_human(call1r1_take_bin1["speech"],30,self.NO_TIMEOUT)
        call2r1_take_bin1 = self.find_step(steps, "takeb1_explain-how-to-prepare-the-bag")
        ##FIXME nothing on the screen, check the relative url?
        self._lm_wrapper.show_video(call2r1_take_bin1["speech"],'',self.NO_TIMEOUT)
        

        ##Be ready to take garbage
        self.poseToTakeGarbage()
        orderState2,result2=self.sendDialogueOrderAction("Garbage/PlaceGarbageStart","Garbage/PlaceGarbageFinished",10.0*1)
        
        ##Go to carry Pose
        self.poseToCarryGarbage()
        self.sendTtsOrderAction("TTS"," Every thing is ready ? " ,"NO_WAIT_END","English",60.0)
        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["TakeB1"], self.NO_TIMEOUT)

        ##Start go to release point
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["CarryB1"], self.NO_TIMEOUT)
        orderState0=self.sendNavOrderAction("NP","CRRCloseToGoal","It0",120.0)
        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["CarryB1"], self.NO_TIMEOUT)


        ##Arrived on release zone
        ##Release Garbage
        self.poseToTakeGarbage()
        ##Release Arm at the end
        self.releaseArms()
        


    def gmBusListener(self,msg): 
        if self._status == self.WAIT_ACTION_STATUS:
           self.checkActionStatus(msg)

    def initScenario(self):
        AbstractScenarioAction.configure_intern(self)
        self.initPoseToCarryGarbage()
        self.initPoseToTakeGarbage()
        self.initReleaseArms()


    def initPoseToCarryGarbage(self):
         ## Wait for service
        try:
            rospy.wait_for_service('/go_to_carry_pose', 5)
            rospy.loginfo("     service /go_to_carry_pose wait time ready")
            self._gotoCarryPose = rospy.ServiceProxy('/go_to_carry_pose', GoToCarryPose)
        except Exception as e:
            rospy.logerr("      Service /go_to_carry_pose call failed: %s" % e)
            return


    def initPoseToTakeGarbage(self):
         ## Wait for service
        try:
            rospy.wait_for_service('/move_arm_hand', 5)
            rospy.loginfo("     service /move_arm_hand wait time ready")
            self._gotoTakePose = rospy.ServiceProxy('/move_arm_hand', MoveArmHand)
        except Exception as e:
            rospy.logerr("      Service /move_arm_hand call failed: %s" % e)
            return

    def initReleaseArms(self):
         ## Wait for service
        try:
            rospy.wait_for_service('/release_arms', 5)
            rospy.loginfo("     service /release_arms wait time ready")
            self._releaseArms = rospy.ServiceProxy('/release_arms', ReleaseArms)
        except Exception as e:
            rospy.logerr("      Service /release_arms call failed: %s" % e)
            return
    

    def poseToTakeGarbage(self):
        result = self._gotoTakePose(arm_l_or_r='l',turn_rad=0.0,stiffness=0.7)

    def poseToCarryGarbage(self):
        result = self._gotoCarryPose(arm_l_or_r='l',keep_pose=True,stiffness=0.9)

    def releaseArms(self):
        result = self._releaseArms(stiffness=0.7)


#####################################################################################################################################

    def find_step(self, steps_array, step_id):
        step_index = self.find_step_index(steps_array, step_id)
        if step_index is None:
            return None
        else:
            return steps_array[step_index]

    def find_step_index(self, steps_array, step_id):
        for index in range(len(steps_array)):
            step = steps_array[index]
            if step["id"] == step_id:
                return index
        return None