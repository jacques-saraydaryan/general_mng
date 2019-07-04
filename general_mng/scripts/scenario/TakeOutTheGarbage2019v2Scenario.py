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
from AbstractScenarioService import AbstractScenarioService
from AbstractScenario import AbstractScenario
from LocalManagerWrapper import LocalManagerWrapper

from map_manager.srv import getitP_service
from robocup_msgs.msg import gm_bus_msg

from navigation_manager.msg import NavMngGoal, NavMngAction
from tts_hri.msg import TtsHriGoal, TtsHriAction
from dialogue_hri_srvs.srv import MoveTurn, MoveArmHand,PointAt,GoToCarryPose,ReleaseArms

class TakeOutTheGarbage2019v2Scenario(AbstractScenario,AbstractScenarioBus,AbstractScenarioAction,AbstractScenarioService):

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
        self._enableMinFrontValueService = True

        self._enableMoveHeadPoseService = True


        AbstractScenarioBus.__init__(self,config)
        AbstractScenarioAction.__init__(self,config)

   
        # TODO : Remove Hardocoded values and get them from config
        # self._lm_wrapper = LocalManagerWrapper("10.10.107.177", 9559, "R2019")
        self._lm_wrapper = LocalManagerWrapper("10.10.65.3", 9559, "R2019")

        # with open(config.scenario_filepath) as data:
        ws = "/home/xia0ben/pepper_ws"
        # ws = "/home/astro/catkin_robocup2019"
        # ws = "/home/astro/catkin_robocup2019"
        with open("{0}/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/takeOutGarbage/scenario.json".format(ws)) as data:
            self._scenario = json.load(data)

        with open("{0}/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/locations.json".format(ws)) as data:
            self._location = json.load(data)

        with open("{0}/src/robocup-main/robocup_pepper-scenario_data_generator/jsons/videos.json".format(
                ws)) as data:
            self._videos = json.load(data)

        self._getPoint_service = rospy.ServiceProxy('get_InterestPoint', getitP_service)

        # Debug options
        self.allow_navigation = True
        self.allow_wait_door_open = True

    def startScenario(self):
        rospy.loginfo("""
        ######################################
        Starting the {scenario_name} Scenario...
        ######################################
        """.format(scenario_name=self._scenario["name"]))

        steps = self._scenario["steps"]

        step_id_to_index = self._lm_wrapper.timeboard_send_steps_list(steps, self._scenario["name"], self.NO_TIMEOUT)[1]
        self._lm_wrapper.timeboard_set_timer_state(True, self.NO_TIMEOUT)

        scenario_start_time = time.time()

        ###############################################################################################################
        #################################################### Go to start point ########################################
        ###############################################################################################################

        # Wait for door to open
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["waitDoorOpen"], self.NO_TIMEOUT)

        # - Detect door opening
        waitdooropen_detect = self.find_step(steps, "waitdooropen_detect")
        if self.allow_wait_door_open: self.waitForDoorToOpen(float(waitdooropen_detect["arguments"]["check_freq"]))

        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["waitDoorOpen"], self.NO_TIMEOUT)

        ###############################################################################################################
        #################################################### 1first bin################################################
        ###############################################################################################################

        ################################
        ### 1. Go to the first Bin   ###
        ################################

        # Go to the first Bin
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["GotoB1"], self.NO_TIMEOUT)
        gotolr1_ask_to_b1 = self.find_step(steps, "gotob1_go-to-the-first-bin")
        ##FIXME Error on python side on the pepper --> [WARN ] [Arg Fetcher]: Key not found: said
        bean1_location = self.find_location(self._location, gotolr1_ask_to_b1["arguments"]["location"])
        self._lm_wrapper.go_to(gotolr1_ask_to_b1["speech"], bean1_location, self.NO_TIMEOUT)
        goto_initial_pose = self.find_step(steps, "goto_initial_pose")
        if self.allow_navigation: orderState0a=self.sendNavOrderAction("NP","CRRCloseToGoal", "DOOR_TO_ENTRANCE_02",120.0)
        if self.allow_navigation: orderState0b=self.sendNavOrderAction("NP","CRRCloseToGoal", "ENTRANCE_TO_LIVINGROOM_02",120.0)
        if self.allow_navigation: orderState0c=self.sendNavOrderAction("NP","CRRCloseToGoal", "LIVINGROOM_BIN_01",120.0)
        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["GotoB1"], self.NO_TIMEOUT)


        ################################
        ### 1. Take the first Bin    ###
        ################################

        ## Start take Garbage
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["TakeB1"], self.NO_TIMEOUT)
        call1r1_take_bin1 = self.find_step(steps, "takeb1_call-human")
        cf1_result=False
        nb_retry=0
        while not cf1_result and nb_retry<3:
            #FIXME need to check also the waittime
            self._lm_wrapper.call_human(call1r1_take_bin1["speech"],3.0,self.NO_TIMEOUT)
            call2r1_take_bin1 = self.find_step(steps, "takeb1_explain-how-to-prepare-the-bag")

            #self.sendTtsOrderAction("TTS",call2r1_take_bin1["speech"]["said"] ,"NO_WAIT_END","English",60.0)
            self._lm_wrapper.generic( self.NO_TIMEOUT, call2r1_take_bin1["speech"])
            video = self.find_video(self._videos, call2r1_take_bin1["arguments"]["what"])
            self._lm_wrapper.show_video({"title":'How to close the garbage',"description":'this video shows how to close the garbage '},video,self.NO_TIMEOUT)

            ##Be ready to take garbage
            self.poseToTakeGarbage()

            call3r1_take_bin1 = self.find_step(steps, "takeb1_explain-how-to-put-the-bag-into-pepper's-hand")
            #self.sendTtsOrderAction("TTS",call3r1_take_bin1["speech"]["said"] ,"NO_WAIT_END","English",60.0)
            self._lm_wrapper.generic( self.NO_TIMEOUT,call3r1_take_bin1["speech"])
            video2 = self.find_video(self._videos, call3r1_take_bin1["arguments"]["what"])
            self._lm_wrapper.show_video({"title":'How to give the garbage to the pepper',"description":'this video shows how to give the garbage to the pepper. The knot must go inside the hand. '},video2,self.NO_TIMEOUT)

             ##Go to carry Pose
            self.poseToCarryGarbage()
            #self.sendTtsOrderAction("TTS","Every thing is ok, can i go ?" ,"NO_WAIT_END","English",60.0)
            cf_status,cf1_result=self._lm_wrapper.confirm( {"title":'Confirm if the bag did not fall',"said":'Please confirm that the bag did not fall.',"description": "-" }, self.NO_TIMEOUT)
            nb_retry=nb_retry+1
        
        ################################
        ### 1. Go to collection zone ###
        ################################

        self._lm_wrapper.generic( self.NO_TIMEOUT,{"said": "let's go!, Referee could you please open the door to the collection zone ?",
                                                   "description": "-",
                                                   "title":"let's go!, Referee could you please open the door to the collection zone ?\n please don't stay in front of me"})
        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["TakeB1"], self.NO_TIMEOUT)

        ##Start go to release point
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["CarryB1"], self.NO_TIMEOUT)
        carryb1_go_to_the_collection_zone= self.find_step(steps, "carryb1_go-to-the-collection-zone")
        if self.allow_navigation: orderState1a =self.sendNavOrderAction("NP","CRRCloseToGoal","LIVINGROOM_TO_KITCHEN_01",120.0)
        if self.allow_navigation: orderState1b =self.sendNavOrderAction("NP","CRRCloseToGoal","LIVINGROOM_TO_KITCHEN_02",120.0)
        if self.allow_navigation: orderState1c =self.sendNavOrderAction("NP","CRRCloseToGoal","KITCHEN_WAYPOINT_TO_DOOR_01",120.0)
        if self.allow_navigation: orderState1d =self.sendNavOrderAction("NP","CRRCloseToGoal","KITCHEN_TO_DOOR_01",120.0)
        if self.allow_navigation: orderState1e = self.sendNavOrderAction("NP", "CRRCloseToGoal", "KITCHEN_TO_DOOR_02", 120.0)
        if self.allow_navigation: orderState1f = self.sendNavOrderAction("NP", "CRRCloseToGoal", "OUTSIDE_GARBAGE_COLLECTION_ZONE_01", 120.0)
        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["CarryB1"], self.NO_TIMEOUT)

        ################################
        ###  1. Release bin          ###
        ################################

        ##Arrived on release zone
        ##Release Garbage
        self._lm_wrapper.timeboard_set_current_step(step_id_to_index["DropB1"], self.NO_TIMEOUT)
        self.poseToReleaseGarbage()
        self._lm_wrapper.timeboard_send_step_done(step_id_to_index["DropB1"], self.NO_TIMEOUT)
        ##Release Arm at the end
        self.releaseArms()
        


        ###############################################################################################################
        #################################################### 2nd bin###################################################
        ###############################################################################################################
        #
        # ################################
        # ### 2. Go to the second Bin   ###
        # ################################
        #
        #  # Go to the second B
        # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["GotoB2"], self.NO_TIMEOUT)
        # goto2r1_ask_to_b1 = self.find_step(steps, "gotob2_go-to-the-second-bin")
        # ##FIXME Error on python side on the pepper --> [WARN ] [Arg Fetcher]: Key not found: said
        # bean2_location = self.find_location(self._location, goto2r1_ask_to_b1["arguments"]["location"])
        # self._lm_wrapper.go_to(goto2r1_ask_to_b1["speech"], bean2_location, self.NO_TIMEOUT)
        #
        # if self.allow_navigation: orderState2a = self.sendNavOrderAction("NP", "CRRCloseToGoal", "DOOR_TO_KITCHEN_01", 120.0)
        # if self.allow_navigation: orderState2b = self.sendNavOrderAction("NP", "CRRCloseToGoal", "DOOR_TO_KITCHEN_02", 120.0)
        # if self.allow_navigation: orderState2c = self.sendNavOrderAction("NP", "CRRCloseToGoal", "KITCHEN_WAYPOINT_TO_BIN_01", 120.0)
        # if self.allow_navigation: orderState2d = self.sendNavOrderAction("NP", "CRRCloseToGoal", "KITCHEN_BIN_01", 120.0)
        # if self.allow_navigation: self._lm_wrapper.timeboard_send_step_done(step_id_to_index["GotoB2"], self.NO_TIMEOUT)
        #
        # ################################
        # ### 2. Take the second Bin    ###
        # ################################
        #
        # ## Start take Garbage
        # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["TakeB2"], self.NO_TIMEOUT)
        #
        #
        # call1r1_take_bin2 = self.find_step(steps, "takeb2_call-human")
        # cf2_result=False
        # nb_retry=0
        # while not cf2_result and nb_retry<3:
        #     #FIXME need to check also the waittime
        #     self._lm_wrapper.call_human(call1r1_take_bin2["speech"],3.0,self.NO_TIMEOUT)
        #     call2r1_take_bin2 = self.find_step(steps, "takeb2_explain-how-to-prepare-the-bag")
        #
        #     #self.sendTtsOrderAction("TTS",call2r1_take_bin2["speech"]["said"] ,"NO_WAIT_END","English",60.0)
        #     self._lm_wrapper.generic( self.NO_TIMEOUT,call2r1_take_bin2["speech"])
        #     video = self.find_video(self._videos, call2r1_take_bin2["arguments"]["what"])
        #     self._lm_wrapper.show_video({"title":'How to close the garbage',"description":'this video shows how to close the garbage '},video,self.NO_TIMEOUT)
        #
        #     ##Be ready to take garbage
        #     self.poseToTakeGarbage()
        #
        #     call3r1_take_bin2 = self.find_step(steps, "takeb2_explain-how-to-put-the-bag-into-pepper's-hand")
        #     #self.sendTtsOrderAction("TTS",call3r1_take_bin2["speech"]["said"] ,"NO_WAIT_END","English",60.0)
        #     self._lm_wrapper.generic( self.NO_TIMEOUT,call3r1_take_bin2["speech"])
        #
        #     #self._lm_wrapper.call_human(call3r1_take_bin1["speech"],3.0,self.NO_TIMEOUT)
        #     video2 = self.find_video(self._videos, call3r1_take_bin2["arguments"]["what"])
        #     self._lm_wrapper.show_video({"title":'How to give the garbage to the pepper',"description":'this video shows how to give the garbage to the pepper '},video2,self.NO_TIMEOUT)
        #
        #     ##Go to carry Pose
        #     self.poseToCarryGarbage()
        #     #self.sendTtsOrderAction("TTS","Everything is ok, can i go ?" ,"NO_WAIT_END","English",60.0)
        #     cf_status,cf2_result=self._lm_wrapper.confirm( {"title":'Confirm if the bag did not fall',"said":'Please confirm that the bag did not fall.', "description":"-"}, self.NO_TIMEOUT)
        #     nb_retry=nb_retry+1
        #
        # ################################
        # ### 2. Go to collection zone ###
        # ################################
        #
        # self._lm_wrapper.generic( self.NO_TIMEOUT,{"said": "let's go!, Referee could you please open the door to the collection zone ? ",
        #                                            "description": "-",
        #                                            "title":"let's go!, Referee could you please open the door to the collection zone ? \n please don't stay in front of me"})
        # self._lm_wrapper.timeboard_send_step_done(step_id_to_index["TakeB2"], self.NO_TIMEOUT)
        #
        # ##Start go to release point
        # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["CarryB2"], self.NO_TIMEOUT)
        # carryb2_go_to_the_collection_zone= self.find_step(steps, "carryb2_go-to-the-collection-zone")
        #
        # if self.allow_navigation: orderState3a = self.sendNavOrderAction("NP", "CRRCloseToGoal", "KITCHEN_WAYPOINT_TO_DOOR_02", 120.0)
        # if self.allow_navigation: orderState1b =self.sendNavOrderAction("NP","CRRCloseToGoal","KITCHEN_TO_DOOR_01",120.0)
        # if self.allow_navigation: orderState1c = self.sendNavOrderAction("NP", "CRRCloseToGoal", "KITCHEN_TO_DOOR_02", 120.0)
        # if self.allow_navigation: orderState1d = self.sendNavOrderAction("NP", "CRRCloseToGoal", "OUTSIDE_GARBAGE_COLLECTION_ZONE_01", 120.0)
        #
        # self._lm_wrapper.timeboard_send_step_done(step_id_to_index["CarryB2"], self.NO_TIMEOUT)
        #
        # ################################
        # ###  2. Release bin          ###
        # ################################
        #
        # ##Arrived on release zone
        # ##Release Garbage
        # self._lm_wrapper.timeboard_set_current_step(step_id_to_index["DropB2"], self.NO_TIMEOUT)
        # self.poseToReleaseGarbage()
        # self._lm_wrapper.timeboard_send_step_done(step_id_to_index["DropB2"], self.NO_TIMEOUT)
        # ##Release Arm at the end
        # self.releaseArms()

        #self._lm_wrapper.timeboard_send_step_done(step_id_to_index["finishscenario_finish-scenario"], self.NO_TIMEOUT)


    def gmBusListener(self,msg): 
        if self._status == self.WAIT_ACTION_STATUS:
           self.checkActionStatus(msg)

    def initScenario(self):
        AbstractScenarioAction.configure_intern(self)
        AbstractScenarioService.configure_intern(self)
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

    def poseToReleaseGarbage(self):
        result = self._gotoTakePose(arm_l_or_r='l',turn_rad=0.5,stiffness=0.7)

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

    def find_location(self, locations_array, location_id):
        for location in locations_array:
            if location["id"] == location_id:
                return location

    def find_video(self, videos_array, video_id):
        for video in videos_array:
            if video["id"] == video_id:
                return video