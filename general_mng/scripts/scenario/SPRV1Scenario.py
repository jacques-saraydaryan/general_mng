__author__ = 'Jacques Saraydaryan'
from abc import ABCMeta, abstractmethod
import rospy
import uuid
import time
import random
import actionlib

from AbstractScenarioBus import AbstractScenarioBus
from AbstractScenarioAction import AbstractScenarioAction
from AbstractScenario import AbstractScenario


from map_manager.srv import getitP_service
from robocup_msgs.msg import gm_bus_msg

from navigation_manager.msg import NavMngGoal, NavMngAction
from tts_hri.msg import TtsHriGoal, TtsHriAction
from pepper_pose_for_nav.srv import MoveHeadAtPosition





class SPRV1Scenario(AbstractScenario,AbstractScenarioBus,AbstractScenarioAction):

    _severalActionPending={}
    _oneActionPending=None
    _peopleMetaMap=''
    HEAD_PITCH_FOR_PEOPLE_DETECTION=-0.3
    HEAD_YAW_FOR_PEOPLE_DETECTION=0.0
    DEFAULT_QUESTION_LOCATION=[]
    QUESTION_GENDER_MALE='gender_male'
    QUESTION_GENDER_FEMALE='gender_female'
    QUESTION_POSE_STANDING='pose_standing'
    QUESTION_POSE_SITTING='pose_sitting'
    QUESTION_POSE_WAVING='pose_waving'
    QUESTION_POSE_LYING='pose_lying'
    QUESTION_PEOPLE='people'
    QUESTION_COLOR_RED='color_red'
    QUESTION_COLOR_BLUE='color_blue'
    QUESTION_COLOR_WHITE='color_white'
    QUESTION_COLOR_BLACK='color_black'
    QUESTION_COLOR_GREEN='color_green'
    QUESTION_COLOR_YELLOW='color_yellow'   
    
    #DEFAULT_OBJ_LABEL=[]
    #DEFAULT_OBJECT_MEMORY_LOCATION="Robocup/objects"

    def __init__(self,config):
        AbstractScenarioBus.__init__(self,config)
        AbstractScenarioAction.__init__(self,config)
        self._getPoint_service = rospy.ServiceProxy('get_InterestPoint', getitP_service)

        try:
            self.question_memory_location=config['question_memory_location']
        except Exception as e:
            rospy.logwarn("no config value for question_memory_location use default:"+str(self.DEFAULT_QUESTION_LOCATION))
            self.question_memory_location=self.DEFAULT_QUESTION_LOCATION
#
        #try:
        #    self.object_memory_location=config['object_memory_location']
        #except Exception as e:
        #    rospy.logwarn("no config value for object_memory_location use default:"+str(self.DEFAULT_OBJECT_MEMORY_LOCATION))
        #    self.object_memory_location=self.DEFAULT_OBJECT_MEMORY_LOCATION

            
        try:
            rospy.wait_for_service('/move_head_pose_srv',5)
            rospy.loginfo("end service move_head_pose_srv wait time")
            self._moveHeadPose = rospy.ServiceProxy('move_head_pose_srv', MoveHeadAtPosition)
        except Exception as e:
            rospy.logerr("Service move_head_pose_srv call failed: %s" % e)
            #return
        self._peopleMetaMap={}
        try:
            self.question_memory_location[self.QUESTION_PEOPLE]
            self.question_memory_location[self.QUESTION_GENDER_MALE]
            self.question_memory_location[self.QUESTION_GENDER_FEMALE]
            self.question_memory_location[self.QUESTION_POSE_STANDING]
            self.question_memory_location[self.QUESTION_POSE_SITTING]
            self.question_memory_location[self.QUESTION_POSE_LYING]
            self.question_memory_location[self.QUESTION_POSE_WAVING]
            self.question_memory_location[self.QUESTION_COLOR_BLACK]
            self.question_memory_location[self.QUESTION_COLOR_BLUE]
            self.question_memory_location[self.QUESTION_COLOR_GREEN]
            self.question_memory_location[self.QUESTION_COLOR_RED]
            self.question_memory_location[self.QUESTION_COLOR_WHITE]
            self.question_memory_location[self.QUESTION_COLOR_YELLOW]

            self.resetPeopleMetaMap()
        except Exception as e:
            rospy.logerr("BAD CONFIG FILE MISSING MEMORY LOCATION KEY:"+str(e))



    def startScenario(self):
        self.resetPeopleMetaMap()
        try:
            rospy.loginfo("")
            rospy.loginfo("######################################")
            rospy.loginfo("Starting the SPRV1 Scenario...")
            rospy.loginfo("######################################")
        
            #TOO make the logic of the scenario

            #turn of 180 degre before detecting people
            #TODO 

            # set the head position for detecting people
            #self.moveheadPose(self.HEAD_PITCH_FOR_PEOPLE_DETECTION,self.HEAD_YAW_FOR_PEOPLE_DETECTION,True)
            #rospy.sleep(2.0)
        
            orderState0,result0=self.detectMetaPeople(30)
            rospy.loginfo(result0)

            if orderState0 == 3:
                #process people attribute to save elts to ALMEMORY
                self.processResult(result0)

            #Tell pepper process is finished
            orderState5,result5=self.sendDialogueOrderAction("SPR/ProcessPeopleFinished","",5.0)
        except Exception as e:
            rospy.logwarn(str(e))




    def gmBusListener(self,msg): 
        if self._status == self.WAIT_ACTION_STATUS:
           self.checkActionStatus(msg)


    def initScenario(self):
        
        self._enableNavAction=False
        self._enableTtsAction=False
        self._enableDialogueAction=True
        self._enableAddInMemoryAction=True
        self._enableObjectMngAction=False
        self._enableMultiplePeopleDetectionAction=True
        
        AbstractScenarioAction.configure_intern(self)

    def moveheadPose(self,pitch_value,yaw_value,track):
        try:
            self._moveHeadPose = rospy.ServiceProxy('move_head_pose_srv', MoveHeadAtPosition)
            result=self._moveHeadPose(pitch_value,yaw_value,track)
        except Exception as e:
            rospy.logerr("Service move_head_pose_srv call failed: %s" % e)
            return
    
    def processResult(self,result):
        if len(result.peopleMetaList.peopleList) == 0:
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_PEOPLE],str(0),5.0)
        else:
            self._peopleMetaMap[self.QUESTION_PEOPLE]=len(result.peopleMetaList.peopleList)
            for person in result.peopleMetaList.peopleList:
               
                self._peopleMetaMap[self.QUESTION_GENDER_MALE]=0
                self._peopleMetaMap[self.QUESTION_GENDER_FEMALE]=0
                if person.posture=='Standing':
                    self._peopleMetaMap[self.QUESTION_POSE_STANDING]+=1
                    
                elif person.posture=='Sitting':
                    self._peopleMetaMap[self.QUESTION_POSE_SITTING]+=1
                    
                elif person.posture=='Lying' or person.posture=='Undefined':
                    self._peopleMetaMap[self.QUESTION_POSE_LYING]+=1
                    
                    
                if person.handCall == True:
                    self._peopleMetaMap[self.QUESTION_POSE_WAVING]+=1
                
                if person.shirt_color_name=='BLACK':
                    self._peopleMetaMap[self.QUESTION_COLOR_BLACK]+=1
                elif 'BLEU' in person.shirt_color_name or 'CYAN' in person.shirt_color_name:
                    self._peopleMetaMap[self.QUESTION_COLOR_BLUE]+=1
                elif 'GREEN' in person.shirt_color_name:
                    self._peopleMetaMap[self.QUESTION_COLOR_GREEN]+=1
                elif 'RED' in person.shirt_color_name:
                    self._peopleMetaMap[self.QUESTION_COLOR_RED]+=1
                elif 'WHITE' in person.shirt_color_name:
                    self._peopleMetaMap[self.QUESTION_COLOR_WHITE]+=1
                elif 'YELLOW' in person.shirt_color_name:
                    self._peopleMetaMap[self.QUESTION_COLOR_YELLOW]+=1

            self.addInPepperMemory(self.question_memory_location[self.QUESTION_PEOPLE],str(self._peopleMetaMap[self.QUESTION_PEOPLE]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_POSE_STANDING],str(self._peopleMetaMap[self.QUESTION_POSE_STANDING]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_POSE_SITTING],str(self._peopleMetaMap[self.QUESTION_POSE_SITTING]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_POSE_LYING],str(self._peopleMetaMap[self.QUESTION_POSE_LYING]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_POSE_WAVING],str(self._peopleMetaMap[self.QUESTION_POSE_WAVING]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_COLOR_BLACK],str(self._peopleMetaMap[self.QUESTION_COLOR_BLACK]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_COLOR_BLUE],str(self._peopleMetaMap[self.QUESTION_COLOR_BLUE]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_COLOR_GREEN],str(self._peopleMetaMap[self.QUESTION_COLOR_GREEN]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_COLOR_RED],str(self._peopleMetaMap[self.QUESTION_COLOR_RED]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_COLOR_WHITE],str(self._peopleMetaMap[self.QUESTION_COLOR_WHITE]),5.0)
            self.addInPepperMemory(self.question_memory_location[self.QUESTION_COLOR_YELLOW],str(self._peopleMetaMap[self.QUESTION_COLOR_YELLOW]),5.0)

    def resetPeopleMetaMap(self):
        try:
           self._peopleMetaMap[self.QUESTION_PEOPLE]=0
           self._peopleMetaMap[self.QUESTION_GENDER_MALE]=0
           self._peopleMetaMap[self.QUESTION_GENDER_FEMALE]=0
           self._peopleMetaMap[self.QUESTION_POSE_STANDING]=0
           self._peopleMetaMap[self.QUESTION_POSE_SITTING]=0
           self._peopleMetaMap[self.QUESTION_POSE_LYING]=0
           self._peopleMetaMap[self.QUESTION_POSE_WAVING]=0
           self._peopleMetaMap[self.QUESTION_COLOR_BLACK]=0
           self._peopleMetaMap[self.QUESTION_COLOR_BLUE]=0
           self._peopleMetaMap[self.QUESTION_COLOR_GREEN]=0
           self._peopleMetaMap[self.QUESTION_COLOR_RED]=0
           self._peopleMetaMap[self.QUESTION_COLOR_WHITE]=0
           self._peopleMetaMap[self.QUESTION_COLOR_YELLOW]=0
        except Exception as e:
           rospy.logwarn("Error resetting _peopleMetaMap"+str(e))