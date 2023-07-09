__author__ = 'Jacques Saraydaryan'
import rospy
import os
import json
from copy import deepcopy
from AbstractScenario import AbstractScenario
from nav_msgs.msg import Path, Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tools.tf_tools import transform_pose

from meta_lib.LTHriManager import LTHriManagerPalbator
from meta_lib.LTNavigation import LTNavigation
from meta_lib.LTPerception import LTPerception
from meta_lib.LTServiceResponse import LTServiceResponse
import time
import math


class ReceptionistScenarioV1(AbstractScenario):
    DEFAULT_TIMEOUT = 5.0
    NO_TIMEOUT = -1.0
    YOLO_CLASS_CHAIR= 'chair'
    YOLO_CLASS_COUCH= 'couch'
    _severalActionPending = {}
    _oneActionPending = None

    def __init__(self,config,scenario_path_folder):
        self.configuration_ready = False
        self.init_scenario(config,scenario_path_folder)

        # get odometry
        rospy.Subscriber("/odom", Odometry, self.odomCallback)
        rospy.on_shutdown(self.onShutDown)
        

    def init_scenario(self, config,scenario_path_folder):

        self._scenario_path_folder = scenario_path_folder
        self._scenario=config
        self.current_dir = os.path.dirname(os.path.abspath(__file__))

        
        _name_action_server_HRI = self._scenario['parameters']['LTHri_action_server_name']
        self._nav_strategy = self._scenario['parameters']['nav_strategy_parameters']
        debug_variables = self._scenario['parameters']['debug_variables']

        self._drink_list = self._scenario['imports']['drink']
        self._person_list = self._scenario['imports']['person']
        
        self._lm_wrapper = LTHriManagerPalbator(_name_action_server_HRI)
        self._lt_navigation = LTNavigation()
        self._lt_perception = LTPerception()

        
        rospy.loginfo("{class_name}: JSON FILES LOADED.".format(class_name=self.__class__.__name__))

        self.configuration_ready = True
        

    def start_scenario(self):
        """
        Runs the scenario according to the scenario JSON file
        """
        rospy.loginfo("""
        ######################################
        Starting the %s Scenario...
        ######################################
        """%str(self._scenario["name"]))

        # Wait door opening
        #result = self.lt_perception.wait_for_door_to_open()
        #self.print_result(result)

        self.manual_scenario()

    def manual_scenario(self):
        #self.steps = deepcopy(self._scenario["steps"])

        rospy.loginfo("{class_name} : WAITING FOR HRI ACTION SERVER ACTIVATION".format(class_name=self.__class__.__name__))
        self._lm_wrapper.client_action_GmToHri.wait_for_server()

        self._lm_wrapper.restart_hri(self.NO_TIMEOUT)

        self.steps = [
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'WaitToStarts', 'name': 'Wait to start', 'order': 1},
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'MoveLoc1', 'name': 'Move to the door', 'order': 2},
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'CheckPeople1', 'name': 'Check People presence', 'order': 3},
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'AskPeople1', 'name': 'Ask people name and preference', 'order': 4},
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'MoveLoc2', 'name': 'Move to the party', 'order': 5},
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'PresentTheGuest', 'name': 'Present new member', 'order': 6},
                        {'action': '', 'arguments': {}, 'eta': 30, 'id': 'FindOtherAndPlace', 'name': 'Ask new member to sit', 'order': 7},

        ]

        #Update HRI Board with tasks list
        self._lm_wrapper.timeboard_send_steps_list(self.steps, self._scenario["name"], self.NO_TIMEOUT)


        #Way Door Opened
        result = self._lm_wrapper.generic_global("WaitToStarts","Wait an opened door",30,"I am waiting an opened door to start",
                        description="I am waiting an opened door to start",
                        wait_answer=False,
                        need_confirmation=False,
                        need_validation=False,
                        media_src="/img/hri/rob2.jpg",
                        media_type="img"
                        )
        #result = self._lt_perception.wait_for_door_to_open()
        rospy.sleep(10)

        # Fake test to move at the end of this scenario
        #result = self._lt_perception.get_object_Boxes3D_from_yolo(classes=[0])
        #person_to_focus_on = self._selectPersonToPointAt(result)
        #angle = 0.0
        #if person_to_focus_on != None:
        #    angle = self._ask_angle_to_point_at_pose(person_to_focus_on["pose"])
        #    rospy.loginfo("[ReceptionistScenarioV1] people selected ask to turn and point at this person")
        #else:
        #    rospy.logwarn("[ReceptionistScenarioV1] no people with valid pose has been detected")
#
        ## Ask robot to turn from the given angle
        #self._lt_navigation.send_nav_rotation_order("NT", angle ,20)
#
#
        ## Fake 2 test to move at the end of this scenario
        #result = self._lt_perception.get_object_Boxes3D_from_yolo(classes=[56,57])
        #person_to_focus_on = self._selectFreeSitToPointAt(result)
        #angle = 0.0        
        #
        #result = self._lm_wrapper.generic_global("MoveLoc1","Move to Loc1",60,"I'm going to the location",
        #                description="I'm going to the location",
        #                media_src="/img/hri/rob1.jpg",
        #                media_type="img",
        #                )
        ## start navigation
        ##result = self._lt_navigation.send_nav_order_to_pt("NP", "CRRCloseToGoal", -11.8, 2.45, 30.0)
        #result = self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","It1",20)
        
        
        #Check People presence 
        result = self._lm_wrapper.generic_global("CheckPeople1","Check People presence",60,"I'm looking for a new guest",
                        description="I'm looking for a new guest",
                        media_src="/img/hri/robot-lookingat2.jpeg",
                        media_type="img",
                        )
        result = self._checkPeopleIsHere()
        if(result.status == result.FAILURE_STATUS):
            self.print_result(result)

        rospy.sleep(2)
        #Learn People Meta (far learn) 
        result = self._lm_wrapper.generic_global("AskPeople1","Ask people name and preference",30,"Hi ! Please wait some seconds I am going to memorise some information",
                        description="Please wait some seconds I am going to memorise some information",
                        media_src="/img/hri/learning-robot.jpg",
                        media_type="img",
                        )
        guest_info_list={}      
        result = self._lt_perception.learn_people_meta_from_img_topic("guest1",30)
        guest_info_list["guest1"]={}
        guest_info_list["guest1"]["MetaInfo"] = result
        rospy.sleep(5)

        #Ask and learn name
        options_drink= self._computeDrinkOptionFromJson(self._drink_list)
        options_user= self._computePeopleNameOptionFromJson(self._person_list)

        result = self._lm_wrapper.generic_global("AskPeople1","Ask people name and preference",30," Please can you come closer and say your name ?",
                        description="Please can you come clother and say your name ?",
                        type="question",
                        wait_answer=True,
                        need_confirmation=True,
                        need_validation=True,
                        options=options_user
                        )
        try:
            guest_info_list["guest1"]["name"]= result.payload['person'][0]
        except KeyError:
            rospy.WARN("Unable to get name from result")
        self.print_result(result)

        result = self._lm_wrapper.generic_global("AskPeople1","Ask people name and preference",30,"I register your data",
                        description="Data Registration",
                        media_src="/img/hri/learning-robot.jpg",
                        media_type="img",
                        )

        result = self._lm_wrapper.generic_global("AskPeople1","Ask people name and preference",30," Please can you select your prefer drink ?",
                        description="Please can you select your prefer drink ?",
                        type="question",
                        wait_answer=True,
                        need_confirmation=True,
                        need_validation=True,
                        options=options_drink
                        )
        
        try:
            guest_info_list["guest1"]["drink"]= result.payload['drink'][0]
        except KeyError:
            rospy.WARN("Unable to get drink from result")

        # Go to the party area
        result = self._lm_wrapper.generic_global("MoveLoc2","Move to the party",60,"Please follow me, I will present you other guests",
                        description="I'm going to the location to meet other guests",
                        media_src="/img/hri/rob1.jpg", 
                        media_type="img", 
                        )
        result = self._lt_navigation.send_nav_order("NP","CRRCloseToGoal","It2",20)

        #Find Other Guest
        #TODO TODO BOXE3D to find people
        #result = self._lt_perception.get_object_from_yolo(model='yolov8m-pose.pt',classes=[0])
        #self._computeRotationToClosestPerson()

        #Face to the present Guest
        #TODO Turn the base in direction to the present guest

        #Present the Guest
        result = self._lm_wrapper.generic_global("PresentTheGuest","Present new member",60,"Hi john, here a new guest " + guest_info_list["guest1"]["name"]+", his prefer drink is "+guest_info_list["guest1"]["drink"],
                        description="Hi john, here a new guest " + guest_info_list["guest1"]["name"]+", his prefer drink is "+guest_info_list["guest1"]["drink"],
                        media_src="/img/hri/person/user.png", 
                        media_type="img", 
                        )
    
        rospy.sleep(10)
        
        #Find Other Chair, sofa 
        #TODO BOXE3D to find object for free space
        #result = self._lt_perception.get_object_from_yolo(model='yolov8m.pt',classes=[])
        #rot_in_rad= self._computeRotationToPointAtObject(result[LTPerception.RESPONSE_LABEL_DETAILS],result[LTPerception.RESPONSE_LABEL_SUMUP])

        #Face to the free space
        #TODO Turn the base in direction to the free Space

         #Present the free Space
        result = self._lm_wrapper.generic_global("FindOtherAndPlace","Ask new member to sit",30, guest_info_list["guest1"]["name"]+", you can sit a the free place here" ,
                        description=guest_info_list["guest1"]["name"]+", you can sit a the free place here",
                        media_src="/img/hri/person/user.png", 
                        media_type="img", 
                        )

        self.print_result(result)
        
    def onShutDown(self):
        rospy.loginfo(""" SHUTDOWN ASKED Clear action server before ending """)
        self._lm_wrapper.cancel_all_actions_HRI_mng


#******************************************************************************************
#*************************************   Tools   **************************************
#******************************************************************************************


    def _checkPeopleIsHere(self,timeout=20):
        people_detected = False
        start_time = time.time()
        #Until a person is detected
        while(not people_detected and time.time()-start_time < timeout):
            result = self._lt_perception.detect_meta_people_from_img_topic(30);
            if(result.status == result.SUCCESS_STATUS):
                if(len(result.payload.peopleMetaList.peopleList)>0):
                    people_detected = True
                    return result
        response = LTServiceResponse(current_status="FAILURE", current_msg="No people detected, timeout")
        return response

    def _computeDrinkOptionFromJson(self, json_drink_list):
        options=[]
        for item in json_drink_list:
            options.append({'value': item['name'],'media_src': item['pathOnTablet'], 'type':'drink','media_type': 'img'})
        return options
    

    def _computePeopleNameOptionFromJson(self, json_people_list):
        options=[]
        for item in json_people_list:
            options.append({'value': item['name'],'media_src': '/img/hri/person/user.png', 'type':'person','media_type': 'img'})
        return options
    
    def _selectFreeSitToPointAt(self, result, choose_couch_if_exist = False):
        chair_and_couch_list = result[LTPerception.RESPONSE_LABEL_DETAILS]
         
        chair_list = []
        couch_list = []
        distance = 9999
        for free_space in chair_and_couch_list:
            if free_space["class"] == self.YOLO_CLASS_CHAIR:
                chair_list.append(free_space)
            elif free_space["class"] == self.YOLO_CLASS_COUCH:
                if choose_couch_if_exist:
                    return free_space
                couch_list.append(free_space)
        
        #select suitable chair
        closest_distance = 9999
        selected_free_sit = None

        for chair in chair_list:
            if chair.pose != None:
                if chair.pose.pose.position.x < closest_distance:
                    selected_free_sit = chair
        return selected_free_sit
    
    def _selectPersonToPointAt(self, result):
        people_list = result[LTPerception.RESPONSE_LABEL_DETAILS]
        selected_people = None
        distance = 9999
        for people in people_list:
            if people["pose"] != None:
                if people["pose"].pose.position.x <distance:
                    distance = people["pose"].pose.position.x
                    selected_people = people
        return selected_people
    
    def _ask_angle_to_point_at_pose(self,pose):
        """
        Get angle in rad between current robot pose in odom
        and given pose
        - params
          - pose: PoseStamped with the frame_id set (use for tf transform into odom)
        - return:
          - angle: Angle in rad.
        """
        angle = 0.0
        try:
            #transform given pose to Odom
            transformed_pose = transform_pose(pose,pose.header.frame_id,'odom')

            angleCurTarget = math.atan2( transformed_pose.pose.y - self.curPose2D.y , transformed_pose.pose.x - self.curPose2D.x )
            angle = self.shortestAngleDiff(angleCurTarget, self.curPose2D.theta)
        except Exception as e:
            rospy.logwarn("[ReceptionistScenarioV1] Error occured when finding point at angle "+ str(e))
        return angle

    def shortestAngleDiff(self, th1, th2):
        """
            Returns the shortest angle between 2 angles in the trigonometric circle
        """        
        anglediff = math.fmod( (th1 - th2), 2*math.pi)

        if anglediff < 0.0:
            if math.fabs(anglediff) > (2*math.pi + anglediff) :
                anglediff = 2*math.pi + anglediff
        else:
            if anglediff > math.fabs(anglediff - 2*math.pi) :
                anglediff = anglediff - 2*math.pi

        return anglediff
    

#******************************************************************************************
#*************************************   CALLBACK   **************************************
#******************************************************************************************

    def odomCallback(self, odom):
        """
            Odometry callback
            Set curPose2D with current robot x y theta
        """
        o = deepcopy(odom.pose.pose.orientation)

        orientation_list = [o.x, o.y, o.z, o.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        self.curPose2D.x = odom.pose.pose.position.x
        self.curPose2D.y = odom.pose.pose.position.y
        self.curPose2D.theta = yaw  
        rospy.loginfo("Odom => X: %.2f \t Y: %.2f \t theta: %.2f"  % (self.curPose2D.x, self.curPose2D.y, self.curPose2D.theta ) )


    