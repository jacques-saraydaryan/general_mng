__author__ = 'Jacques Saraydaryan'

from abc import ABCMeta, abstractmethod
from LTAbstract import LTAbstract
from LTServiceResponse import LTServiceResponse
from actionlib_msgs.msg import GoalStatus

import actionlib
import rospy
from rospy.exceptions import ROSException, ROSInterruptException

from object_management.msg import LookAtObjectAction, LookAtObjectGoal, LookAtObjectResult
from dialogue_hri_srvs.srv import ReleaseArms
from pepper_pose_for_nav.srv import MoveHeadAtPosition
from dialogue_hri_srvs.srv import MoveTurn
from dialogue_hri_srvs.srv import PointAt
from dialogue_hri_srvs.srv import TurnToInterestPoint
from convert_2d_to_3d.srv import SwitchMode
from pmb2_apps.msg import ArmControlGoal, ArmControlAction
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
def singleton(cls):   
    """
    Enables the system to create at most one instance of the class. Two instances of the same class can't be running at the same time.
    """
    instance = [None]
    def wrapper(*args, **kwargs):
        if instance[0] is None:
            instance[0] = cls(*args, **kwargs)
        return instance[0]

    return wrapper

@singleton
class LTMotionPalbator(LTAbstract):

    _enableArmControlAction = True

    def __init__(self):
        """
        Initializes the LTMotion API for Palbator. It will deal with every function related to Palbator movements.
        """
        self.configure_intern()

        #Inform configuration is ready
        self.configurationReady = True
        rospy.loginfo("{class_name}: LTMotionPalbator initialized".format(class_name=self.__class__.__name__))

    #######################################
    # CONFIGURATION
    ######################################


    def configure_intern(self):
        """
        Loads the configuration needed to use correctly every motion function for Palbator.
        """
        if self._enableArmControlAction:
            service_name ="merge_register_data_switch_config"
            self.switch_config = rospy.ServiceProxy(service_name, SwitchMode)
            try:
                rospy.wait_for_service(service_name,timeout = self.SERVICE_WAIT_TIMEOUT)
                rospy.loginfo("{class_name}: merge_register_data_switch_config server connected".format(class_name=self.__class__.__name__))
            except (ROSException, ROSInterruptException) as e:
                rospy.logwarn("{class_name}: Unable to connect to the merge_register_data_switch_config service.".format(class_name=self.__class__.__name__))
            
            action_server_name = "/Moveit_Palbator_global_action"
            self._action_client_arm_control = actionlib.SimpleActionClient(action_server_name,ArmControlAction)
            server_is_up = self._action_client_arm_control.wait_for_server(timeout=rospy.Duration(self.ACTION_WAIT_TIMEOUT))
            
            if server_is_up:
                rospy.loginfo("{class_name}: Palbator Moveit control server connected".format(class_name=self.__class__.__name__))
            else:
                rospy.logwarn("{class_name}: Palbator Moveit control server disconnected".format(class_name=self.__class__.__name__))


    def reset(self):
        """
        Reloads the configuration needed to use correctly every motion function for Palbator.
        """
        self.configure_intern()

    #######################################
    # MOTION API
    ######################################
    def arm_controller(self, action, service_mode=LTAbstract.ACTION):
        """
        Will send a request to the Moveit Global controller to move the robot into a pose to travel without risks (to avoid arm collisions with walls for instance).
        Returns a response containing the result, the status and the feedback of the executed action.
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__arm_controller,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for set_palbator_ready_to_travel" % (service_mode)
            return response
        else:
            feedback, result = fct(action)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during " + str(action)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes " + str(action)
                response.result = result
                return response
        return response

    def set_palbator_ready_to_travel(self,service_mode=LTAbstract.ACTION):
        """
        Will send a request to the Moveit Global controller to move the robot into a pose to travel without risks (to avoid arm collisions with walls for instance).
        Returns a response containing the result, the status and the feedback of the executed action.
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__set_palbator_ready_to_travel,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for set_palbator_ready_to_travel" % (service_mode)
            return response
        else:
            feedback, result = fct()
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during set_palbator_ready_to_travel"
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes set_palbator_ready_to_travel" 
                response.result = result
                return response
        return response

    def point_at_object(self, object_label, service_mode=LTAbstract.ACTION):
        """
        Will send a request to the Moveit Global controller to point an object.
        Returns a response containing the result, the status and the feedback of the executed action.
        :param object_label: name of the target object
        :type object_label: string
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__point_at_object,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for point_at_object" % (service_mode)
            return response
        else:
            feedback, result = fct(object_label)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during point_at_object to object: %s" % (object_label)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes point_at_object to object: %s" % (object_label)
                response.result = result
                return response
        return response

    def look_at_object(self, object_label, service_mode=LTAbstract.ACTION):
        """
        Will send a request to the Moveit Global controller to point an object.
        Returns a response containing the result, the status and the feedback of the executed action.
        :param object_label: name of the target object
        :type object_label: string
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__look_at_object,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for look_at_object" % (service_mode)
            return response
        else:
            feedback, result = fct(object_label)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during look_at_object to object: %s" % (object_label)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes look_at_object to object: %s" % (object_label)
                response.result = result
                return response
        return response


    def look_at_object_XYZ(self, coord_x, coord_y, coord_z, service_mode=LTAbstract.ACTION):
        """
        Will send a request to the Moveit Global controller to point an object.
        Returns a response containing the result, the status and the feedback of the executed action.
        :param object_label: name of the target object
        :type object_label: string
        """
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__look_at_object_XYZ,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for look_at_object XYZ" % (service_mode)
            return response
        else:
            feedback, result = fct(coord_x, coord_y, coord_z)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during look_at_object XYZ: %s" % ([coord_x, coord_y, coord_z])
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes look_at_object XYZ: %s" % ([coord_x, coord_y, coord_z])
                response.result = result
                return response
        return response

    def catch_object_label(self, object_label, service_mode=LTAbstract.ACTION):
        """
        Will send a request to the Moveit Global controller to point an object.
        Returns a response containing the result, the status and the feedback of the executed action.
        :param object_label: name of the target object
        :type object_label: string
        """

        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__catch_object_label,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for catch_object_label" % (service_mode)
        else:
            feedback, result = fct(object_label)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during catch_object_label to object: %s" % (object_label)
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes catch_object_label to object: %s" % (object_label)
                response.result = result
        
        return response

    def catch_object_XYZ(self, coord_x, coord_y, coord_z, dimensions, service_mode=LTAbstract.ACTION):
        """
        Will send a request to the Moveit Global controller to point an object.
        Returns a response containing the result, the status and the feedback of the executed action.
        :param coord_x: x coord of the target object
        :param coord_y: y coord of the target object
        :param coord_z: z coord of the target object
        :param dimensions : size dimensions of the target object
        :type coord_x: float32
        :type coord_y: float32
        :type coord_z: float32
        """

        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__catch_object_XYZ,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for catch_object_XYZ" % (service_mode)

        else:
            feedback, result = fct(coord_x, coord_y, coord_z, dimensions)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during catch_object_XYZ at: %i %i %i" % (coord_x, coord_y, coord_z)
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes catch_object_XYZ to object: %i %i %i" % (coord_x, coord_y, coord_z)
                response.result = result


        return response


    def dropping_XYZ(self, coord_x, coord_y, coord_z, service_mode=LTAbstract.ACTION):
        """
        Will send a request to the Moveit Global controller to point an object.
        Returns a response containing the result, the status and the feedback of the executed action.
        :param coord_x: x coord of the target object
        :param coord_y: y coord of the target object
        :param coord_z: z coord of the target object
        :type coord_x: float32
        :type coord_y: float32
        :type coord_z: float32
        """

        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__dropping_XYZ,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for dropping_xyz" % (service_mode)

        else:
            feedback, result = fct(coord_x, coord_y, coord_z)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during dropping_xyz at: %i %i %i" % (coord_x, coord_y, coord_z)
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes dropping_xyz to object: %i %i %i" % (coord_x, coord_y, coord_z)
                response.result = result


        return response

    def dropping_label(self, object_label, service_mode=LTAbstract.ACTION):
        """
        Will send a request to the Moveit Global controller to point an object.
        Returns a response containing the result, the status and the feedback of the executed action.
        :param object_label: name of the target object
        :type object_label: string
        """
        

        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__dropping_label,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for dropping_label" % (service_mode)
        else:
            feedback, result = fct(object_label)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during dropping_label to object: %s" % (object_label)
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes dropping_label to object: %s" % (object_label)
                response.result = result


        return response

    def namo_XYZ(self, coord_x, coord_y, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__namo_XYZ,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for namo_XYZ" % (service_mode)

        else:
            feedback, result = fct(coord_x, coord_y)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during namo_XYZ at: %i %i" % (coord_x, coord_y)
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes namo_XYZ to object: %i %i" % (coord_x, coord_y)
                response.result = result


        return response

    #######################################
    # MOTION ACTION
    ######################################
    def __arm_controller(self, action):
        """
        Action client which will send a request to the Moveit Global controller to move the robot into a pose to travel without risks (to avoid arm collisions with walls for instance).
        Returns a GoalStatus and an action result.
        """
        try:
            goal = ArmControlGoal()
            goal.action = action
            self._action_client_arm_control.send_goal(goal)
            rospy.loginfo("{class_name}: SENDING %s GOAL".format(class_name=self.__class__.__name__), str(action))
            self._action_client_arm_control.wait_for_result()
            result = self._action_client_arm_control.get_result()
            return GoalStatus.SUCCEEDED, result
        except Exception as e:
            rospy.logerr("{class_name}: Action set_palbator_ready_to_travel could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None

    def __set_palbator_ready_to_travel(self):
        """
        Action client which will send a request to the Moveit Global controller to move the robot into a pose to travel without risks (to avoid arm collisions with walls for instance).
        Returns a GoalStatus and an action result.
        """
        try:
            goal = ArmControlGoal()
            goal.action = "Travelling"
            self._action_client_arm_control.send_goal(goal)
            rospy.loginfo("{class_name}: SETTING PAL IN TRAVEL MODE".format(class_name=self.__class__.__name__))
            self._action_client_arm_control.wait_for_result()
            rospy.loginfo("{class_name}: RESULT OF SETTING".format(class_name=self.__class__.__name__))
            result = self._action_client_arm_control.get_result()
            return GoalStatus.SUCCEEDED, result
        except Exception as e:
            rospy.logerr("{class_name}: Action set_palbator_ready_to_travel could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None

    def __point_at_object(self,object_label):
        """
        Action client which will send a request to the Moveit Global controller to point an object.
        Returns a GoalStatus and an action result.
        :param object_label: name of the target object
        :type object_label: string
        """
        try:
            goal = ArmControlGoal()
            goal.action = 'Pointing'
            goal.object_label = object_label
            self._action_client_arm_control.send_goal(goal)
            rospy.loginfo("{class_name}: SENDING POINTING GOAL".format(class_name=self.__class__.__name__))
            self._action_client_arm_control.wait_for_result()
            result = self._action_client_arm_control.get_result()
            return GoalStatus.SUCCEEDED, result

        except Exception as e:
            rospy.logerr("{class_name}: Action point_at_object could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None

    def __look_at_object(self,object_label):
        """
        Action client which will send a request to the Moveit Global controller to point an object.
        Returns a GoalStatus and an action result.
        :param object_label: name of the target object
        :type object_label: string
        """
        try:
            goal = ArmControlGoal()
            goal.action = 'Looking'
            goal.object_label = object_label
            self._action_client_arm_control.send_goal(goal)
            rospy.loginfo("{class_name}: SENDING LOOKING GOAL".format(class_name=self.__class__.__name__))
            self._action_client_arm_control.wait_for_result()
            result = self._action_client_arm_control.get_result()
            return GoalStatus.SUCCEEDED, result

        except Exception as e:
            rospy.logerr("{class_name}: Action look_at_object could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None

    def __look_at_object_XYZ(self, coord_x, coord_y, coord_z):
        """
        Action client which will send a request to the Moveit Global controller to point an object.
        Returns a GoalStatus and an action result.
        :param object_label: name of the target object
        :type object_label: string
        """
        try:
            goal = ArmControlGoal()
            goal.action = 'LookingXYZ'
            pose = Pose()
            pose.position.x = coord_x
            pose.position.y = coord_y
            pose.position.z = coord_z
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            goal.pose = pose
            self._action_client_arm_control.send_goal(goal)
            rospy.loginfo("{class_name}: SENDING LOOKING XYZ GOAL".format(class_name=self.__class__.__name__))
            self._action_client_arm_control.wait_for_result()
            result = self._action_client_arm_control.get_result()
            return GoalStatus.SUCCEEDED, result

        except Exception as e:
            rospy.logerr("{class_name}: Action look_at_object XYZ could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None


    def __catch_object_label(self, object_label):
        """
        Action client which will send a request to the Moveit Global controller to point an object.
        Returns a GoalStatus and an action result.
        :param object_label: name of the target object
        :type object_label: string
        """
        try:
            goal = ArmControlGoal()
            goal.action = 'Grasping'
            goal.object_label = object_label
            rospy.logwarn("{class_name}: CATCHING LABEL GOAL: %s", goal)
            self._action_client_arm_control.send_goal(goal)
            rospy.loginfo("{class_name}: SENDING CATCHING LABEL GOAL".format(class_name=self.__class__.__name__))
            self._action_client_arm_control.wait_for_result()
            result = self._action_client_arm_control.get_result()
            return GoalStatus.SUCCEEDED, result

        except Exception as e:
            rospy.logerr("{class_name}: Action catch_object could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None
    
    def __catch_object_XYZ(self, coord_x, coord_y, coord_z, dimensions):
        """
        Action client which will send a request to the Moveit Global controller to point an object.
        Returns a GoalStatus and an action result.
        :param coord_x: x coord of the target object
        :param coord_y: y coord of the target object
        :param coord_z: z coord of the target object
        :type coord_x: float32
        :type coord_y: float32
        :type coord_z: float32
        :param dimensions : size dimensions of the target object
        """
        try:
            goal = ArmControlGoal()
            goal.action = 'GraspingXYZ'
            pose = Pose()
            pose.position.x = coord_x
            pose.position.y = coord_y
            pose.position.z = coord_z
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            goal.pose = pose
            solid = SolidPrimitive()
            solid.type = 1
            solid.dimensions = dimensions
            goal.solidPrimitive = solid
            self._action_client_arm_control.send_goal(goal)
            rospy.loginfo("{class_name}: SENDING CATCHING XYZ GOAL".format(class_name=self.__class__.__name__))
            self._action_client_arm_control.wait_for_result()
            result = self._action_client_arm_control.get_result()
            return GoalStatus.SUCCEEDED, result

        except Exception as e:
            rospy.logerr("{class_name}: Action catch_object could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None

    def __dropping_XYZ(self, coord_x, coord_y, coord_z):
        """
        Action client which will send a request to the Moveit Global controller to point an object.
        Returns a GoalStatus and an action result.
        :param coord_x: x coord of the target object
        :param coord_y: y coord of the target object
        :param coord_z: z coord of the target object
        :type coord_x: float32
        :type coord_y: float32
        :type coord_z: float32
        """
        try:
            goal = ArmControlGoal()
            goal.action = 'DroppingXYZ'
            pose = Pose()
            pose.position.x = coord_x
            pose.position.y = coord_y
            pose.position.z = coord_z
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            goal.pose = pose
            self._action_client_arm_control.send_goal(goal)
            rospy.loginfo("{class_name}: SENDING DROPPING XYZ GOAL".format(class_name=self.__class__.__name__))
            self._action_client_arm_control.wait_for_result()
            result = self._action_client_arm_control.get_result()
            return GoalStatus.SUCCEEDED, result

        except Exception as e:
            rospy.logerr("{class_name}: Action dropping_xyz could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None

    def __dropping_label(self, object_label):
        """
        Action client which will send a request to the Moveit Global controller to point an object.
        Returns a GoalStatus and an action result.
        :param object_label: name of the target object
        :type object_label: string
        """
        try:
            goal = ArmControlGoal()
            goal.action = 'Dropping'
            goal.object_label = object_label
            rospy.logwarn("{class_name}: DROPPING LABEL GOAL: %s", goal)
            self._action_client_arm_control.send_goal(goal)
            rospy.loginfo("{class_name}: SENDING DROPPING LABEL GOAL".format(class_name=self.__class__.__name__))
            self._action_client_arm_control.wait_for_result()
            result = self._action_client_arm_control.get_result()
            return GoalStatus.SUCCEEDED, result

        except Exception as e:
            rospy.logerr("{class_name}: Action dropping_label could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None

    def __namo_XYZ(self, coord_x, coord_y):
        try:
            goal = ArmControlGoal()
            goal.action = 'NamoXYZ'
            pose = Pose()
            pose.position.x = coord_x
            pose.position.y = coord_y
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            goal.pose = pose
            self._action_client_arm_control.send_goal(goal)
            rospy.loginfo("{class_name}: SENDING NAMO XYZ GOAL".format(class_name=self.__class__.__name__))
            self._action_client_arm_control.wait_for_result()
            result = self._action_client_arm_control.get_result()
            return GoalStatus.SUCCEEDED, result

        except Exception as e:
            rospy.logerr("{class_name}: Action namo_xyz could not process request: {error}".format(class_name=self.__class__.__name__,error=e))
            return GoalStatus.ABORTED, None
class LTMotion(LTAbstract):

    HEAD_PITCH_CENTER = 0.0
    HEAD_PITCH_FOR_SPEECH_POSE = -0.30
    HEAD_PITCH_FOR_LOOK_AT_PEOPLE = -0.5
    HEAD_PITCH_FOR_LOOK_FOR_CHAIR = 0.1
    HEAD_PITCH_FOR_NAV_POSE = 0.5
    HEAD_YAW_CENTER = 0.0
    RELEASE_ARM_STIFFNESS = 0.6

    _enableLookAtObjectMngAction = True
    _enableMoveHeadPoseService = True
    _enableMoveTurnService = True
    _enablePointAtService = True
    _enableReleaseArmsService = True

    def __init__(self):
        self.configure_intern()

        #Inform configuration is ready
        self.configurationReady = True

    #######################################
    # CONFIGURATION
    ######################################

    def configure_intern(self):
        rospy.loginfo("Connecting to motion action server ... ")

        if self._enableLookAtObjectMngAction:
            self._actionLookAtObjectMng_server = actionlib.SimpleActionClient('look_at_object_action',
                                                                              LookAtObjectAction)
            finished6 = self._actionLookAtObjectMng_server.wait_for_server(
                timeout=rospy.Duration(self.ACTION_WAIT_TIMEOUT))
            if finished6:
                rospy.loginfo("LookAtObjectMng Connected")
            else:
                rospy.logwarn("Unable to connect to LookAtObjectMng action server")

        # Connect to move_head_service service
        if self._enableMoveHeadPoseService:
            rospy.loginfo("Connecting to the move_head_pose_srv service...")
            self._moveHeadPoseSP = rospy.ServiceProxy('move_head_pose_srv', MoveHeadAtPosition)
            try:
                move_head_pose_srv_is_up = rospy.wait_for_service('move_head_pose_srv', timeout = self.SERVICE_WAIT_TIMEOUT)
                rospy.loginfo("Connected to the move_head_pose_srv service.")
            except (ROSException, ROSInterruptException) as e:
                rospy.logwarn("Unable to connect to the move_head_pose_srv service.")

        # Connect to move_turn_service service
        if self._enableMoveTurnService:
            rospy.loginfo("Connecting to the move_turn_srv service...")
            self._moveTurnSP = rospy.ServiceProxy('move_turn_service', MoveTurn)
            try:
                move_turn_srv_is_up = rospy.wait_for_service('move_turn_service', timeout = self.SERVICE_WAIT_TIMEOUT)
                rospy.loginfo("Connected to the move_turn_service service.")
            except (ROSException, ROSInterruptException) as e:
                rospy.logwarn("Unable to connect to the move_turn_service service.")
        # Connect to point_at service
        if self._enablePointAtService:
            rospy.loginfo("Connecting to the point_at service...")
            self._pointAtSP = rospy.ServiceProxy('point_at', PointAt)
            try:
                point_at_srv_is_up = rospy.wait_for_service('point_at', timeout = self.SERVICE_WAIT_TIMEOUT)
                rospy.loginfo("Connected to the point_at service.")
            except (ROSException, ROSInterruptException) as e:
                rospy.logwarn("Unable to connect to the point_at service.")

        if self._enableReleaseArmsService:
            rospy.loginfo("Connecting to the release_arms service...")
            self._releaseArmsSP = rospy.ServiceProxy('release_arms', ReleaseArms)
            try:
                release_arms_srv_is_up = rospy.wait_for_service('release_arms', timeout = self.SERVICE_WAIT_TIMEOUT)
                rospy.loginfo("Connected to the release_arms service.")
            except (ROSException, ROSInterruptException) as e:
                rospy.logwarn("Unable to connect to the release_arms service.")

        # Connect to turn_to_interest_point service
        if self._enableTurnToInterestPointService:
            rospy.loginfo("Connecting to the turn_to_interest_point service...")
            self._turnToInterestPointSP = rospy.ServiceProxy('turn_to_interest_point', TurnToInterestPoint)
            try:
                turn_to_interest_point_srv_is_up = rospy.wait_for_service('turn_to_interest_point', timeout = self.SERVICE_WAIT_TIMEOUT)
                rospy.loginfo("Connected to the turn_to_interest_point service.")
            except (ROSException, ROSInterruptException) as e:
                rospy.logwarn("Unable to connect to the turn_to_interest_point service.")


    def reset(self):
        self.configure_intern()

    #######################################
    # MOTION API
    ######################################
    def look_at_object(self, labels, index, head, base, finger, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self.__look_at_object,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for look_at_object" % (service_mode)
            return response
        else:
            feedback, result = fct(labels, index, head, base, finger)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during look_at_object to labels:[%s], index:[%s], head[%s], base[%s], finger[%s]" % (
                labels, index, head, base, finger)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes look_at_object to labels:[%s], index:[%s], head[%s], base[%s], finger[%s]" % (
                labels, index, head, base, finger)
                response.result = result
                return response
        return response

    def move_head_pose(self, pitch_value, yaw_value, track, service_mode=LTAbstract.SERVICE):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: None,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: self.__move_head_pose
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for move_head_pose" % (service_mode)
            return response
        else:
            feedback, result = fct(pitch_value, yaw_value, track)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during move_head_pose to pitch_value:[%s], yaw_value:[%s], track[%s]" % (
                pitch_value, yaw_value, track)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes move_head_pose to pitch_value:[%s], yaw_value:[%s], track[%s]" % (
                pitch_value, yaw_value, track)
                response.result = result
                return response
        return response

    def move_turn(self, rad, service_mode=LTAbstract.SERVICE):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: None,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: self.__move_turn
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for move_turn" % (service_mode)
            return response
        else:
            feedback, result = fct(rad)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during move_turn to rad:[%s]" % (rad)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes move_turn to rad:[%s]" % (rad)
                response.result = result
                return response
        return response

    def point_at(self, x, y, z, head, arm, duration, service_mode=LTAbstract.SERVICE):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: None,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: self.__point_at
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for point_at" % (service_mode)
            return response
        else:
            feedback, result = fct(x, y, z, head, arm, duration)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during point_at to x:[%s], y:[%s], z:[%s], head:[%s], arm:[%s], duration:[%s]" % (
                x, y, z, head, arm, duration)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes point_at to x:[%s], y:[%s], z:[%s], head:[%s], arm:[%s], duration:[%s]" % (
                x, y, z, head, arm, duration)
                response.result = result
                return response
        return response


    def release_arms(self, service_mode=LTAbstract.SERVICE):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: None,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: self.__release_arms
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for release_arms" % (service_mode)
            return response
        else:
            feedback, result = fct()
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during release_arms "
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes release_arms"
                response.result = result
                return response
        return response


    def turn_to_interest_point(self, interest_point_label, service_mode=LTAbstract.SERVICE):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: None,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: self.__turn_to_interest_point
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = "[%s] is not available for turn_to_interest_point" % (service_mode)
            return response
        else:
            feedback, result = fct(interest_point_label)
            response.process_state(feedback)
            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during turn_to_interest_point interest_point_label:[%s] " % (interest_point_label)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation succes turn_to_interest_point interest_point_label:[%s] " % (interest_point_label)
                response.result = result
                return response
        return response

    #######################################
    # MOTION ACTION
    ######################################

    def __turn_to_interest_point(self, interest_point_label):
        try:
            result = self._turnToInterestPointSP(interest_point_label)
            return GoalStatus.SUCCEEDED, result
        except rospy.ServiceException as e:
            rospy.logerr("Service turn_to_interest_point_srv could not process request: {error}".format(error=e))
            return GoalStatus.ABORTED, None
        except Exception as e:
            rospy.logerr("Service turn_to_interest_point_srv could not process request: {error}".format(error=e))
            return GoalStatus.ABORTED, None

    def __look_at_object(self, labels, index, head, base, finger, timeout):
        try:
            goalLookAtObj = LookAtObjectGoal()
            goalLookAtObj.labels = labels
            goalLookAtObj.index = index
            goalLookAtObj.head = head
            goalLookAtObj.base = base
            goalLookAtObj.finger = finger

            rospy.loginfo("### LOOK AT OBJECT MNG GET OBJECT ACTION PENDING : %s",
                          str(goalLookAtObj).replace('\n', ', '))

            # send the current goal to the action server
            self._actionLookAtObjectMng_server.send_goal(goalLookAtObj)
            # wait action server result
            finished_before_timeout = self._actionLookAtObjectMng_server.wait_for_result(
                rospy.Duration.from_sec(timeout))
            state = self._actionLookAtObjectMng_server.get_state()
            result = self._actionLookAtObjectMng_server.get_result()
            rospy.loginfo("###### LOOK AT OBJECT MNG GET OBJECT ACTION END , State: %s", str(state))
            # if timeout cancel all goals on the action server
            if finished_before_timeout:
                self._actionLookAtObjectMng_server.cancel_all_goals()
            # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
            return state, result
        except Exception as e:
            rospy.logwarn("###### LOOK AT OBJECT MNG ACTION FAILURE , State: %s", str(e))
        return GoalStatus.ABORTED, None

    def __move_head_pose(self, pitch_value, yaw_value, track):
        try:
            result = self._moveHeadPoseSP(pitch_value, yaw_value, track)
            return GoalStatus.SUCCEEDED, result

        except rospy.ServiceException as e:
            rospy.logerr("Service move_head_pose_srv could not process request: {error}".format(error=e))
            return GoalStatus.ABORTED, None
        except Exception as e:
            rospy.logerr("Service move_head_pose_srv could not process request: {error}".format(error=e))
            return GoalStatus.ABORTED, None

    def __move_turn(self, rad):
        try:
            result = self._moveTurnSP(rad)
            return GoalStatus.SUCCEEDED, result
        except rospy.ServiceException as e:
            rospy.logerr("Service move_turn_service could not process request: {error}".format(error=e))
            return GoalStatus.ABORTED, None
        except Exception as e:
            rospy.logerr("Service move_turn_service could not process request: {error}".format(error=e))
            return GoalStatus.ABORTED, None

    def __point_at(self, x, y, z, head, arm, duration):
        try:
            result = self._pointAtSP(x, y, z, head, arm, duration)
            return GoalStatus.SUCCEEDED, result
        except rospy.ServiceException as e:
            rospy.logerr("Service point_at could not process request: {error}".format(error=e))
            return GoalStatus.ABORTED, None
        except Exception as e:
            rospy.logerr("Service point_at could not process request: {error}".format(error=e))
            return GoalStatus.ABORTED, None

    def __release_arms(self):
        try:
            result = self._releaseArmsSP(self.RELEASE_ARM_STIFFNESS)
            return GoalStatus.SUCCEEDED, result
        except rospy.ServiceException as e:
            rospy.logerr("Service release_arms could not process request: {error}".format(error=e))
            return GoalStatus.ABORTED, None
        except Exception as e:
            rospy.logerr("Service release_arms could not process request: {error}".format(error=e))
            return GoalStatus.ABORTED, None
