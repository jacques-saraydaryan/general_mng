__author__ = 'Jacques Saraydaryan'

from abc import ABCMeta, abstractmethod
from LTAbstract import LTAbstract
from LTServiceResponse import LTServiceResponse

import actionlib
import rospy

from ros_people_mng_actions.msg import ProcessPeopleFromImgAction, ProcessPeopleFromImgGoal
from ros_people_mng_actions.msg import LearnPeopleFromImgAction, LearnPeopleFromImgGoal
from ros_people_mng_actions.msg import GetPeopleNameFromImgAction, GetPeopleNameFromImgGoal

from object_management.msg import ObjectDetectionAction, ObjectDetectionGoal

from actionlib_msgs.msg import GoalStatus
import cv2
from rospy.exceptions import ROSException, ROSInterruptException
from std_srvs.srv import Trigger
from pepper_door_open_detector.srv import MinFrontValue


class LTPerception(LTAbstract):

    OPEN_DOOR_MIN_DISTANCE = 0.8
    CHECK_DISTANCE_FREQUENCY = 10

    _enableObjectDetectionMngAction = True
    _enableLearnPeopleMetaAction = True
    _enableMultiplePeopleDetectionAction = True
    _enableGetPeopleNameAction = True
    _enableMinFrontValueService = False
    _enableResetPersonMetaInfoMapService = False

    def __init__(self):
        self.configure_intern()

        #Inform configuration is ready
        self.configurationReady = True

    #######################################
    # CONFIGURATION
    ######################################

    def configure_intern(self):
        rospy.loginfo("Connecting to tts_hri action server ... ")

        if self._enableObjectDetectionMngAction:
            self._actionObjectDetectionMng_server = actionlib.SimpleActionClient('object_detection_action',
                                                                                 ObjectDetectionAction)
            finished5 = self._actionObjectDetectionMng_server.wait_for_server(
                timeout=rospy.Duration(self.ACTION_WAIT_TIMEOUT))
            if finished5:
                rospy.loginfo("ObjectDetectionMng Connected")
            else:
                rospy.logwarn("Unable to connect to ObjectDetectionMng action server")

        if self._enableLearnPeopleMetaAction:
            self._actionLearnPeopleMeta_server = actionlib.SimpleActionClient('learn_people_meta_action',
                                                                              LearnPeopleFromImgAction)
            finished7 = self._actionLearnPeopleMeta_server.wait_for_server(
                timeout=rospy.Duration(self.ACTION_WAIT_TIMEOUT))
            if finished7:
                rospy.loginfo("learnPeopleMeta action server Connected")
            else:
                rospy.logwarn("Unable to connect to learnPeopleMeta action server")

        if self._enableMultiplePeopleDetectionAction:
            self._actionMultiplePeopleDetection_server = actionlib.SimpleActionClient('detect_people_meta_action',
                                                                                      ProcessPeopleFromImgAction)
            finished8 = self._actionMultiplePeopleDetection_server.wait_for_server(
                timeout=rospy.Duration(self.ACTION_WAIT_TIMEOUT))
            if finished8:
                rospy.loginfo("MultiplePeopleDetection Connected")
            else:
                rospy.logwarn("Unable to connect to MultiplePeopleDetection action server")

        if self._enableGetPeopleNameAction:
            self._actionGetPeopleName_server = actionlib.SimpleActionClient('get_people_name_action',
                                                                            GetPeopleNameFromImgAction)
            finished9 = self._actionGetPeopleName_server.wait_for_server(
                timeout=rospy.Duration(self.ACTION_WAIT_TIMEOUT))
            if finished9:
                rospy.loginfo("GetPeopleName Connected")
            else:
                rospy.logwarn("Unable to connect to GetPeopleName action server")

        if self._enableResetPersonMetaInfoMapService:
            rospy.loginfo("Connecting to the reset_people_meta_info_map_srv service...")
            self._resetPeopleMetaInfoMapSP = rospy.ServiceProxy('reset_people_meta_info_map_srv', Trigger)
            try:
                reset_people_meta_info_map_srv_is_up = rospy.wait_for_service('reset_people_meta_info_map_srv',
                                                                              timeout = self.SERVICE_WAIT_TIMEOUT)
                rospy.loginfo("Connected to the reset_people_meta_info_map_srv service.")
            except (ROSException, ROSInterruptException) as e:
                rospy.logwarn("Unable to connect to the reset_people_meta_info_map_srv service.")

        if self._enableMinFrontValueService:
            rospy.loginfo("Connecting to the min_front_value_srv service...")
            self._minFrontValueSP = rospy.ServiceProxy('min_front_value_srv', MinFrontValue)
            try:
                min_front_value_srv_is_up = rospy.wait_for_service('min_front_value_srv', timeout = self.SERVICE_WAIT_TIMEOUT)
                rospy.loginfo("Connected to the min_front_value_srv service.")
            except (ROSException, ROSInterruptException) as e:
                rospy.logwarn("Unable to connect to the min_front_value_srv service.")

    def reset(self):
        self.configure_intern()

    #######################################
    # PERCEPTION API
    ######################################

    def get_object_in_front_robot(self, labels, move_head, timeout, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self._get_object_in_front_robot,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for get_object_in_front_robot" % (service_mode)
            return response
        else:
            feedback, result = fct(labels, move_head, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during get_object_in_front_robot to labels:[%s], move_head:[%s]" % (
                    labels, move_head)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success get_object_in_front_robot to to labels:[%s], move_head:[%s]" % (
                    labels, move_head)
                response.payload = result
                return response
        return response

    def learn_people_meta_from_img_topic(self, name, timeout, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self._learn_people_meta_from_img_topic,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for learn_people_meta_from_img_topic" % (service_mode)
            return response
        else:
            feedback, result = fct(name, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during learn_people_meta_from_img_topic to name:[%s]" % (
                    name)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success learn_people_meta_from_img_topic to name:[%s]" % (
                    name)
                response.payload = result
                return response
        return response

    def learn_people_meta_from_img_path(self, img_path, name, timeout, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self._learn_people_meta_from_img_path,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for learn_people_meta_from_img_path" % (service_mode)
            return response
        else:
            feedback, result = fct(img_path, name, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during _learn_people_meta_from_img_path to img_path:[%s] name:[%s]" % (
                    img_path, name)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success _learn_people_meta_from_img_path to img_path:[%s] name:[%s]" % (
                    img_path, name)
                response.payload = result
                return response
        return response

    def learn_people_meta(self, goalLearnPeople, timeout, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self._learn_people_meta,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for learn_people_meta" % (service_mode)
            return response
        else:
            feedback, result = fct(goalLearnPeople, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during learn_people_meta to goalLearnPeople:[%s] " % (
                    goalLearnPeople)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success learn_people_meta to goalLearnPeople:[%s] " % (
                    goalLearnPeople)
                response.payload = result
                return response
        return response

    def detect_meta_people_from_img_topic(self, timeout, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self._detect_meta_people,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for detect_meta_people_from_img_topic" % (service_mode)
            return response
        else:
            feedback, result = fct(timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during detect_meta_people_from_img_topic "
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success detect_meta_people_from_img_topic "
                response.payload = result
                return response
        return response

    def detect_meta_people_from_img_path(self, img_path, timeout, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self._detect_meta_people_from_img_path,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for detect_meta_people_from_img_path" % (service_mode)
            return response
        else:
            feedback, result = fct(img_path, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during detect_meta_people_from_img_path to img_path:[%s] " % (
                    img_path)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success detect_meta_people_from_img_path to img_path:[%s]" % (
                    img_path)
                response.payload = result
                return response
        return response

    def detect_meta_people(self, goalMetaPeople, timeout, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self._detect_meta_people,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for detect_meta_people" % (service_mode)
            return response
        else:
            feedback, result = fct(goalMetaPeople, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during detect_meta_people to goalMetaPeople:[%s] " % (
                    goalMetaPeople)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success detect_meta_people to goalMetaPeople:[%s]" % (
                    goalMetaPeople)
                response.payload = result
                return response
        return response

    def get_people_name_from_img_topic(self, timeout, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self._get_people_name_from_img_topic,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for get_people_name_from_img_topic" % (service_mode)
            return response
        else:
            feedback, result = fct(timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during get_people_name_from_img_topic "
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success get_people_name_from_img_topic "
                response.payload = result
                return response
        return response

    def get_people_name(self, timeout, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self._get_people_name,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for get_people_name" % (service_mode)
            return response
        else:
            feedback, result = fct(timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during get_people_name "
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success get_people_name "
                response.payload = result
                return response
        return response

    def get_people_name_from_img_path(self, goalPeopleName, timeout, service_mode=LTAbstract.ACTION):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: self._get_people_name_from_img_path,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: None
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for get_people_name_from_img_path" % (service_mode)
            return response
        else:
            feedback, result = fct(goalPeopleName, timeout)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during get_people_name_from_img_path to goalPeopleName:[%s] " % (
                    goalPeopleName)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success get_people_name_from_img_path to goalPeopleName:[%s]" % (
                    goalPeopleName)
                response.payload = result
                return response
        return response

    def reset_people_meta_info_map(self, service_mode=LTAbstract.SERVICE):
        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: None,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: self._reset_people_meta_info_map,
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for reset_people_meta_info_map" % (service_mode)
            return response
        else:
            feedback, result = fct()
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during reset_people_meta_info_map "
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success reset_people_meta_info_map "
                response.payload = result
                return response
        return response

    def wait_for_door_to_open(self, check_freq=None, min_dist=None, service_mode=LTAbstract.SERVICE):
        if min_dist is None:
            min_dist = self.OPEN_DOOR_MIN_DISTANCE
        if check_freq is None:
            check_freq = self.CHECK_DISTANCE_FREQUENCY

        response = LTServiceResponse()

        # Check different service mode
        switcher = {
            LTAbstract.ACTION: None,
            LTAbstract.BUS: None,
            LTAbstract.SERVICE: self._wait_for_door_to_open,
        }

        fct = switcher[service_mode]

        # if service mode not available return an Failure
        if fct is None:
            response.status = LTServiceResponse.FAILURE_STATUS
            response.msg = " is not available for wait_for_door_to_open" % (service_mode)
            return response
        else:
            feedback, result = fct(check_freq, min_dist)
            response.process_state(feedback)

            if response.status == LTServiceResponse.FAILURE_STATUS:
                response.msg = " Failure during wait_for_door_to_open check_freq:[%s], min_dist[%s] " % (
                check_freq, min_dist)
                return response
            else:
                # FIXME to be completed with all ACTION status in GoalStatus
                response.msg = " Operation success wait_for_door_to_open check_freq:[%s], min_dist[%s] " % (
                check_freq, min_dist)
                response.payload = result
                return response
        return response

    #######################################
    # PERCEPTION ACTION
    ######################################

    def _get_object_in_front_robot(self, labels, move_head, timeout):
        try:
            goalObjDetection = ObjectDetectionGoal()
            goalObjDetection.labels = labels
            goalObjDetection.moveHead = move_head

            rospy.loginfo("### OBJECT DETECTION MNG GET OBJECT ACTION PENDING : %s",
                          str(goalObjDetection).replace('\n', ', '))

            # send the current goal to the action server
            self._actionObjectDetectionMng_server.send_goal(goalObjDetection)
            # wait action server result
            finished_before_timeout = self._actionObjectDetectionMng_server.wait_for_result(
                rospy.Duration.from_sec(timeout))
            state = self._actionObjectDetectionMng_server.get_state()
            result = self._actionObjectDetectionMng_server.get_result()
            rospy.loginfo("###### OBJECT DETECTION MNG GET OBJECT ACTION END , State: %s", str(state))
            # if timeout cancel all goals on the action server
            if finished_before_timeout:
                self._actionObjectDetectionMng_server.cancel_all_goals()
            # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
            return state, result
        except Exception as e:
            rospy.logwarn("###### OBJECT DETECTION MNG ACTION FAILURE , State: %s", str(e))
        return GoalStatus.ABORTED, None

    def _learn_people_meta_from_img_topic(self, name, timeout):
        """ Appel de l'apprentissage des attributs d'une personne """
        goalLearnPeople = LearnPeopleFromImgGoal(name=name)
        state, result = self._learn_people_meta(goalLearnPeople, timeout)
        return state, result

    def _learn_people_meta_from_img_path(self, img_path, name, timeout):
        """ Appel de l'apprentissage des attributs d'une personne """
        img_loaded = cv2.imread(img_path)
        msg_img = self._bridge.cv2_to_imgmsg(img_loaded, encoding="bgr8")
        goalLearnPeople = LearnPeopleFromImgGoal(name=name, img=msg_img)
        state, result = self._learn_people_meta(goalLearnPeople, timeout)
        return state, result

    def _learn_people_meta(self, goalLearnPeople, timeout):
        try:
            rospy.loginfo("### LEARN PEOPLE ATTRIBUTES ACTION PENDING")
            # send the current goal to the action server
            self._actionLearnPeopleMeta_server.send_goal(goalLearnPeople)
            # wait action server result
            finished_before_timeout = self._actionLearnPeopleMeta_server.wait_for_result(
                rospy.Duration.from_sec(timeout))
            state = self._actionLearnPeopleMeta_server.get_state()
            result = self._actionLearnPeopleMeta_server.get_result()
            rospy.loginfo("###### LEARN PEOPLE ATTRIBUTES ACTION END , State: %s", str(state))
            # if timeout cancel all goals on the action server
            if finished_before_timeout:
                self._actionLearnPeopleMeta_server.cancel_all_goals()
            # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
            return state, result
        except Exception as e:
            rospy.logwarn("###### LEARN PEOPLE ATTRIBUTES FAILURE , State: %s", str(e))
        return GoalStatus.ABORTED, None

    def _detect_meta_people_from_img_topic(self, timeout):
        goalMetaPeople = ProcessPeopleFromImgGoal()
        state, result = self._detect_meta_people(goalMetaPeople, timeout)
        return state, result

    def _detect_meta_people_from_img_path(self, img_path, timeout):
        img_loaded1 = cv2.imread(img_path)
        msg_im1 = self._bridge.cv2_to_imgmsg(img_loaded1, encoding="bgr8")
        goalMetaPeople = ProcessPeopleFromImgGoal(img=msg_im1)
        state, result = self._detect_meta_people(goalMetaPeople, timeout)
        return state, result

    def _detect_meta_people(self, goalMetaPeople, timeout):
        try:
            rospy.loginfo("### DETECT META PEOPLE ACTION PENDING")
            # send the current goal to the action server
            self._actionMultiplePeopleDetection_server.send_goal(goalMetaPeople)
            # wait action server result
            finished_before_timeout = self._actionMultiplePeopleDetection_server.wait_for_result(
                rospy.Duration.from_sec(timeout))
            state = self._actionMultiplePeopleDetection_server.get_state()
            result = self._actionMultiplePeopleDetection_server.get_result()
            rospy.loginfo("###### DETECT META PEOPLE ACTION END , State: %s", str(state))
            # if timeout cancel all goals on the action server
            if finished_before_timeout:
                self._actionMultiplePeopleDetection_server.cancel_all_goals()
            # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
            return state, result
        except Exception as e:
            rospy.logwarn("###### DETECT META PEOPLE FAILURE , State: %s", str(e))
        return GoalStatus.ABORTED, None

    def _get_people_name_from_img_topic(self, timeout):
        goalPeopleName = GetPeopleNameFromImgGoal()
        state, result = self._get_people_name(goalPeopleName, timeout)
        return state, result

    def _get_people_name_from_img_path(self, img_path, timeout):
        img_loaded = cv2.imread(img_path)
        img_msg = self._bridge.cv2_to_imgmsg(img_loaded, encoding="bgr8")
        goalPeopleName = ProcessPeopleFromImgGoal(img=img_msg)
        state, result = self._get_people_name(goalPeopleName, timeout)
        return state, result

    def _get_people_name(self, goalPeopleName, timeout):
        try:
            rospy.loginfo("### GET PEOPLE NAME ACTION PENDING")
            # send the current goal to the action server
            self._actionGetPeopleName_server.send_goal(goalPeopleName)
            # wait action server result
            finished_before_timeout = self._actionGetPeopleName_server.wait_for_result(rospy.Duration.from_sec(timeout))
            state = self._actionGetPeopleName_server.get_state()
            result = self._actionGetPeopleName_server.get_result()
            rospy.loginfo("###### GET PEOPLE NAME ACTION END , State: %s", str(state))
            # if timeout cancel all goals on the action server
            if finished_before_timeout:
                self._actionGetPeopleName_server.cancel_all_goals()
            # return both state : action state, success:3, failure:4, timeout:1 and result (information send back naoqi)
            return state, result
        except Exception as e:
            rospy.logwarn("###### GET PEOPLE NAME FAILURE , State: %s", str(e))
        except Exception as e:
            rospy.logerr("Service min_front_value_srv could not process request: {error}".format(error=e))
            return GoalStatus.ABORTED, None

        return GoalStatus.ABORTED, None

    def _reset_people_meta_info_map(self):
        try:
            result = self._resetPeopleMetaInfoMapSP()
            return GoalStatus.SUCCEEDED, result
        except rospy.ServiceException as e:
            rospy.logerr("Service reset_people_meta_info_map_srv could not process request: {error}".format(error=e))
            return GoalStatus.ABORTED, None
        except Exception as e:
            rospy.logerr("Service min_front_value_srv could not process request: {error}".format(error=e))
            return GoalStatus.ABORTED, None

    def _wait_for_door_to_open(self, check_freq, min_dist):
        """
        Check the min front value at a given frequency.

        :param check_freq: Frequency at which to check door opening
        :type check_freq: float
        :return: True if detected door opening without error, False otherwise
        """
        ##
        try:
            rate = rospy.Rate(check_freq)
            current_distance = 0.0
            while not rospy.is_shutdown() and current_distance < min_dist:
                current_distance = self._minFrontValueSP().value
                rate.sleep()
            rospy.loginfo("************ DOOR IS OPENED ! ****************")
            return GoalStatus.SUCCEEDED, True
        except rospy.ServiceException as e:
            rospy.logerr("Service min_front_value_srv could not process request: {error}".format(error=e))
            return GoalStatus.ABORTED, None
        except Exception as e:
            rospy.logerr("Service min_front_value_srv could not process request: {error}".format(error=e))
            return GoalStatus.ABORTED, None
