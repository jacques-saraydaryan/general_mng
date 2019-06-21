import rospy
from pepper_pose_for_nav.srv import MoveHeadAtPosition

from rospy.exceptions import ROSException, ROSInterruptException


class AbstractScenarioService:
    HEAD_PITCH_FOR_SPEECH_POSE = 0.20
    HEAD_PITCH_FOR_NAV_POSE = 0.5
    HEAD_YAW_CENTER = 0.0

    _enableMoveHeadPoseService = True

    def __init__(self):
        pass

    def configure_intern(self):
        if self._enableMoveHeadPoseService:
            rospy.loginfo("Connecting to the move_head_pose_srv service...")
            self._moveHeadPoseSP = rospy.ServiceProxy('move_head_pose_srv', MoveHeadAtPosition)
            try:
                move_head_pose_srv_is_up = rospy.wait_for_service('move_head_pose_srv', timeout=10.0)
                rospy.loginfo("Connected to the move_head_pose_srv service.")
            except (ROSException, ROSInterruptException) as e:
                rospy.logwarn("Unable to connect to the move_head_pose_srv service.")

    def moveheadPose(self, pitch_value, yaw_value, track):
        try:
            return self._moveHeadPoseSP(pitch_value, yaw_value, track)
        except rospy.ServiceException as e:
            rospy.logerr("Service move_head_pose_srv could not process request: {error}".format(error=e))
