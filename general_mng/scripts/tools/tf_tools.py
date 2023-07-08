# Transform a given input pose from one fixed frame to another
import rospy
from geometry_msgs.msg import Pose, PoseStamped
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def transform_pose(input_pose, from_frame, to_frame):

    listener = tf.TransformListener()

    try:
            now = rospy.Time(0)
            listener.waitForTransform(from_frame, to_frame, now, rospy.Duration(5.0))
            input_pose.header.stamp = now
            output_pose_stamped = listener.transformPose(to_frame, input_pose)
            return output_pose_stamped
    except Exception as err:
            rospy.loginfo("%s",str(err))
            return None