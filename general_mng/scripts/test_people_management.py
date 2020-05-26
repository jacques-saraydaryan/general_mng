#!/usr/bin/env python

import rospy
from meta_lib.LTPerception import LTPerception

class Test:

    def __init__(self):

        rospy.init_node("test_people")

        self._lt_perception = LTPerception()

        response = self._lt_perception.learn_people_meta_from_img_topic(name="Thomas",timeout=10)

        rospy.loginfo("RESPONSE : %s",str(response))


if __name__ == "__main__":
    a = Test()

