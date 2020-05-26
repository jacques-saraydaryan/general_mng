#!/usr/bin/env python

import rospy

from meta_lib.LTPerception import LTPerception
from meta_lib.LTNavigation import LTNavigation

from meta_behaviour.LTHighBehaviour import LTHighBehaviour

class Test:

    def __init__(self):
        rospy.init_node("test_pointing")

        self._perception = LTPerception()

        self._navigation = LTNavigation()

        self._behaviour = LTHighBehaviour()

        room = "Kitchen"
    
        response = self._perception.get_object_in_room(room)
        objects_list = response.payload
        rospy.logwarn("OBJECTS IN ROOM %s",str(objects_list))


        object_to_point = "object_windex2_TF"
        self._behaviour.point_an_object(object_to_point)

if __name__ == "__main__":

    a=Test()


