#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from common.robotState import RobotState


class A(RobotState):
    def __init__(self):
        RobotState.__init__(self,timeout=1,retry_nb=2,name="A_STATE", outcomes=['outcome1'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        return 'outcome1'



# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome1'
        




def main():
    rospy.init_node('smach_example_state_machine')
    
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    #Create an introspection server to use smatch viewer
    sis = smach_ros.IntrospectionServer('server_name', sm, '/ROBOT_CUP_TEST')
    #Start theintrospection server
    sis.start()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('A', A(), 
                               transitions={'outcome1':'outcome4'})
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'})

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    #Stop the introspection server
    sis.stop()



if __name__ == '__main__':
    main()
