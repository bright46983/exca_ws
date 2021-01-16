#!/usr/bin/env python

import rospy
import smach
from exca_autodig.srv import ExcaGoal
from exca_autodig.srv import ExcaGoalRequest
from exca_autodig.srv import ExcaGoalResponse

class GotoPoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reach_point','abort'])
        rospy.wait_for_service('gotopoint')
        penetrate_srv = rospy.ServiceProxy('gotopoint',ExcaGoal)

    def execute(self, userdata):
        rospy.loginfo('Executing state GotoPoint')
        goal_reached = penetrate_srv(input?)
        if goal_reached:
            rospy.loginfo('Finish Penetrate')
            return 'reach_point'
        else:
            rospy.loginfo('Fail to Penetrate')
            return 'abort'

class Penetrate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reach_depth','abort'])
        rospy.wait_for_service('penetrate')
        penetrate_srv = rospy.ServiceProxy('penetrate',ExcaGoal)

    def execute(self, userdata):
        rospy.loginfo('Executing state Penetrate')

        goal_reached = penetrate_srv(input?)
        if goal_reached:
            rospy.loginfo('Finish Penetrate')
            return 'reach_depth'
        else:
            rospy.loginfo('Fail to Penetrate')
            return 'abort'


class Drag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reach_length','abort'])
        rospy.wait_for_service('drag')
        penetrate_srv = rospy.ServiceProxy('drag',ExcaGoal)

    def execute(self, userdata):
        rospy.loginfo('Executing state Penetrate')

        goal_reached = penetrate_srv(input?)
        if goal_reached:
            rospy.loginfo('Finish Drag')
            return 'reach_depth'
        else:
            rospy.loginfo('Fail to Drag')
            return 'abort'
        
class Recovery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome4'])
        rospy.wait_for_service('drag')
        penetrate_srv = rospy.ServiceProxy('drag',ExcaGoal)

    def execute(self, userdata):
        rospy.loginfo('Executing state Penetrate')

        goal_reached = penetrate_srv(input?)
        if goal_reached:
            rospy.loginfo('Finish Drag')
            return 'reach_depth'
        else:
            rospy.loginfo('Fail to Drag')
            return 'abort'


# main
def main():
    rospy.init_node('exca_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Penetrate', Penetrate(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'outcome4'})
        smach.StateMachine.add('Drag', Drag(), 
                               transitions={'outcome2':'FOO'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()