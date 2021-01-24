#!/usr/bin/env python

import rospy
import smach
from exca_autodig.srv import ExcaGoal ,ExcaGoalRequest, ExcaGoalResponse
from exca_autodig.srv import TrenchGoal, TrenchGoalRequest, TrenchGoalResponse



class FindPoa(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found_poa'],input_keys=['findpoa_goal'],output_keys=['poa'])
        rospy.wait_for_service('find_poa')
        self.poa_srv = rospy.ServiceProxy('find_poa',TrenchGoal)

    def execute(self, userdata):
        rospy.loginfo('Executing state FindPoa')
        poa = self.poa_srv(userdata.findpoa_goal[0],userdata.findpoa_goal[1],userdata.findpoa_goal[2],userdata.findpoa_goal[3],userdata.findpoa_goal[4])
        userdata.poa = [[poa.poa_x,poa.poa_y,poa.poa_z],poa.drag_length,poa.penetrate_depth]
        rospy.loginfo("POA: %f,%f,%f   Length:%f   Depth:%f",poa.poa_x,poa.poa_y,poa.poa_z,poa.drag_length,poa.penetrate_depth)
        return 'found_poa'

class GotoPoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reach_point','abort'],input_keys=['gotopoint_goal'])
        rospy.wait_for_service('gotopoint')
        self.goto_srv = rospy.ServiceProxy('gotopoint',ExcaGoal)

    def execute(self, userdata):
        rospy.loginfo('Executing state GotoPoint')
        goal_reached = self.goto_srv(userdata.gotopoint_goal[0],userdata.gotopoint_goal[1],userdata.gotopoint_goal[2],0)
        if goal_reached.finish:
            rospy.loginfo('Finish GotoPoint')
            return 'reach_point'
        else:
            rospy.loginfo('Fail to GotoPoint')
            return 'abort'

class Penetrate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reach_depth','abort'],input_keys=['penetrate_goal'])
        rospy.wait_for_service('penetrate')
        self.penetrate_srv = rospy.ServiceProxy('penetrate',ExcaGoal)

    def execute(self, userdata):
        rospy.loginfo('Executing state Penetrate')

        goal_reached = self.penetrate_srv(userdata.penetrate_goal[0],userdata.penetrate_goal[1],userdata.penetrate_goal[2],0)
        if goal_reached.finish:
            rospy.loginfo('Finish Penetrate')
            return 'reach_depth'
        else:
            rospy.loginfo('Fail to Penetrate')
            return 'abort'


class Drag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reach_length','abort'],input_keys=['drag_goal'])
        rospy.wait_for_service('drag')
        self.drag_srv = rospy.ServiceProxy('drag',ExcaGoal)

    def execute(self, userdata):
        rospy.loginfo('Executing state Drag')

        goal_reached = self.drag_srv(userdata.drag_goal[0],userdata.drag_goal[1],userdata.drag_goal[2],0)
        if goal_reached.finish:
            rospy.loginfo('Finish Drag')
            return 'reach_length'
        else:
            rospy.loginfo('Fail to Drag')
            return 'abort'

class Closing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['closed','abort'])
        rospy.wait_for_service('close')
        self.close_srv = rospy.ServiceProxy('close',ExcaGoal)

    def execute(self, userdata):
        rospy.loginfo('Executing state Closing')
        exca_goal = [[0.45,0,0],0.02,0.30]
        goal_reached = self.close_srv(exca_goal[0],exca_goal[1],exca_goal[2],0)
        if goal_reached.finish:
            rospy.loginfo('Finish Closing')
            return 'closed'
        else:
            rospy.loginfo('Fail to Close')
            return 'abort'

class Recovery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finish_recovery'])
        rospy.wait_for_service('recovery')
        self.recovery_srv = rospy.ServiceProxy('recovery',ExcaGoal)

    def execute(self, userdata):
        rospy.loginfo('Executing state Recovery')
        exca_goal = [[0.45,0,0],-0.02,0.30]
        goal_reached= self.recovery_srv(exca_goal[0],exca_goal[1],exca_goal[2],0)
        if goal_reached.finish:
            rospy.loginfo('Finish Recovery')
            return 'finish_recovery'
        


# main
def main():
    rospy.init_node('exca_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.trench_goal = [0.5,0,1,0.5,3]

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FindPoa', FindPoa(), 
                               transitions={'found_poa':'GotoPoint'},
                                remapping={'findpoa_goal':'trench_goal',
                                            'poa' : 'exca_goal'})
        smach.StateMachine.add('GotoPoint', GotoPoint(), 
                               transitions={'reach_point':'Penetrate', 
                                            'abort':'Recovery'},
                                remapping={'gotopoint_goal':'exca_goal'})
        smach.StateMachine.add('Penetrate', Penetrate(), 
                               transitions={'reach_depth':'Drag', 
                                            'abort':'Recovery'},
                                remapping={'penetrate_goal':'exca_goal'})
        smach.StateMachine.add('Drag', Drag(), 
                               transitions={'reach_length':'Closing', 
                                            'abort':'Recovery'},
                                remapping={'drag_goal':'exca_goal'})
        smach.StateMachine.add('Closing', Closing(), 
                               transitions={'closed':'Recovery', 
                                            'abort':'Recovery'})
        smach.StateMachine.add('Recovery', Recovery(), 
                               transitions={'finish_recovery':'outcome4'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()