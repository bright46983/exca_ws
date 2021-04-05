#!/usr/bin/env python

import rospy
import smach
from exca_autodig.srv import ExcaGoal ,ExcaGoalRequest, ExcaGoalResponse
from exca_autodig.srv import TrenchGoal, TrenchGoalRequest, TrenchGoalResponse
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Empty
import tf

class FindPoa(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found_poa','no_poa'],input_keys=['findpoa_goal'],output_keys=['poa'])
        rospy.wait_for_service('find_poa')
        self.poa_srv = rospy.ServiceProxy('find_poa',TrenchGoal)
        rospy.wait_for_service('/elevation_mapping/disable_updates')
        self.dis_update = rospy.ServiceProxy('/elevation_mapping/disable_updates',Empty)
        

    def execute(self, userdata):
        tff = tf.TransformListener()
        rospy.loginfo('Executing state FindPoa')
        poa_res = self.poa_srv(userdata.findpoa_goal[0],userdata.findpoa_goal[1],userdata.findpoa_goal[2],userdata.findpoa_goal[3],userdata.findpoa_goal[4])
        
        tff.waitForTransform("/base_footprint","/map", rospy.Time(), rospy.Duration(1.0))

        poa_map = PointStamped()
        poa_map.header.frame_id = "map"
        poa_map.header.stamp = rospy.Time.now()
        

        
        if not poa_res.finish:
            poa_map.point.x = userdata.findpoa_goal[0] - userdata.findpoa_goal[2] + 0.05
            poa_map.point.y = userdata.findpoa_goal[1]
            poa_map.point.z = -userdata.findpoa_goal[4] +0.02
            poa = tff.transformPoint("base_footprint", poa_map)
            userdata.poa = [[poa.point.x,poa.point.y,poa.point.z],userdata.findpoa_goal[3],0]
            return 'no_poa'

        poa_map.point.x = poa_res.poa_x
        poa_map.point.y = poa_res.poa_y
        poa_map.point.z = poa_res.poa_z
        poa = tff.transformPoint("base_footprint", poa_map)
        userdata.poa = [[poa.point.x,poa.point.y,poa.point.z],poa_res.drag_length,poa_res.penetrate_depth]
        rospy.loginfo("POA: %f,%f,%f   Length:%f   Depth:%f",poa.point.x,poa.point.y,poa.point.z,poa_res.drag_length,poa_res.penetrate_depth)
        self.dis_update()
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


class Dump(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finish_dump'])
        rospy.wait_for_service('/elevation_mapping/enable_updates')
        self.en_update = rospy.ServiceProxy('/elevation_mapping/enable_updates',Empty)
        rospy.wait_for_service('dump')
        self.dump_srv = rospy.ServiceProxy('dump',ExcaGoal)

    def execute(self, userdata):
        rospy.loginfo('Executing state Dump')
        self.en_update()
        exca_goal = [[0.45,0,0],-0.02,0.30]
        goal_reached= self.dump_srv(exca_goal[0],exca_goal[1],exca_goal[2],0)
        rospy.sleep(20)
        return 'finish_dump'


class Recovery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finish_recovery'])
        rospy.wait_for_service('recovery')
        self.recovery_srv = rospy.ServiceProxy('recovery',ExcaGoal)
        rospy.wait_for_service('/elevation_mapping/enable_updates')
        self.en_update = rospy.ServiceProxy('/elevation_mapping/enable_updates',Empty)

    def execute(self, userdata):
        rospy.loginfo('Executing state Recovery')
        self.en_update()
        exca_goal = [[0.45,0,0],-0.02,0.30]
        goal_reached= self.recovery_srv(exca_goal[0],exca_goal[1],exca_goal[2],0)
        if goal_reached.finish:
            rospy.loginfo('Finish Recovery')
            return 'finish_recovery'
        
    
class Finshing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['trench_complete','abort'],input_keys=['finishing_goal'])
        rospy.wait_for_service('gotopoint')
        self.gotofin_srv = rospy.ServiceProxy('gotopoint',ExcaGoal)
        rospy.wait_for_service('drag')
        self.dragfin_srv = rospy.ServiceProxy('drag',ExcaGoal)
        rospy.wait_for_service('close')
        self.closefin_srv = rospy.ServiceProxy('close',ExcaGoal)
        rospy.wait_for_service('dump')
        self.dumpfin_srv = rospy.ServiceProxy('dump',ExcaGoal)

    def execute(self, userdata):
        rospy.loginfo('Executing state Finishing')
        goal_reached = self.gotofin_srv(userdata.finishing_goal[0],userdata.finishing_goal[1],userdata.finishing_goal[2],0)
        if goal_reached.finish:
            rospy.loginfo('Finish GotoPoint')
            goal_reached = self.dragfin_srv(userdata.finishing_goal[0],userdata.finishing_goal[1],userdata.finishing_goal[2],0)
            if goal_reached.finish:
                rospy.loginfo('Finish Drag')
                exca_goal = [[0.45,0,0],-0.02,0.30]
                goal_reached = self.closefin_srv(exca_goal[0],exca_goal[1],exca_goal[2],0)
                goal_reached= self.dumpfin_srv(exca_goal[0],exca_goal[1],exca_goal[2],0)
                return 'trench_complete'

            else:
                rospy.loginfo('Fail to Drag')
                return 'abort'
            
        else:
            rospy.loginfo('Fail to GotoPoint')
            return 'abort'


# main
def main():
    rospy.init_node('exca_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.trench_goal = [0.5,0,0.2,0.1,0.07]

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FindPoa', FindPoa(), 
                               transitions={'found_poa':'GotoPoint',
                                            'no_poa':'Finishing'},
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
                               transitions={'closed':'Dump', 
                                            'abort':'Recovery'})
        smach.StateMachine.add('Dump', Dump(), 
                               transitions={'finish_dump':'FindPoa'})
        smach.StateMachine.add('Finishing', Finshing(), 
                                transitions={'trench_complete':'outcome4', 
                                            'abort':'Recovery'},
                                remapping={'finishing_goal':'exca_goal'})
        smach.StateMachine.add('Recovery', Recovery(), 
                               transitions={'finish_recovery':'outcome4'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()