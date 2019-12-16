#!/usr/bin/env python
# -*- coding=utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    '''
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    '''
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


# 初始化
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_kr3', anonymous=True)

# 实例化
robot = moveit_commander.RobotCommander()
move_group = moveit_commander.MoveGroupCommander('arm')
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

# 等待RVIZ启动
print('============ Waiting for RVIZ...')
rospy.sleep(1)
print('============ Starting tutorial')

# 打印信息
print '============ Reference frame: %s' % move_group.get_planning_frame()
print '============ Reference frame eef: %s' % move_group.get_end_effector_link()
print '============ Robot Groups:'
print robot.get_group_names()
print '============ Printing robot state'
print robot.get_current_state()

# 运动规划
print '>>>>>>>>>>>> Printing current pose'
print move_group.get_current_pose().pose

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.4
pose_goal.position.y = 0.1
pose_goal.position.z = 0.4

move_group.set_pose_target(pose_goal)
plan = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()

print '<<<<<<<<<<< Printing current pose'
print move_group.get_current_pose().pose

waypoints = []
scale = 1.0

wpose = move_group.get_current_pose().pose
wpose.position.z -= scale * 0.1  # First move up (z)
wpose.position.y += scale * 0.2  # and sideways (y)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.y -= scale * 0.1  # Third move sideways (y)
waypoints.append(copy.deepcopy(wpose))

(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
display_trajectory_publisher.publish(display_trajectory)

# move_group.execute(plan, wait=True)
