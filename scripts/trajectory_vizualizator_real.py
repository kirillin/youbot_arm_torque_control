#!/usr/bin/python

import rospy
import string
import threading
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from brics_actuator.msg import JointPositions, JointValue, JointVelocities

def jointTrajectoryCallback(msg):
    global joint_trajectory
    lock.acquire()
    joint_trajectory = msg
    lock.release()




def showTrajectoryCallback(req):
    jp_msg = JointPositions()
    for j in range(5):
        jp_msg.positions.append(JointValue())
        jp_msg.positions[j].joint_uri = 'arm_joint_' + str(j + 1)
        jp_msg.positions[j].unit = b'rad'



    lock.acquire()
    if len(joint_trajectory.points) == 0:
        lock.release()
        return TriggerResponse(False, 'No one or empty trajectory have been received!')
    while True:
        for i, pose in enumerate(joint_trajectory.points):
            #q0 = [3.04, 1.18, -2.63, 1.75, 2.89]
            for j in range(5):
                jp_msg.positions[j].value = pose.positions[j]
            js_pub.publish(jp_msg)
            if i != len(joint_trajectory.points) - 1:
                dt = joint_trajectory.points[i+1].time_from_start.to_sec() - pose.time_from_start.to_sec()
                rospy.sleep(dt)
            else:
                break

        j = len(joint_trajectory.points)-1
        pose = joint_trajectory.points
        while j > 0:
            for i in range(5):
                jp_msg.positions[i].value = pose[j].positions[i]
            js_pub.publish(jp_msg)
            j = j - 1
            rospy.sleep(0.1)
    lock.release()
    return TriggerResponse(True, 'Successfully done!')


def showVelTrajectoryCallback(req):
    jv_msg = JointVelocities()
    for i in range(5):
        jv_msg.velocities.append(JointValue())
        jv_msg.velocities[i].joint_uri = 'arm_joint_' + str(i + 1)
        jv_msg.velocities[i].unit = 's^-1 rad'

    lock.acquire()
    if len(joint_trajectory.points) == 0:
        lock.release()
        return TriggerResponse(False, 'No one or empty trajectory have been received!')
    while True:
        for i, pose in enumerate(joint_trajectory.points):
            for j in range(5):
                jv_msg.velocities[j].value = pose.velocities[j]
            jv_pub.publish(jv_msg)

            if i != len(joint_trajectory.points) - 1:
                dt = joint_trajectory.points[i + 1].time_from_start.to_sec() - pose.time_from_start.to_sec()
                rospy.sleep(dt)
            else:
                break
    lock.release()
    return TriggerResponse(True, 'Successfully done!')


if __name__=="__main__":
    rospy.init_node("trajectory_vizualizator__real_node")

    lock = threading.Lock()
    joint_trajectory = JointTrajectory()
    joint_trajectory.points = []

    jt_sub = rospy.Subscriber("joint_trajectory", JointTrajectory, jointTrajectoryCallback)
    js_pub = rospy.Publisher("arm_1/arm_controller/position_command", JointPositions, queue_size=2, latch=True)
    jv_pub = rospy.Publisher("arm_1/arm_controller/velocity_command", JointVelocities, queue_size=2, latch=True)
    #js_pub = rospy.Publisher("/", JointState, queue_size=1, latch=True)
    show_poses = rospy.Service('show_trajectory', Trigger, showTrajectoryCallback)
    show_poses_vel = rospy.Service('show_trajectory_vel', Trigger, showVelTrajectoryCallback)

    rospy.spin()

