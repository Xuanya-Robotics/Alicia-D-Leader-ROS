#!/usr/bin/env python3
# coding=utf-8

"""
ROS节点示例：订阅并打印来自 /arm_joint_state 话题的机械臂状态数据。
"""

import rospy
from alicia_duo_leader_driver.msg import ArmJointState
import math

# 常量，用于将弧度转换为角度以便于阅读
RAD_TO_DEG = 180.0 / math.pi

def arm_state_callback(msg):
    """
    处理接收到的 ArmJointState 消息的回调函数。
    将关节和夹爪的角度从弧度转换为度，并打印所有信息。
    """
    rospy.loginfo("--- Received Arm Joint State ---")
    rospy.loginfo(f"  Timestamp: {msg.header.stamp}")
    rospy.loginfo(f"  Joint 1: {msg.joint1 * RAD_TO_DEG:.2f} deg ({msg.joint1:.4f} rad)")
    rospy.loginfo(f"  Joint 2: {msg.joint2 * RAD_TO_DEG:.2f} deg ({msg.joint2:.4f} rad)")
    rospy.loginfo(f"  Joint 3: {msg.joint3 * RAD_TO_DEG:.2f} deg ({msg.joint3:.4f} rad)")
    rospy.loginfo(f"  Joint 4: {msg.joint4 * RAD_TO_DEG:.2f} deg ({msg.joint4:.4f} rad)")
    rospy.loginfo(f"  Joint 5: {msg.joint5 * RAD_TO_DEG:.2f} deg ({msg.joint5:.4f} rad)")
    rospy.loginfo(f"  Joint 6: {msg.joint6 * RAD_TO_DEG:.2f} deg ({msg.joint6:.4f} rad)")
    rospy.loginfo(f"  Gripper: {msg.gripper * RAD_TO_DEG:.2f} deg ({msg.gripper:.4f} rad)")
    rospy.loginfo(f"  Button 1: {msg.but1}")
    rospy.loginfo(f"  Button 2: {msg.but2}")
    rospy.loginfo("---------------------------------")

def main():
    """
    初始化ROS节点并订阅 /arm_joint_state 话题。
    """
    try:
        # 初始化ROS节点，anonymous=True确保节点名称唯一
        rospy.init_node('arm_state_reader_demo', anonymous=True)
        rospy.loginfo("Arm State Reader Demo Node Started.")

        # 创建一个订阅者，监听 /arm_joint_state 话题，
        # 当收到消息时调用 arm_state_callback 函数
        rospy.Subscriber('/arm_joint_state', ArmJointState, arm_state_callback)

        rospy.loginfo("Subscribed to /arm_joint_state. Waiting for messages...")

        # rospy.spin() 使脚本保持运行，直到节点被关闭（例如，通过Ctrl+C）
        rospy.spin()

    except rospy.ROSInterruptException:
        # 捕获Ctrl+C中断信号
        rospy.loginfo("Arm State Reader Demo Node Shutting Down.")
    except Exception as e:
        # 捕获其他可能的异常
        rospy.logerr(f"An error occurred in the main loop: {e}")

if __name__ == '__main__':
    # 当脚本作为主程序执行时，调用main函数
    main()