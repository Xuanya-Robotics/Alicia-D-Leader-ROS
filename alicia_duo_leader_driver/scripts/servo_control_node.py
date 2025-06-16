#!/usr/bin/env python3
# coding=utf-8

"""
标准机械臂控制节点
接收7自由度(6关节+夹爪)的弧度控制命令，转换为硬件协议格式并发送。
"""

import rospy
import math
import numpy as np
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension,Float32, Bool
from alicia_duo_leader_driver.msg import ArmJointState

# 常量定义
RAD_TO_DEG = 180.0 / math.pi  # 弧度转角度系数
DEG_TO_RAD = math.pi / 180.0  # 角度转弧度系数

# 协议常量
FRAME_HEADER = 0xAA
FRAME_FOOTER = 0xFF
CMD_SERVO_CONTROL = 0x04
CMD_EXTENDED_CONTROL = 0x06
CMD_GRIPPER_CONTROL = 0x02
CMD_ZERO_CAL = 0x03
CMD_DEMO_CONTROL = 0x13

class ServoControlNode:
    def __init__(self):
        """初始化节点"""
        rospy.init_node('servo_control_node')
        
        # 配置参数
        self.servo_count = rospy.get_param('~servo_count', 9)  # 总舵机数量
        self.joint_count = 6  # 有效关节数
        self.debug_mode = rospy.get_param('~debug_mode', False)
        
        # 设置ROS通信
        self._setup_communication()
        
        rospy.loginfo("机械臂控制节点已初始化，使用标准弧度接口")
        

    def _setup_communication(self):
        """设置订阅者和发布者"""
        # 发布者 - 发送硬件协议数据到串口节点
        self.serial_pub = rospy.Publisher('/send_serial_data', UInt8MultiArray, queue_size=10)
        self.zero_calibration = rospy.Subscriber('/zero_calibrate', Bool, self.zero_calib_callback, queue_size=10)


    def calculate_checksum(self, frame):
        """计算校验和"""
        # 计算索引3到倒数第2个元素的总和，对2取模
        checksum = sum(frame[3:-2]) % 2
        return checksum
    
    
    def rad_to_hardware_value(self, angle_rad):
        """将弧度转换为硬件值(0-4095)"""
        # 先转换为角度
        angle_deg = angle_rad * RAD_TO_DEG
        
        # 范围检查
        if angle_deg < -180.0 or angle_deg > 180.0:
            rospy.logwarn("角度值超出范围: %.2f度，会被截断", angle_deg)
            angle_deg = max(-180.0, min(180.0, angle_deg))
        
        # 转换公式: -180° → 0, 0° → 1024, +180° → 2048
        # 实际映射到0-4095
        value = int((angle_deg + 180.0) / 360.0 * 4096)
        
        # 范围限制
        return max(0, min(4095, value))
    
    
    def _print_debug_info(self):
        """打印调试信息"""
        # 打印舵机帧
        servo_hex = " ".join([f"{b:02X}" for b in self.servo_angle_frame])
        rospy.logdebug("舵机帧: %s", servo_hex)
        
        # 打印夹爪帧
        gripper_hex = " ".join([f"{b:02X}" for b in self.gripper_frame])
        rospy.logdebug("夹爪帧: %s", gripper_hex)

    def zero_calib_callback(self, msg):
        """处理零点校准命令"""
        try:
            if self.debug_mode:
                rospy.loginfo("接收到零点校准命令: %d", msg.data)

            if msg.data:
                # 发送零点校准命令
                zero_calib_msg = self.frame_ge(CMD_ZERO_CAL)
                rospy.loginfo("开始零点校准")
            # else:
                
            # zero_calib_msg =self.frame_ge(CMD_ZERO_CAL)
            self.serial_pub.publish(zero_calib_msg)

            
            rospy.loginfo("零点校准完成")
                
        except Exception as e:
            rospy.logerr("处理零点校准命令出错: %s", str(e))
            
    def frame_ge(self, control_cmd, control_data=0x00, check=True):
        frame_d = [0] * 6
        frame_d[0] = FRAME_HEADER
        frame_d[1] = control_cmd
        frame_d[2] = 0x01  # 数据长度
        frame_d[3] = control_data  # 数据内容
        if check:
            frame_d[-2] = self.calculate_checksum(frame_d)
        else:
            frame_d[-2] = 0x00
        frame_d[-1] = FRAME_FOOTER
            
        binary_data = bytearray()
        for byte in frame_d:
            binary_data.append(byte)
            # 创建并发送零点校准消息
        frame_d_msg = UInt8MultiArray()
        frame_d_msg.data = binary_data
        return frame_d_msg
      
          
def main():
    """主函数"""
    try:
        node = ServoControlNode()
        rospy.loginfo("机械臂控制节点已启动")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("机械臂控制节点异常: %s", str(e))

if __name__ == '__main__':
    main()