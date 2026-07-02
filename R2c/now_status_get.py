#!/usr/bin/env python3
import math
import struct
import time
import rospy
import serial
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

"""
========================================
  R1 Control Pipeline
  Mode:      test
  Side:      red
  Scene:     1
========================================
[Test] Scene 1 (red):
  Start=(1, 5), Target=(0, 0)
  Grid:
[[3 2 1]
 [2 0 1]
 [1 2 0]
 [2 0 0]]
[Pipeline] Grid: R1=3, R2=4, Fake=(0, 1)
  R1: [(2, 1), (2, 2), (0, 3)]
  R2: [(1, 1), (0, 2), (1, 3), (0, 4)]
  Side=red, Start=(1, 5), Target=(2, 0)
  Base height map:
[[0 0 0]
 [1 2 1]
 [2 3 2]
 [3 2 1]
 [2 1 2]
 [0 0 0]]
  Working map (with KFS):
[[0.  0.  0. ]
 [4.  2.1 1.2]
 [2.1 3.  2.2]
 [3.2 2.1 1. ]
 [2.1 1.  2. ]
 [0.  0.  0. ]]

[Pipeline] A* 规划成功! 路径长度=7, 代价=15.5
  Path: [(1, 5), (2, 5), (2, 4), (2, 3), (2, 2), (2, 1), (2, 0)]

[Pipeline] /path/commands:
  move_right+face_up+move_up_2+face_up+move_down_1+face_left+take_front_r2_up200+face_up+move_up_1+face_up+move_down_1+face_left+take_front_r2_up200+face_up+move_down_1
  Binary (15 bytes): 0xB2 0xC1 0xA3 0xC1 0xA2 0xC2 0xD1 0xC1 0xA1 0xC1 0xA2 0xC2 0xD1 0xC1 0xA2
"""
class Config:
    #共有参数
    SERIAL_PORT = "/dev/base"  # 端口
    BAUD_RATE = 115200  # 波特率
    SERIAL_RETRY_INTERVAL = 1.0  # 串口重连间隔(s)

    # 车体中心位置  串口参数
    POS_SEND_BYTES = 14  # 总字节数
    SEND_HEADER = 0x55  # 帧头
    SEND_TAIL = 0xaa  # 帧尾
    # 偏置
    OFFSET_IMU = [0.011, -0.02329, 0.04412]
    LIDAR_XOFFSET = 0.0  # 雷达相对于机器人中心的X轴偏移
    LIDAR_YOFFSET = -0.27117 # 雷达相对于机器人中心的Y轴偏移


    # R2 Area 2 路径规划 发送
    # SERIAL_AREA2_PATH_PORT  = "/dev/base"
    # BAUD_RATE = 115200  # 波特率
    # PATH_SEND_BYTES = 25  # 总字节数
    # SEND_HEADER = 0x11  # 帧头  -> 不仅仅作为帧头 ， 还作为red/blue场地切换的标志位 （默认red）
    PATH_SEND_TAIL = 0x99  # 帧尾
    BTYE_DATA = [0x11, 0xB2, 0xC1, 0xA3, 0xC1, 0xA2, 0xC2, 0xD1, 0xC1, 0xA1, 0xC1, 0xA2, 0xC2, 0xD1, 0xC1, 0xA2]
    TAR_ARR_L = 25
    #暂时使用这个来作为行驶的路线(二进制数组)  0x11 ->  red 场景

    #到达这个位置时候触发二进制数据的发送
    # x, y, theta ; 这些都是相对于 Fast-livo初定位的zero_point
    # red:   x 水平向右为正 , y 竖直向上为正 
    # blue:  x 水平向左为正 , y 竖直向下为正
    # theta: 以x轴正方向为零角度线 , 逆时针为正
    tar_pos = [2.433, 2.185, math.pi / 2]   # pi/2  ->  yaw角朝前
    threshold_x = 0.03;  #3cm
    threshold_y = 0.03;  #3cm
    threshold_theta = 0.02;  # 3.6/pi rad




class R2:
    def __init__(self):
        # 常规状态
        self.origin_pose = None
        self.current_pose = [0.0, 0.0, 0.0, 0.0]  # 当前位置x,y,z,yaw(cm,cm,cm,rad)
        self.last_serial_retry = 0.0
        # 串口初始化
        self.serial_port = None
        self.serial_data = bytearray(Config.POS_SEND_BYTES)
        self.serial_init()
        # 话题接收
        rospy.Subscriber("/aft_mapped_to_init", Odometry, self.calculate_odom)  # 位姿处理
        #rospy.Subscriber("/Odometry", Odometry, self.calculate_odom)  # 位姿处理
        rospy.on_shutdown(self.shutdown_cleanup)
        # count
        self.satisify_count = 0
        self.tar_count = 4   #目标连续满足帧数
        self.path_sent = False




    def pad_to_length(self, arr, target_len):
        """将数组补齐到目标长度，用原数组最后一个元素填充"""
        if len(arr) >= target_len:
            return arr[:target_len]
        return arr + [arr[-1]] * (target_len - len(arr))

    def calculate_odom(self, msg: Odometry):
        """位置姿态解算"""
        position = msg.pose.pose.position  # 位置
        orientation = msg.pose.pose.orientation  # 姿态
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )  # 朝向：横滚角x 横滚角y 偏航角/航向角z
        # 雷达当前坐标
        dx = Config.OFFSET_IMU[0]
        dy = Config.OFFSET_IMU[1]
        dz = Config.OFFSET_IMU[2]
        # 坐标变换：雷达坐标 → 车体中心坐标
        center_x = position.x - Config.LIDAR_XOFFSET * math.cos(yaw) + Config.LIDAR_YOFFSET * math.sin(yaw) + dx * math.cos(yaw) - dy * math.sin(yaw)
        center_y = position.y - Config.LIDAR_XOFFSET * math.sin(yaw) - Config.LIDAR_YOFFSET * math.cos(yaw) + dx * math.sin(yaw) + dy * math.cos(yaw)
        center_z = position.z + dz
        if self.origin_pose is None:
            self.origin_pose = [center_x, center_y, center_z, yaw]
            rospy.loginfo(
                f"已设置定位原点: ({center_x:.5f}, {center_y:.5f}, {center_z:.5f}, {yaw:.5f})"
            )

        # 更新目前状态
        origin_x, origin_y, origin_z, origin_yaw = self.origin_pose

        curr_x = (center_x - origin_x) * 100
        curr_y = (center_y - origin_y) * 100
        curr_z = (center_z - origin_z) * 100
        curr_yaw = math.atan2(math.sin(yaw - origin_yaw), math.cos(yaw - origin_yaw))
        self.current_pose = [curr_x, curr_y, curr_z, curr_yaw]

        rospy.logwarn(f"当前坐标与朝向: ({curr_x:.5f}, {curr_y:.5f}, {curr_z:.5f}, {curr_yaw:.5f})")
        

        dx = abs(curr_x - Config.tar_pos[0] * 100)
        dy = abs(curr_y - Config.tar_pos[1] * 100)
        dtheta = abs(math.atan2(math.sin(curr_yaw - Config.tar_pos[2]),
                                math.cos(curr_yaw - Config.tar_pos[2])))

        if dx < Config.threshold_x * 100 and dy < Config.threshold_y * 100 and dtheta < Config.threshold_theta:
            self.satisify_count += 1
        else:
            self.satisify_count = 0
            self.path_sent = False

        if self.satisify_count > self.tar_count and not self.path_sent:
            for _ in range(10):
                self.serial_send_path()
                time.sleep(0.05)
            self.satisify_count = 0
            self.path_sent = True
        else:
            self.serial_send_pos(curr_x, curr_y, curr_yaw)
            


    # 负责  路径环 的发送
    def serial_send_path(self):
        """串口发送路径数据：帧头(场景标志位) + path + 帧尾"""
        if not self.serial_port or not self.serial_port.is_open:
            self.serial_reconnect()
            return
        try:
            arr = self.pad_to_length(Config.BTYE_DATA, Config.TAR_ARR_L)
            arr[-1] = Config.PATH_SEND_TAIL
            # 发送数据
            self.serial_port.write(bytes(arr))
            rospy.loginfo(f"path串口发送完整帧(十六进制): {bytes(arr).hex()}")
        except Exception as e:
            rospy.logerr(f"串口发送失败: {e}")
            self.close_serial()


    # 负责 位置 与 位姿 环的发送
    def serial_send_pos(self, curr_x, curr_y, curr_yaw):
        """串口发送定位数据：帧头 + x(cm) + y(cm) + yaw(rad) + 帧尾"""
        if not self.serial_port or not self.serial_port.is_open:
            self.serial_reconnect()
            return
        try:
            self.serial_data[0] = Config.SEND_HEADER
            self.serial_data[-1] = Config.SEND_TAIL
            pack_data = struct.pack('<3f', curr_x, curr_y, curr_yaw)
            self.serial_data[1:-1] = pack_data
            # 发送数据
            self.serial_port.write(self.serial_data)
            # 调试打印
            rospy.loginfo(f"pos串口发送pos帧: {curr_x:.5f}cm; {curr_y:.5f}cm; {curr_yaw:.5f}rad")
            rospy.loginfo(f"pos串口发送完整帧(十六进制): {self.serial_data.hex()}")
        except Exception as e:
            rospy.logerr(f"pos串口发送失败: {e}")
            self.close_serial()


    def serial_init(self):
        """初始化串口"""
        try:
            self.serial_port = serial.Serial(
                Config.SERIAL_PORT, Config.BAUD_RATE, timeout=1.0)
            rospy.loginfo("串口初始化成功!")
        except serial.SerialException as e:
            self.serial_port = None
            rospy.logerr(f"串口打开失败: {e}")

    def serial_reconnect(self):
        """串口断开时按间隔尝试重连，避免每帧刷屏。"""
        now = time.time()
        if now - self.last_serial_retry < Config.SERIAL_RETRY_INTERVAL:
            return
        self.last_serial_retry = now
        rospy.logwarn("串口未连接，尝试重新打开...")
        self.serial_init()

    def close_serial(self):
        """关闭串口资源。"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            rospy.loginfo("串口已关闭")

    def shutdown_cleanup(self):
        """节点关闭时释放资源。"""
        self.close_serial()

if __name__ == '__main__':
    try:
        rospy.init_node("R1", anonymous=True)
        rc = R2()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("节点被强制中断")
    except Exception as e:
        rospy.logerr(f"节点运行异常: {e}")
    finally:
        if 'rc' in locals():
            rc.shutdown_cleanup()
