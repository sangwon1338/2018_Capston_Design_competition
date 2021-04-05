#! /usr/bin/env python
#-*-coding:utf-8-*-

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import rospy
from matplotlib import pyplot as plt
import math
import time

hz = 20
dt = 1.0 / hz

class callback():
    # imu data 로깅 함수
    def imu_data(self,msg):
        global yaw, a_x, a_y
        yaw = round(msg.orientation.w)
        a_x = round(msg.linear_acceleration.x, 2)
        a_y = round(msg.linear_acceleration.y, 2)

        # rospy.loginfo('yaw :: {}'.format(a_x))

        return yaw, a_x, a_y

    def sonar_data(self,msg):
        global sonar
        sonar = msg.data
        if sonar > 100:
            sonar = 100
        # rospy.loginfo('sonar :: {}'.format(sonar))
        return sonar

    def frw(self,msg):
        global frw
        frw = msg.data * 2 * 3.141592 / 60
        # rospy.loginfo('front_hall :: {}'.format(frw))
        return frw

    def flw(self,msg):
        global flw
        flw = msg.data * 2 * 3.141592 / 60
        # rospy.loginfo('front_hall :: {}'.format(flw))
        return flw

    def rrw(self,msg):
        global rrw
        rrw = msg.data * 2 * 3.141592 / 60
        # rospy.loginfo('rear_hall :: {}'.format(rear_hall))
        return rrw

    def rlw(self,msg):
        global rlw
        rlw = msg.data * 2 * 3.141592 / 60
        # rospy.loginfo('rear_hall :: {}'.format(rear_hall))
        return rlw

class barc(callback):

    # 서브스크라이버
    def sub(self):
        rospy.init_node('main')

        # topic 선언
        imu = rospy.Subscriber('/imu/data', Imu, self.imu_data)
        sonar = rospy.Subscriber('/dist', Float32, self.sonar_data)
        frw = rospy.Subscriber('/Rps_front_right', Float32, self.frw)
        flw = rospy.Subscriber('/Rps_front_left', Float32, self.flw)
        rrw = rospy.Subscriber('/Rps_rear_right', Float32, self.rrw)
        flw = rospy.Subscriber('/Rps_rear_left', Float32, self.rlw)

        return imu, sonar, frw,flw,rrw,flw

    # 퍼블리셔
    def pub(self):
        rospy.init_node('main')

        # topic 선언
        pub1 = rospy.Publisher('/angle_msg', Int32, queue_size=10)
        pub2 = rospy.Publisher('/motor_speed', Int32, queue_size=10)

        return pub1, pub2


class logic(callback, barc):

    def control_tcs(self):
        global Fw, Rw
        # 목표 슬립율 0.04
        flw = 1
        frw = 1

        Fw = (flw + frw) / 2.0
        Rw = (rlw + rrw) / 2.0
        slip_ratio = Rw / Fw - 1
        return slip_ratio

    def servo_initialize(self):
        '''
        left_max : 157 degree
        neutral : 135 degree
        right_max : 112 degree
        steer angle : 22 degree
        :return:
        '''
        # 각도는 절대각도, 증가하지 않음
        while 1:
            for angle_start in range(112,157):
                # angle_start = input("측정 각도를 입력하시오 : ")
                print(angle_start)
                pub1, pub2 = self.pub()
                pub1.publish(int(angle_start))
                time.sleep(0.05)

    def servo_setting(self, data):
        if data >= 157:
            data = 157
        elif data <= 113:
            data = 113

        return data

    def motor_initialize(self):
        '''
        start_velocity : 1510
        :return:
        '''
        while 1:
            motor_msg = input("측정 속도를 입력하시오 : ")
            pub1, pub2 = barc_project.pub()
            pub2.publish(int(motor_msg))

    def plot_data(self, x1, x2, cnt):

        # plot
        plt.axis([0, 300, -300, 300])
        sub1, = plt.plot(cnt, x1, 'o', color='blue', ms=5)
        sub2, = plt.plot(cnt, x2, 'o', color='red', ms=5)
        plt.pause(0.0001)
        # sub1.remove()
        # sub2.remove()

    def main_1(self):
        # 셋팅값
        yaw = 1

        angle_start = input("시작 각도를 입력하시오 : ")
        motor_start = input("시작 전압를 입력하시오 : ")
        pub1, pub2 = self.pub()
        pub1.publish(int(angle_start))
        pub2.publish(int(motor_start))

        # PID parameter
        slip_e_prev, slip_ui_prev = [0, 0]
        angle_e_prev, angle_ui_prev = [0, 0]
        p_gain, i_gain, d_gain = [1, 10, 10]
        desire_angle = yaw + 0 # 원하는 각도 입력
        desire_slip = 0

        angle_msg = 0
        cnt = 0
        while True:

            # motor control
            slip_e = self.control_tcs() - desire_slip  # error initialize
            slip_ui = slip_ui_prev + 1 / i_gain * dt * slip_e
            slip_ud = 1 / d_gain * (slip_e - slip_e_prev) / dt
            slip_e_prev = slip_e
            slip_ui_prev = slip_ui
            slip_msg = round(p_gain * (slip_e + slip_ui + slip_ud))  # PID 제어 출력

            if slip_msg < desire_slip: # 슬립이 목표보다 클 경우
                motor_start = motor_start - Rw/(slip_msg + 1)
            else : # 슬립이 목표보다 작을 경우
                motor_start = motor_start + Rw/(slip_msg + 1)

            motor_output = motor_start

            # angle control
            angle_current = yaw - desire_angle  # angle initialize
            angle_e = angle_msg - angle_current  # error initialize
            angle_ui = angle_ui_prev + 1 / i_gain * dt * angle_e
            angle_ud = 1 / d_gain * (angle_e - angle_e_prev) / dt
            angle_e_prev = angle_e
            angle_ui_prev = angle_ui
            angle_msg = round(p_gain * (angle_e + angle_ui + angle_ud)) # PID 제어 출력
            angle_output = self.servo_setting(angle_msg + 135) # 서보모터 출력

            # 출력값
            print("현재 각도 : {}".format(angle_current))
            print("서보 각도 : {}".format(angle_output))

            # message 방출
            pub1.publish(int(angle_output))
            pub2.publish(int(motor_output))
            rate = rospy.Rate(hz)
            rate.sleep()


            # 데이터 그래프로 나타내기
            # self.plot_data(angle_current, angle_output, cnt)
            # self.plot_data(slip_msg, motor_output, cnt)

            cnt += 1

#executable
if __name__ == '__main__':
    barc_project = barc()
    logic = logic()

    barc_project.sub()
    barc_project.pub()

    print("******************************\n")
    print("왼쪽 최대 각도      : 157 degree")
    print("중립 각도          : 135 degree")
    print("오른쪽 최대 각도    : 113 degree")
    print("조향 각도          : 22 degree")
    print("최소 시작 속도      : 1510")
    print("******************************\n")

    try:
        # logic.main_1()
        logic.servo_initialize()
        # logic.motor_initialize()
    except rospy.ROSInterruptException:
        pass
