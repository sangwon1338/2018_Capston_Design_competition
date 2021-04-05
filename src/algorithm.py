#! /usr/bin/env python
#-*-coding:utf-8-*-

import rospy

# 센서 데이터를 전처리 하는 객체
class py_sensor():
    # array의 평균을 구하는 함수
    def mean_data(self,values):
        data = sum(values, 0.0) / len(values)
        return data

    # 홀 센서 데이터로 모터 맵핑하는 함수
    def motor_mapping(self, data):
        if (data == 6):
            return 10
        elif (data== 7):
            return 20
        elif(data > 7 and data< 11):
            return 35
        elif (data == 12):
            return 50
        elif(data > 12):
            return 60

    # 홀 센서 데이터로 앞,뒤바퀴의 휠 속도 구하는 함수
    def hall_to_velocity(self, front_hall, rear_hall ):
        front_v = self.motor_mapping(front_hall)
        rear_v = self.motor_mapping(rear_hall)
        return front_v, rear_v

# 센서 데이터를 이용해 동역학적 알고리즘을 작성하는 객체
class py_dynamic(py_sensor):
    # 슬립율 구하는 함수
    def slip_ratio(self, front_hall, rear_hall):
        # 앞, 뒤 휠 속도
        front_v, rear_v = self.hall_to_velocity(front_hall, rear_hall)
        if rear_v == 0 :
            slip_ratio = 0
        else :
            slip_ratio = (rear_v - front_v)/ rear_v

        return slip_ratio

    def anti_slip(self,slip, motor_start):
        if slip == 0:
            motor_start += 1
        else:
            motor_start -= 1

        return motor_start

    def velocity_to_volatge(self):
        print("속도를 전압으로 바꿔서 모터에 전송해줘야")

    def pid_controller(self,y_target, y_current, h, i_gain, d_gain, p_gain, u0, e0):
        """Calculate System Input using a PID Controller
        Arguments:
        y  .. Measured Output of the System
        yc .. Desired Output of the System
        h  .. Sampling Time
        Kp .. Controller Gain Constant
        Ti .. Controller Integration Constant
        Td .. Controller Derivation Constant
        u0 .. Initial state of the integrator
        e0 .. Initial error
        Make sure this function gets called every h seconds!
        """

        # Initialization
        e_prev = e0
        ui_prev = u0

        while 1:
            e = y_target - y_current

            ui = ui_prev + 1 / i_gain * h * e
            ud = 1 / d_gain * (e - e_prev) / h

            e_prev = e
            ui_prev = ui

            y_target = p_gain * (e + ui + ud)

