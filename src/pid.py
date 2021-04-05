from time import sleep
import threading


class MockRobotEncoder(object):
    def __init__(self, speed_error):
        self._speed_error = speed_error
        self._value = 0
        self._total = 0
        self.speed = 0.5

        self._t = threading.Thread(
            target=self._mock_encoder,
            args=(0.1,))
        self._t.start()

    def reset(self):
        self._value = 0

    def _mock_encoder(self, interval):
        while True:
            self._increment()
            # sleep differing amounts based on the speed and error introduced
            sleep(interval * (2 - self._speed) * self._speed_error)

    def _increment(self):
        self._value += 1
        self._total += 1

    @property
    def value(self):
        return int(self._value)

    @property
    def speed(self):
        return self._speed

    @property
    def total(self):
        return int(self._total)

    @speed.setter
    def speed(self, value):
        self._speed = value


KP = 2
KD = 0
KI = 0
TARGET = 0

motor_speed = 0.5
e1 = MockRobotEncoder(1.13) # 엔코더값
e1_prev_error = 0
e1_sum_error = 0

while True:
    e1_error = TARGET - e1.value
    e1_sum_error += e1_error
    print(e1.speed)
    print(e1.value)

    e1_adj = (e1_error * KP) + (e1_prev_error * KD) + (e1_sum_error * KI)
    e1_adj = round(e1_adj,2)

    e1_prev_error = e1_error

    motor_speed += e1_adj
    motor_speed = round(motor_speed,2)

    e1.speed = motor_speed

    print("********************************")
    print("error1 :: {} ".format(e1_error))
    print("adj1 :: {}".format(e1_adj))
    print("e1 :: {}".format(e1.value))
    print("m1 :: {}".format(motor_speed))
    print("********************************")

    e1.reset()

    sleep(1)