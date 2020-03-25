#! /usr/bin/env python
#encoding: utf8
import rospy, copy, math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

class WallAround():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.sensor_values = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.callback)

    def callback(self, messages):
        self.sensor_values = messages

    def wall_front(self, ls):
        return ls.left_forward > 50 or ls.right_forward > 50

    def too_right(self, ls):
        return ls.right_side > 50

    def too_left(self, ls):
        return ls.left_side > 50

    def run(self):
        rate = rospy.Rate(20)
        data = Twist()

        data.linear.x = 0.05 # 直進速度0.05[m/s]
        data.angular.z = 0.0
        while not rospy.is_shutdown():
            data.linear.x = 0.05 # 直進速度0.05[m/s]
            if self.wall_front(self.sensor_values): # 壁が前にあるか
                data.linear.x = 0.0 # 直進速度0.0[m/s]
                data.angular.z = - math.pi/2 # 角速度を-π/2（右回り）に設定
#            elif self.too_right(self.sensor_values): # 右の壁に近づきすぎてないか
#		data.linear.x = 0.0 # 直進速度0.0[m/s]
#               data.angular.z = math.pi * 0.1 # 壁から遠ざかるように左側にそれるように進む。
#            elif self.too_left(self.sensor_values): # 左の壁に近づきすぎてないか
#		data.linear.x = 0.0 # 直進速度0.0[m/s]
#               data.angular.z = - math.pi * 0.1 # 壁から遠ざかるように右側にそれるように進む。
            else: # それ以外。左向きのセンサが50になるように比例制御で角速度を調整。
                e = 50 - self.sensor_values.left_side
#		data.linear.x = 0.0 # 直進速度0.0[m/s]
#               data.angular.z = e * math.pi / 180.0 #センサの値が1につき、角速度1[deg/s]変化させる。曲がり角で左折する。

            self.cmd_vel.publish(data)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_around')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()
    WallAround().run()
