#! /usr/bin/env python
#encoding: utf8
import sys, rospy, math, tf
from pimouse_ros.msg import MotorFreqs
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.srv import TimedMotion
from nav_msgs.msg import Odometry

class Motor():
    def __init__(self):
        if not self.set_power(False): sys.exit(1)

        rospy.on_shutdown(self.set_power)
        self.sub_raw = rospy.Subscriber('motor_raw', MotorFreqs, self.callback_raw_freq)
        self.seb_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.callback_cmd_vel)
        self.srv_on = rospy.Service('motor_on', Trigger, self.callback_on)
        self.srv_off = rospy.Service('motor_off', Trigger, self.callback_off)
        self.last_time = rospy.Time.now()
        self.using_cmd_vel = False
        self.srv_tm = rospy.Service('timed_motion', TimedMotion, self.callback_tm)

        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10) # odomトピックの準備。トピックの型はnav_masg/Odometry
        self.bc_odom = tf.TransformBroadcaster() # tfに変換をブロードキャストするもの

        self.x, self.y, self.th = 0.0, 0.0, 0.0 # デッドレコニングに基づいて計算したロボットの現在位置・向きを記録する変数を定義して初期化
        self.vx, self.vth = 0.0, 0.0 # ロボットの速度と角速度を格納する変数を作って初期化

        self.cur_time = rospy.Time.now() # オドメトリや座標の情報を送るときの時刻
        self.last_time = self.cur_time # 前回に情報を送ったときの時刻

    def set_power(self, onoff=False):
        en = "/dev/rtmotoren0"
        try:
            with open(en, 'w') as f:
                f.write("1\n" if onoff else "0\n")
            self.is_on = onoff
            return True
        except:
            rospy.logerr("cannot write to " + en)

        return False

    def set_raw_freq(self, left_hz, right_hz):
        if not self.is_on:
            rospy.logerr("not enpowered")
            return

        try:
            with open("/dev/rtmotor_raw_l0", "w") as lf, open("/dev/rtmotor_raw_r0", "w") as rf:
                lf.write(str(int(round(left_hz))) + "\n")
                rf.write(str(int(round(right_hz))) + "\n")
        except:
            rospy.logerr("cannot write to rtmotor_raw_*")

    def callback_raw_freq(self, message):
        self.set_raw_freq(message.left_hz, message.right_hz)

    def callback_cmd_vel(self, message):
        if not self.is_on:
            return
        self.vx = message.linear.x
        self.vth = message.angular.z

        forward_hz = 80000.0*message.linear.x/(9*math.pi)
        rot_hz = 400.0*message.angular.z/math.pi
        self.set_raw_freq(forward_hz-rot_hz, forward_hz+rot_hz)
        self.using_cmd_vel = True
        self.last_time = rospy.Time.now()

    def onoff_response(self, onoff):
        d = TriggerResponse()
        d.success = self.set_power(onoff)
        d.message = "ON" if self.is_on else "OFF"
        return d

    def callback_on(self, message):
        return self.onoff_response(True)

    def callback_off(self, message):
        return self.onoff_response(False)

    def callback_tm(self, message):
        if not self.is_on:
            rospy.logerr("not enpowered")
            return False

        dev = "/dev/rtmotor0"
        try:
            with open(dev, 'w') as f:
                f.write("%d %d %d\n" % (message.left_hz, message.right_hz, message.duration_ms))

        except:
            rospy.logerr("cannot write to " + dev)
            return False

        return True

    def send_odom(self):
        self.cur_time = rospy.Time.now() # 現在時刻を代入

        dt = self.cur_time.to_sec() - self.last_time.to_sec() # 前回の処理からの経過時間を代入。to_sec()はrospy.Timeのメソッド
        # 現在のロボットの速度・角速度からデッドレコニングで計算したロボットの姿勢の値self.x, self.y, self.thを更新。
        # ロボットの進行方向の速度self.vx、前回計算した向きself.th、前回の処理からの時間dtからデッドレコニングで求めているロボットの姿勢の変化量を求めている。
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        self.th += self.vth * dt # last_timeからcur_timeの間、ロボットの向きがself.thだと仮定。dtが大きいと誤差が発生する。

        # self.thをgeometry_msgs/Quaternion型のqに変換。tfが座標系の向きをクォータニオンで入出力するため変換が必要。
        # クォータニオンにより3次元中でのものの向きが簡潔に表現、演算できるため、関節が多いロボットにとっては有用で、tfが受け付けるようになる。
        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        # tfに計算結果を送信。引数はロボットの位置（高さは0.0）、q、座標系を計算した時刻（現在時刻）、子供のフレーム、親のフレーム。
        # base_link：ロボット座標系、odom：オドメトリを計算するためのグローバル座標、
        self.bc_odom.sendTransform((self.x, self.y, 0.0), q, self.cur_time, "base_link", "odom")

        odom = Odometry() # Odometry型インスタンスの作成。計算した値が入る。
        odom.header.stamp = self.cur_time # 時刻の指定
        odom.header.frame_id = "odom" # 親のフレームの指定
        odom.child_frame_id = "base_link" # 子のフレームの指定

        # 親のフレームに対する子供のフレームの相対姿勢をセット
        odom.pose.pose.position = Point(self.x, self.y, 0)
        odom.pose.pose.orientation = Quaternion(*q) # *q（リストの中身）をバラバラに分解して引数として渡す

        # 子供のフレームで見たときの子供のフレームの運動をセット
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        self.pub_odom.publish(odom) # オドメトリ情報をパブリッシュ

        self.last_time = self.cur_time

if __name__ == '__main__':
    rospy.init_node('motors')
    m = Motor()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        m.send_odom() # odomはオドメトリの略
        rate.sleep()
