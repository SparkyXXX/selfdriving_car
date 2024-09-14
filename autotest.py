#!/usr/bin/env python
# coding=utf-8
import rospy
# 倒入自定义的数据类型

from std_msgs.msg import Int32


def talker():
    # Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
    # queue_size: None（不建议）  #这将设置为阻塞式同步收发模式！
    # queue_size: 0（不建议）#这将设置为无限缓冲区模式，很危险！
    # queue_size: 10 or more  #一般情况下，设为10 。queue_size太大了会导致数据延迟不同步。
    pub1 = rospy.Publisher('/bluetooth/received/manul', Int32, queue_size=10)
    pub2 = rospy.Publisher('/auto_driver/send/direction', Int32, queue_size=10)
    pub3 = rospy.Publisher('/auto_driver/send/speed', Int32, queue_size=10)
    pub4 = rospy.Publisher('/auto_driver/send/gear', Int32, queue_size=10)
    manul = 0
    speed = 00
    direction = 50
    gear = 1

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    # 更新频率是1hz
    while not rospy.is_shutdown():
        speed = 30
        direction = 0
        rospy.loginfo('Talker: direction=%f ,speed= %f', direction, speed)
        pub1.publish(manul)
        pub2.publish(direction)
        pub3.publish(speed)
        pub4.publish(gear)
        rate.sleep()


if __name__ == '__main__':
    talker()
