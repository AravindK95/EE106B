#!/usr/bin/env python
import sys
import rospy
import roslib
from reflex_sf_msgs.msg import SFPose


presets = {
    'o' : (0.0, 0.0, 0.0),
    'c' : (3.5, 3.5, 3.5)
}

def main():
    rospy.init_node("reflex_ctrl")
    pub = rospy.Publisher('/reflex_sf/command', SFPose, queue_size=10)

    while not rospy.is_shutdown():
        cmd = raw_input("reflex pose vals/preset >> ")
        cmd = cmd.split()

        if cmd[0] in presets:
            p = presets[cmd[0]]
            pub.publish(p[0], p[1], p[2], 0.0)
            continue

        try:
            cmd = [float(x) for x in cmd]
        except ValueError:
            print("Bad command!")
            continue
        pub.publish(cmd[0], cmd[1], cmd[2], 0.0)


if __name__ == '__main__':
    main()