#!/usr/bin/python
import rospy

from dynamic_reconfigure.server import Server
from kobuki_control.cfg import PIDConfig

def callback(config, level):
    rospy.loginfo("""Linear velocity:\n Kp={Kpv}, Ki={Kiv}, Kd={Kdv}""".format(**config))
    rospy.loginfo("""Angular velocity:\n Kp={Kpw}, Ki={Kiw}, Kd={Kdw}""".format(**config))
    
    return config

if __name__ == "__main__":
    rospy.init_node("param_server", anonymous=True)
    
    srv = Server(PIDConfig, callback)
    rospy.spin()

