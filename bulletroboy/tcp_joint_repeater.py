import pybullet as p
import rospy
from sensor_msgs.msg import JointState

rospy.init_node("bullet_joint_repeater")
joint_target_pub = rospy.Publisher("/joint_targets", JointState, queue_size=1)
client = p.connect(p.TCP, "192.168.2.114")


p.syncBodyInfo(client)
roboy = 0
msg = JointState()

joint_names = []
for i in range(p.getNumJoints(roboy)):
    ji = p.getJointInfo(roboy,i)
    joint_names.append(ji[1].decode("utf-8"))

msg.velocity = [0]*len(joint_names)
msg.effort = [0]*len(joint_names)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    msg.position = []
    msg.name = []
    for i in range(p.getNumJoints(roboy)):
        if ("lh_" in joint_names[i] or 
                "rh_" in joint_names[i] or
                "camera" in joint_names[i]):
            continue
        js = p.getJointState(roboy, i)
        msg.position.append(js[0])
        msg.name.append(joint_names[i])
    joint_target_pub.publish(msg)
    rate.sleep()
p.disconnect()
