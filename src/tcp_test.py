import pybullet as p
import rospy
from sensor_msgs.msg import JointState
rospy.init_node("tcptest")

joint_cmd = []
active_joints = []

cl = p.connect(p.TCP, "192.168.2.114",6667)
p.syncBodyInfo(cl)
p.resetSimulation()
# p.syncUserData(cl)
rospy.loginfo("num bodies: " + str(p.getNumBodies()))
rospy.loginfo("num joints: " + str(p.getNumJoints(1)))
roboy = p.loadURDF('C:\\Users\\roboy\\Documents\\code\\roboy3_models\\upper_body\\bullet.urdf', useFixedBase=1)
joints = []
for i in range(p.getNumJoints(roboy)):
    print(p.getJointInfo(roboy, i)[1].decode("utf-8"))
    joints.append(p.getJointInfo(roboy, i)[1].decode("utf-8"))
    p.setJointMotorControl2(roboy, i, p.POSITION_CONTROL, targetPosition=0, force=500)

def joint_target_cb(msg):
    global joint_cmd, active_joints
    # rospy.loginfo_throttle(1, msg.name)
    active = []
    positions = []
    for i in range(len(msg.name)):
        j = msg.name[i]
        if "head" in j:
            continue
        try:
            idx = joints.index(j)
            active.append(idx)
            positions.append(msg.position[i])
            # p.setJointMotorControl2(roboy,
            #                         idx,
            #                         p.POSITION_CONTROL,
            #                         targetPosition=msg.position[i],
            #                         force=500)
        except Exception as e:
            # str = msg.name[i] + ": " + str(e)
            # rospy.logwarn_throttle(1, msg.name[i])
            rospy.logwarn_throttle(1, e)
        # except:
        #     rospy.logwarn("Joint %s was not found in pybullet model"%j)
    active_joints = active
    joint_cmd = positions
    # try:
    #     p.setJointMotorControlArray(roboy,
    #                             active,
    #                             p.POSITION_CONTROL,
    #                             targetPositions=msg.position)
    # except Exception as e:
    #     rospy.logwarn_throttle(1, e)
    # p.syncBodyInfo()
    # rospy.loginfo_throttle(1, p.getNumJoints(roboy))
    # rospy.loginfo_throttle(1, "set joint motor control")
    # p.stepSimulation()



joint_target_sub = rospy.Subscriber("/cardsflow_joint_states", JointState, joint_target_cb)
r = rospy.Rate(100)
#p.setGravity(0,0,-10)
#p.setRealTimeSimulation(1)
while not rospy.is_shutdown():
    r.sleep()
    p.setJointMotorControlArray(roboy,
                                active_joints,
                                p.POSITION_CONTROL,
                                targetPositions=joint_cmd)
# rospy.spin()