import pybullet as p
import time
import os
import yaml
import matplotlib.pyplot as plt

print("Started logging and plotting script.")

LOG_DIR=os.path.dirname(os.path.realpath(__file__))+"\\..\\logs\\"
if not os.path.exists(LOG_DIR):
    os.makedirs(LOG_DIR)
print(f"Logging to: {LOG_DIR}")

p.connect(p.SHARED_MEMORY)
humanoid = 0
relevantJoints = [8, 9, 10, 11, 12, 13, 14]
relevantJointsNames = []
jointStates = {}

for j in relevantJoints:
    name = p.getJointInfo(humanoid,j)[1].decode("utf-8")
    relevantJointsNames.append(name)
    jointStates[name] = {}
    jointStates[name]["position"]=[]
    jointStates[name]["reactionForces"]=[]
    jointStates[name]["torque"]=[]

saved = True
p.setRealTimeSimulation(1)
timeStep = 0
startRecordStep = None
stop = False
while(not stop):
    try:
        timeStep += 1
        p.syncUserData()
        # p.syncBodyInfo()
        # p.stepSimulation()
        # print(p.getUserData(0))
        if p.getUserData(0) == b'False' and startRecordStep is None: # server is not set to manual control mode
            startRecordStep = timeStep
        #     print("Recording joint data")
        saved = False
        for j in relevantJoints:
            jointState = p.getJointState(humanoid,j)
            name = p.getJointInfo(humanoid,j)[1].decode("utf-8")
            jointStates[name]["position"].append(jointState[0])
            jointStates[name]["reactionForces"].append(jointState[2])
            jointStates[name]["torque"].append(jointState[3])

        if p.getUserData(0) == b'True' and startRecordStep is not None:
            stopRecordStep = timeStep
            data_log = {}
            # stop = True
            
            plottedJoints = ["Shoulder1_Right","Shoulder2_Right","Shoulder3_Right","Elbow1_X_Right"]
            fig, axs = plt.subplots(len(plottedJoints),2)
            for i in range(len(plottedJoints)):
                recordedPositions = jointStates[plottedJoints[i]]["position"][startRecordStep-100:stopRecordStep]
                recordedTorques = jointStates[plottedJoints[i]]["torque"][startRecordStep-100:stopRecordStep]
                recordedReactionForces = jointStates[plottedJoints[i]]["reactionForces"][startRecordStep-100:stopRecordStep]
                data_log[plottedJoints[i]] = {}
                data_log[plottedJoints[i]]["positions"] = recordedPositions
                data_log[plottedJoints[i]]["torques"] = recordedTorques
                data_log[plottedJoints[i]]["reactionForces"] = recordedReactionForces

                axs[i,0].plot(recordedPositions)
                axs[i,0].set_title(f"{plottedJoints[i]} position")
                axs[i,1].plot(recordedTorques)
                axs[i,1].set_title(f"{plottedJoints[i]} torque")      
            startRecordStep = None        

            # cur_time = time.strftime("%b-%d-%H-%M-%S", time.localtime())
            # with open(f"{LOG_DIR}//joint-torque-{cur_time}.yaml", "w") as f:
            #     yaml.dump(data_log, f)
            # print(f"Saved to joint-torque-{cur_time}.yaml")

            plt.show()  

            
            
    except:
        # if p.getUserData(0) == b'True' and not saved:
        # cur_time = time.strftime("%b-%d-%H-%M-%S", time.localtime())
        # with open(f"joint-torque-{cur_time}.yaml", "w") as f:
        #     yaml.dump(jointStates, f)
        # print(f"Saved to joint-torque-{cur_time}.yaml")
        # saved = True
        pass

# fig, axs = plt.subplots(len(relevantJoints))
# print(f"started recording at timeStep: {startRecordStep}\nstopped at: {stopRecordStep}")


# fig, axs = plt.subplots(3,2)
# plottedJoints = ["Shoulder1_Right","Shoulder2_Right","Shoulder3_Right"]

# for i in range(len(plottedJoints)):
#     axs[i,0].plot(jointStates[plottedJoints[i]]["position"][startRecordStep-100:stopRecordStep])
#     axs[i,0].set_title(f"{plottedJoints[i]} position")
#     axs[i,1].plot(jointStates[plottedJoints[i]]["torque"][startRecordStep-100:stopRecordStep])
#     axs[i,1].set_title(f"{plottedJoints[i]} torque")

# # for i in range(len(relevantJoints)):
# #     axs[i].plot(jointStates[relevantJointsNames[i]]["position"])
# #     axs[i].set_title(relevantJointsNames[i])

# plt.show()
#     # time.sleep(0.1)

