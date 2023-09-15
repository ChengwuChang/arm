import pybullet as p
import time
import pybullet_data
import numpy as np
import random

physicsClient = p.connect(p.GUI)
def loadCube_B(x, y):#藍色方塊起始位置
    cubeBStartPos = [x, y, 0.635]
    cubeId = p.loadURDF("/KUKA-NiaPy/Nia-master/cube/cube_B.urdf", cubeBStartPos, globalScaling=0.5)#方塊大小

def reset():
    global robotId
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)#設置環境重力值，向右跑gravX值為正
    planeId = p.loadURDF("plane.urdf")#設置平面
    robotId = p.loadSDF("/KUKA-NiaPy/Nia-master/kuka_iiwa/kuka_with_gripper.sdf")[0]#相對路徑pybullet手冊有
    robotStartPos = [0, 0, 0.695]#robotarmstartposition
    robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    p.resetBasePositionAndOrientation(robotId, robotStartPos, robotStartOrientation)

  # 轉變視角
    p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.5, 0, 0.5])
    #yaw是視角的角度(左右)pitch是上下視角

    trayStartPos = [0.96, -0.28, 0.63]#藍色托盤位置
    trayId = p.loadURDF("/KUKA-NiaPy/Nia-master/tray/trayboxB.urdf", trayStartPos,
                        globalScaling=0.4)  # 改數字會使托盤大小更動
    tableStartPos = [0.7, 0, 0]
    tableId = p.loadURDF("/KUKA-NiaPy/Nia-master/table/table.urdf",
                         tableStartPos)
    loadCube_B(0.54, -0.28)
    jointPositions = [0, 0, 0, 0, 0, 0, 0, 0, -0.2, 0, 0, 0.2, 0, 0]  # pybullet手冊查詢手臂關節位置
    for jointIndex in range(8, 14):  # 上面有14個數字表示要控制的關節數字
        p.setJointMotorControl2(robotId, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex])


def sort_list(l):
    alpha_list_random = []
    weighted_dict_random = {'A': l[0], 'B': l[1], 'C': l[2],
                            'D': l[3], 'E': l[4], 'F': l[5],
                            'G': l[6], 'H': l[7], 'I': l[8]}  # 標示方塊的數自始電腦能理解哪個是哪個方塊
    sorted_list_random = sorted(weighted_dict_random.items(), reverse=True, key=lambda x: x[1])
    for n in range(0, 9):
        alpha_list_random.append(sorted_list_random[n][0])
    return alpha_list_random

def jointControl(joint_poses, gripper_val):
    for j in range(0, 8):
        p.setJointMotorControl2(bodyIndex=robotId, jointIndex=j, controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_poses[j])
    p.setJointMotorControl2(robotId, 10, p.POSITION_CONTROL, targetPosition=gripper_val, force=100)
    p.setJointMotorControl2(robotId, 13, p.POSITION_CONTROL, targetPosition=-gripper_val, force=100)
    p.stepSimulation()
    time.sleep(1. / 240.)


def jointVelocityControl(velocity_value,gripper_val):
    targetVel2 = 0
    targetVel4 = 0
    p.setJointMotorControl2(robotId, 2, p.VELOCITY_CONTROL, targetVelocity=targetVel2)
    p.setJointMotorControl2(bodyIndex=robotId, jointIndex=2, controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=targetVel2)
    p.setJointMotorControl2(bodyIndex=robotId, jointIndex=4, controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=targetVel4)
    p.setJointMotorControl2(robotId, 10, p.POSITION_CONTROL, targetPosition=gripper_val, force=100)
    p.setJointMotorControl2(robotId, 13, p.POSITION_CONTROL, targetPosition=-gripper_val, force=100)
    p.stepSimulation()
    time.sleep(1. / 240.)


# def motion6(x, y):
#     for t in range(650):  #Duration
#         target_pos, gripper_val = [x, y, 0.91], 0
#         if t >= 90 and t < 190:
#             target_pos, gripper_val = [x, y, 0.86], 1  # grab object
#         elif t >= 190 and t < 290:
#             target_pos, gripper_val = [x, y, 0.89 + 0.13 * (t - 190) / 100.], 1  # move up after picking object
#         elif t >= 290 and t < 440:
#             target_pos, gripper_val = [x+0.55 * (t - 290) / 150., y, 1.02], 1  # move to target position
#         elif t >= 440 and t < 540:
#             target_pos, gripper_val = [x+0.55, y-0.04, 0.975], 0  # stop at target position
#         elif t >= 540:
#             target_pos, gripper_val = [x, -0.03, 0.92], 0  # drop object
#         target_orn = p.getQuaternionFromEuler([0, 3.14, 0])
#         joint_poses = p.calculateInverseKinematics(robotId, 7, target_pos, target_orn)
#         jointControl(joint_poses, gripper_val)


def motion7(x, y):
    for t in range(650):  #Duration
        targetVel , gripper_val = [x, y, 0.91], 0
        if t >= 90 and t < 190:
            target_pos, gripper_val = [x, y, 0.86], 1  # grab object
        elif t >= 190 and t < 290:
            target_pos, gripper_val = [x, y, 0.89 + 0.13 * (t - 190) / 100.], 1  # move up after picking object
        elif t >= 290 and t < 440:
            target_pos, gripper_val = [x+0.55 * (t - 290) / 150., y, 1.02], 1  # move to target position
        elif t >= 440 and t < 540:
            target_pos, gripper_val = [x+0.55, y-0.04, 0.975], 0  # stop at target position
        elif t >= 540:
            target_pos, gripper_val = [x, -0.03, 0.92], 0  # drop object
        target_orn = p.getQuaternionFromEuler([0, 3.14, 0])

        # joint_poses = p.calculateInverseKinematics(robotId, 7, target_pos, target_orn)
        # jointControl(joint_poses, gripper_val)



def move(str):
    reset()
    processTime = 0
    list_processTime = []
    # start = time.process_time()
    start = time.perf_counter()
    for i in range(0, 9):
        # motion6(0.54,-0.28)
        # motion7()
    # end = time.process_time()
    end = time.perf_counter()
    processTime = (end - start)
    list_processTime.append(processTime)
    list_processTime = sorted(list_processTime)
    return processTime, list_processTime

# test
# Select_list = sort_list([5,6,7,8,9,4,3,20,100])
# Select_list = sort_list([8,7,6,5,4,3,2,1,9])
# t, tl = move(Select_list)
# print("運行時間:%f秒" % t)
# print("最速運行時間為:%f秒" % tl[0])
# p.disconnect()
