import pybullet as p
import time
import pybullet_data
import numpy as np
import random

physicsClient = p.connect(p.GUI)

#####  Load_cube
def loadCube_B(x, y):
    cubeBStartPos = [x, y, 0.635]
    cubeId = p.loadURDF("/KUKA-NiaPy/Nia-master/cube/cube_B.urdf", cubeBStartPos, globalScaling=0.5)
def loadCube_R(x, y):
    cubeRStartPos = [x, y, 0.635]
    cube2Id = p.loadURDF("/KUKA-NiaPy/Nia-master/cube/cube_R.urdf", cubeRStartPos, globalScaling=0.5)
def loadCube_G(x, y):
    cubeGStartPos = [x, y, 0.635]
    cube3Id = p.loadURDF("/KUKA-NiaPy/Nia-master/cube/cube_G.urdf", cubeGStartPos, globalScaling=0.5)
def reset():
    global robotId
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    planeId = p.loadURDF("plane.urdf")
    robotId = p.loadSDF("/KUKA-NiaPy/Nia-master/kuka_iiwa/kuka_with_gripper.sdf")[0]
    robotStartPos = [0, 0, 0.695]
    robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    p.resetBasePositionAndOrientation(robotId, robotStartPos, robotStartOrientation)

    # 轉變視角
    p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.5, 0, 0.5])

    trayStartPos = [0.96, -0.28, 0.63]
    tray2StartPos = [0.96, 0.28, 0.63]
    tray3StartPos = [0.96, 0, 0.63]
    trayId = p.loadURDF("/KUKA-NiaPy/Nia-master/tray/trayboxB.urdf", trayStartPos,
                        globalScaling=0.4)
    trayId2 = p.loadURDF("/KUKA-NiaPy/Nia-master/tray/trayboxR.urdf", tray2StartPos,
                         globalScaling=0.4)
    trayId3 = p.loadURDF("/KUKA-NiaPy/Nia-master/tray/trayboxG.urdf", tray3StartPos,
                         globalScaling=0.4)

    tableStartPos = [0.7, 0, 0]
    tableId = p.loadURDF("/KUKA-NiaPy/Nia-master/table/table.urdf",
                         tableStartPos)
    loadCube_R(0.76, 0)
    loadCube_R(0.65, 0.28)
    loadCube_R(0.54, 0.28)
    loadCube_B(0.76, 0.28)
    loadCube_B(0.65, -0.28)
    loadCube_B(0.54, -0.28)
    loadCube_G(0.76, -0.28)
    loadCube_G(0.65, 0)
    loadCube_G(0.54, 0)
    jointPositions = [0, 0, 0, 0, 0, 0, 0, 0, -0.2, 0, 0, 0.2, 0, 0]
    for jointIndex in range(8, 14):
        p.setJointMotorControl2(robotId, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex])
def jointControl(joint_poses, gripper_val):
    for j in range(0, 8):
        p.setJointMotorControl2(bodyIndex=robotId, jointIndex=j, controlMode=p.POSITION_CONTROL,
                                targetPosition=joint_poses[j])
    p.setJointMotorControl2(robotId, 10, p.POSITION_CONTROL, targetPosition=gripper_val, force=100)
    p.setJointMotorControl2(robotId, 13, p.POSITION_CONTROL, targetPosition=-gripper_val, force=100)
    p.stepSimulation()
    # time.sleep(1. / 240.)

def motion6(x, y):
    for t in range(650):  #Duration
        target_pos, gripper_val = [x, y, 0.91], 0
        if t >= 90 and t < 190:
            target_pos, gripper_val = [x, y, 0.86], 1  # grab object
        elif t >= 190 and t < 290:
            target_pos, gripper_val = [x, y, 0.89 + 0.13 * (t - 190) / 100.], 1  # move up after picking object
        elif t >= 290 and t < 440:
            target_pos, gripper_val = [x+0.55 * (t - 290) / 150., y-0.04 * (t - 290) / 150., 1.02], 1  # move to target position
        elif t >= 440 and t < 540:
            target_pos, gripper_val = [x+0.55, y-0.04, 0.975], 0  # stop at target position
        elif t >= 540:
            target_pos, gripper_val = [x, -0.03, 0.92], 0  # drop object
        target_orn = p.getQuaternionFromEuler([0, 3.14, 0])
        joint_poses = p.calculateInverseKinematics(robotId, 7, target_pos, target_orn)
        jointControl(joint_poses, gripper_val)
def motion7(x, y):
    for t in range(650):  #Duration
        target_pos, gripper_val = [x, y, 0.91], 0
        if t >= 90 and t < 190:
            target_pos, gripper_val = [x, y, 0.87], 1  # grab object
        elif t >= 190 and t < 290:
            target_pos, gripper_val = [x, y, 0.89 + 0.13 * (t - 190) / 100.], 1  # move up after picking object
        elif t >= 290 and t < 440:
            target_pos, gripper_val = [x+0.55 * (t - 290) / 150., y, 1.02], 1  # move to target position
        elif t >= 440 and t < 540:
            target_pos, gripper_val = [x+0.55, y, 0.975], 0  # stop at target position
        elif t >= 540:
            target_pos, gripper_val = [x, -0.03, 0.92], 0  # drop object
        target_orn = p.getQuaternionFromEuler([0, 3.14, 0])
        joint_poses = p.calculateInverseKinematics(robotId, 7, target_pos, target_orn)
        jointControl(joint_poses, gripper_val)
def motion9(x, y):
    for t in range(650):  #Duration
        target_pos, gripper_val = [x, y, 0.91], 0
        if t >= 90 and t < 190:
            target_pos, gripper_val = [x, y, 0.87], 1  # grab object
        elif t >= 190 and t < 290:
            target_pos, gripper_val = [x, y, 0.89 + 0.13 * (t - 190) / 100.], 1  # move up after picking object
        elif t >= 290 and t < 440:
            target_pos, gripper_val = [x+0.2 * (t - 290) / 150., y+0.3*(t-290)/150, 1.02], 1  # move to target position
        elif t >= 440 and t < 540:
            target_pos, gripper_val = [x+0.2, y+0.3, 0.975], 0  # stop at target position
        elif t >= 540:
            target_pos, gripper_val = [x, -0.03, 0.93], 0  # drop object
        target_orn = p.getQuaternionFromEuler([0, 3.14, 0])
        joint_poses = p.calculateInverseKinematics(robotId, 7, target_pos, target_orn)
        jointControl(joint_poses, gripper_val)
def motion8(x, y):
    for t in range(650):  #Duration
        target_pos, gripper_val = [x, y, 0.91], 0
        if t >= 90 and t < 190:
            target_pos, gripper_val = [x, y, 0.87], 1  # grab object
        elif t >= 190 and t < 290:
            target_pos, gripper_val = [x, y, 0.89 + 0.13 * (t - 190) / 100.], 1  # move up after picking object
        elif t >= 290 and t < 440:
            target_pos, gripper_val = [x+0.2 * (t - 290) / 150., y-0.6*(t-290)/150, 1.02], 1  # move to target position
        elif t >= 440 and t < 540:
            target_pos, gripper_val = [x+0.2, y-0.6, 0.975], 0  # stop at target position
        elif t >= 540:
            target_pos, gripper_val = [x, -0.03, 0.93], 0  # drop object
        target_orn = p.getQuaternionFromEuler([0, 3.14, 0])
        joint_poses = p.calculateInverseKinematics(robotId, 7, target_pos, target_orn)
        jointControl(joint_poses, gripper_val)

def sort_list(l):
    alpha_list_random = []
    weighted_dict_random = {'A': l[0], 'B': l[1], 'C': l[2],
                            'D': l[3], 'E': l[4], 'F': l[5],
                            'G': l[6], 'H': l[7], 'I': l[8]}
    sorted_list_random = sorted(weighted_dict_random.items(), reverse=True, key=lambda x: x[1])
    for n in range(0, 9):
        alpha_list_random.append(sorted_list_random[n][0])
    return alpha_list_random
def decide_motion(str):
    weighted_matrix = {'A': [0.752, 0.26, 8], 'B': [0.752, -0.02, 9], 'C': [0.752, -0.3, 9], 'D': [0.642, 0.26, 6],
                       'E': [0.642, -0.02, 6], 'F': [0.642, -0.3, 6], 'G': [0.532, 0.26, 7], 'H': [0.532, -0.02, 7],
                       'I': [0.532, -0.3, 7]}
    n = weighted_matrix[str][2]
    if n == 6:
        motion6(weighted_matrix[str][0], weighted_matrix[str][1])
    if n == 7:
        motion7(weighted_matrix[str][0], weighted_matrix[str][1])
    if n == 8:
        motion8(weighted_matrix[str][0], weighted_matrix[str][1])
    if n == 9:
        motion9(weighted_matrix[str][0], weighted_matrix[str][1])
def move(str):
    reset()
    processTime = 0
    list_processTime = []
    # start = time.process_time()
    start = time.perf_counter()
    for i in range(0, 9):
        decide_motion(str[i])
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
