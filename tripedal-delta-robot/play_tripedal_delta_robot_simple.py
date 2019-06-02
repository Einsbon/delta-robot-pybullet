import pybullet as p
import time
import pybullet_data
import math
import os

import deltaKinematics

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setTimeStep(1 / 1000)
planeID = p.loadURDF('plane.urdf')

p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=20, cameraPitch=0, cameraTargetPosition=[-0, 0, 0.15], physicsClientId=physicsClient)

robotPos = [0, 0, 0.5]
robotScale = 1
robot = p.loadURDF(os.path.abspath(os.path.dirname(__file__)) + '/triDelta.urdf',
                   robotPos,
                   p.getQuaternionFromEuler([0, 0, 0]),
                   useFixedBase=0,
                   globalScaling=robotScale)

jointNameToID = {}
linkNameToID = {}
revoluteID = []

for j in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, j)
    # print(info)
    jointID = info[0]
    jointName = info[1].decode('UTF-8')
    jointType = info[2]
    if (jointType == p.JOINT_REVOLUTE):
        jointNameToID[jointName] = info[0]
        linkNameToID[info[12].decode('UTF-8')] = info[0]
        revoluteID.append(j)

colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=0.02 * robotScale)
visualShapeId = -1

ballposA = [robotPos[0] + 0.12 * robotScale, robotPos[1], robotPos[2] + -0.26525 * robotScale]
ballposB = [robotPos[0] - 0.06 * robotScale, robotPos[1] + 0.103923048454 * robotScale, robotPos[2] - 0.26525 * robotScale]
ballposC = [robotPos[0] - 0.06 * robotScale, robotPos[1] - 0.103923048454 * robotScale, robotPos[2] - 0.26525 * robotScale]

sphereA = p.createMultiBody(0.03, colSphereId, visualShapeId, ballposA)
sphereB = p.createMultiBody(0.03, colSphereId, visualShapeId, ballposB)
sphereC = p.createMultiBody(0.03, colSphereId, visualShapeId, ballposC)

constraintPosition = [[[0.03, -0.02, 0.01], [0.03, 0.02, 0.01]], [[0.00232050807, 0.03598076211, 0.01], [-0.03232050807, 0.01598076211, 0.01]],
                      [[-0.03232050807, -0.01598076211, 0.01], [0.00232050807, -0.03598076211, 0.01]]]

constraintPositionScale = [[[0.03 * robotScale, -0.02 * robotScale, 0.01 * robotScale], [0.03 * robotScale, 0.02 * robotScale, 0.01 * robotScale]],
                           [[0.00232050807 * robotScale, 0.03598076211 * robotScale, 0.01 * robotScale],
                            [-0.03232050807 * robotScale, 0.01598076211 * robotScale, 0.01 * robotScale]],
                           [[-0.03232050807 * robotScale, -0.01598076211 * robotScale, 0.01 * robotScale],
                            [0.00232050807 * robotScale, -0.03598076211 * robotScale, 0.01 * robotScale]]]

# A constraint
p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_A_1_parallel_1_link'],
                   childBodyUniqueId=sphereA,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[0][0])
p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_A_1_parallel_2_link'],
                   childBodyUniqueId=sphereA,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[0][1])

p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_A_2_parallel_1_link'],
                   childBodyUniqueId=sphereA,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[1][0])
p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_A_2_parallel_2_link'],
                   childBodyUniqueId=sphereA,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[1][1])

p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_A_3_parallel_1_link'],
                   childBodyUniqueId=sphereA,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[2][0])
p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_A_3_parallel_2_link'],
                   childBodyUniqueId=sphereA,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[2][1])

# B constraint
p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_B_1_parallel_1_link'],
                   childBodyUniqueId=sphereB,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[0][0])
p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_B_1_parallel_2_link'],
                   childBodyUniqueId=sphereB,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[0][1])

p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_B_2_parallel_1_link'],
                   childBodyUniqueId=sphereB,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[1][0])
p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_B_2_parallel_2_link'],
                   childBodyUniqueId=sphereB,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[1][1])

p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_B_3_parallel_1_link'],
                   childBodyUniqueId=sphereB,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[2][0])
p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_B_3_parallel_2_link'],
                   childBodyUniqueId=sphereB,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[2][1])

# C constraint
p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_C_1_parallel_1_link'],
                   childBodyUniqueId=sphereC,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[0][0])
p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_C_1_parallel_2_link'],
                   childBodyUniqueId=sphereC,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[0][1])

p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_C_2_parallel_1_link'],
                   childBodyUniqueId=sphereC,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[1][0])
p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_C_2_parallel_2_link'],
                   childBodyUniqueId=sphereC,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[1][1])

p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_C_3_parallel_1_link'],
                   childBodyUniqueId=sphereC,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[2][0])
p.createConstraint(parentBodyUniqueId=robot,
                   parentLinkIndex=linkNameToID['delta_C_3_parallel_2_link'],
                   childBodyUniqueId=sphereC,
                   childLinkIndex=-1,
                   jointType=p.JOINT_POINT2POINT,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, -0.14],
                   childFramePosition=constraintPosition[2][1])

input('robot loaded. press enter to continue')

p.setGravity(0, 0, -9.8)

for i in revoluteID:
    if i in [
            jointNameToID['motor_A_1'], jointNameToID['motor_A_2'], jointNameToID['motor_A_3'], jointNameToID['motor_B_1'],
            jointNameToID['motor_B_2'], jointNameToID['motor_B_3'], jointNameToID['motor_C_1'], jointNameToID['motor_C_2'], jointNameToID['motor_C_3']
    ]:
        pass
    else:
        # set friction of the other joint to 0.001
        p.setJointMotorControl2(robot, i, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=0.001)

deltaTool = deltaKinematics.Delta(70, 140, 30, 30, 10)

for _ in range(1000):
    (targetPos1, targetPos2, targetPos3) = deltaTool.pointToAngle(0, 0, -220)

    p.setJointMotorControl2(robot, jointNameToID['motor_A_1'], p.POSITION_CONTROL, targetPos1, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_A_2'], p.POSITION_CONTROL, targetPos2, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_A_3'], p.POSITION_CONTROL, targetPos3, force=100)

    p.setJointMotorControl2(robot, jointNameToID['motor_B_1'], p.POSITION_CONTROL, targetPos1, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_B_2'], p.POSITION_CONTROL, targetPos2, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_B_3'], p.POSITION_CONTROL, targetPos3, force=100)

    p.setJointMotorControl2(robot, jointNameToID['motor_C_1'], p.POSITION_CONTROL, targetPos1, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_C_2'], p.POSITION_CONTROL, targetPos2, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_C_3'], p.POSITION_CONTROL, targetPos3, force=100)
    p.stepSimulation()

for i in range(1000):
    (targetPos1, targetPos2, targetPos3) = deltaTool.pointToAngle(0, 0, (-220 + i * 50 / 1000))

    p.setJointMotorControl2(robot, jointNameToID['motor_A_1'], p.POSITION_CONTROL, targetPos1, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_A_2'], p.POSITION_CONTROL, targetPos2, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_A_3'], p.POSITION_CONTROL, targetPos3, force=100)

    p.setJointMotorControl2(robot, jointNameToID['motor_B_1'], p.POSITION_CONTROL, targetPos1, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_B_2'], p.POSITION_CONTROL, targetPos2, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_B_3'], p.POSITION_CONTROL, targetPos3, force=100)

    p.setJointMotorControl2(robot, jointNameToID['motor_C_1'], p.POSITION_CONTROL, targetPos1, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_C_2'], p.POSITION_CONTROL, targetPos2, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_C_3'], p.POSITION_CONTROL, targetPos3, force=100)
    p.stepSimulation()

for i in range(1000):
    (targetPos1, targetPos2, targetPos3) = deltaTool.pointToAngle(i * 60 / 1000, 0, -170)

    p.setJointMotorControl2(robot, jointNameToID['motor_A_1'], p.POSITION_CONTROL, targetPos1, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_A_2'], p.POSITION_CONTROL, targetPos2, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_A_3'], p.POSITION_CONTROL, targetPos3, force=100)

    p.setJointMotorControl2(robot, jointNameToID['motor_B_1'], p.POSITION_CONTROL, targetPos1, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_B_2'], p.POSITION_CONTROL, targetPos2, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_B_3'], p.POSITION_CONTROL, targetPos3, force=100)

    p.setJointMotorControl2(robot, jointNameToID['motor_C_1'], p.POSITION_CONTROL, targetPos1, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_C_2'], p.POSITION_CONTROL, targetPos2, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_C_3'], p.POSITION_CONTROL, targetPos3, force=100)
    p.stepSimulation()
count = 0
while (True):
    count = count + 0.0005
    if count > 1:
        count = 0
    x = math.cos(math.pi * 2 * count) * 60
    y = math.sin(math.pi * 2 * count) * 60
    z = -170
    (targetPos1, targetPos2, targetPos3) = deltaTool.pointToAngle(x, y, z)
    p.setJointMotorControl2(robot, jointNameToID['motor_A_1'], p.POSITION_CONTROL, targetPos1, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_A_2'], p.POSITION_CONTROL, targetPos2, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_A_3'], p.POSITION_CONTROL, targetPos3, force=100)

    p.setJointMotorControl2(robot, jointNameToID['motor_B_1'], p.POSITION_CONTROL, targetPos1, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_B_2'], p.POSITION_CONTROL, targetPos2, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_B_3'], p.POSITION_CONTROL, targetPos3, force=100)

    p.setJointMotorControl2(robot, jointNameToID['motor_C_1'], p.POSITION_CONTROL, targetPos1, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_C_2'], p.POSITION_CONTROL, targetPos2, force=100)
    p.setJointMotorControl2(robot, jointNameToID['motor_C_3'], p.POSITION_CONTROL, targetPos3, force=100)

    p.stepSimulation()
