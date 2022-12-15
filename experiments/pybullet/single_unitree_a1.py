import numpy as np
import pybullet as p
import pybullet_data
import time


def inspect_robot(robot):
    print(f"Number of Joints in robot: {p.getNumJoints(robot)}")
    for i in range(p.getNumJoints(robot)):
        joint_info = p.getJointInfo(robot, i)
        print(
            f"Joint {i} ({joint_info[1]}) - type = {joint_info[2]}, "
            f"[{joint_info[8], joint_info[9]}]"
        )


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0.0, 0.0, -9.8)
p.setRealTimeSimulation(0)

print(f"hasNumpy = {p.isNumpyEnabled()}")

ground = p.loadURDF(
    "plane.urdf", [0.0, 0.0, 0.0], flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
)
a1 = p.loadURDF(
    "a1/a1.urdf",
    [0, 0, 1.0],
    flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
    useMaximalCoordinates=False,
)
inspect_robot(a1)


for setp in range(3000):
    target_positions = np.random.uniform(-0.8, 0.8, size=(4,))
    p.setJointMotorControlArray(
        a1, [3, 8, 13, 18], p.POSITION_CONTROL, targetPositions=target_positions
    )

    focus_position, _ = p.getBasePositionAndOrientation(a1)
    p.resetDebugVisualizerCamera(
        cameraDistance=3,
        cameraYaw=0,
        cameraPitch=-40,
        cameraTargetPosition=focus_position,
    )
    p.stepSimulation()
    time.sleep(0.01)
