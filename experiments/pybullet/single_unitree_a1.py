import pkgutil
import numpy as np
import pybullet as p
import pybullet_data
import time
from PIL import Image


class Perf(object):
    def __init__(self, name):
        self._name = name
        self._n = 0
        self._t = 0.0
        self._ts = 0.0

    def start(self):
        self._ts = time.perf_counter()

    def stop(self):
        end = time.perf_counter()
        self._t += end - self._ts
        self._n += 1

    def report(self):
        print(
            f"{self._name}: N = {self._n}, Total = {self._t:.3f}, "
            f"Average = {self._t / self._n:.10f}"
        )


def inspect_robot(robot):
    print(f"Number of Joints in robot: {p.getNumJoints(robot)}")
    for i in range(p.getNumJoints(robot)):
        joint_info = p.getJointInfo(robot, i)
        print(
            f"Joint {i} ({joint_info[1]}) - type = {joint_info[2]}, "
            f"[{joint_info[8], joint_info[9]}]"
        )


p.connect(p.DIRECT)
egl = pkgutil.get_loader("eglRenderer")
plugin = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)


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

projection_matrix = [
    1.0825318098068237,
    0.0,
    0.0,
    0.0,
    0.0,
    1.732050895690918,
    0.0,
    0.0,
    0.0,
    0.0,
    -1.0002000331878662,
    -1.0,
    0.0,
    0.0,
    -0.020002000033855438,
    0.0,
]

perf_physics = Perf("Physics")
perf_render = Perf("Render")

img = None
camera_distance = 1.0

for step in range(1000):
    target_positions = np.random.uniform(-0.8, 0.8, size=(4,))
    p.setJointMotorControlArray(
        a1, [3, 8, 13, 18], p.POSITION_CONTROL, targetPositions=target_positions
    )

    perf_physics.start()
    p.stepSimulation()
    perf_physics.stop()

    focus_position, _ = p.getBasePositionAndOrientation(a1)
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        focus_position,
        camera_distance,  # distance
        0.0,  # yaw
        -40.0,  # pitch
        0.0,  # roll
        2,
    )  # up axis index (2 means z direction)

    perf_render.start()
    img = p.getCameraImage(
        600,
        400,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        shadow=1,
        lightDirection=[1, 1, 1],
    )
    perf_render.stop()

    im = Image.fromarray(img[2][:, :, :3])
    im.save(f"/home/breakds/tmp/bullet_images_{step}.jpg")

print(f"Rendering image shape: {img[2].shape}")
perf_physics.report()
perf_render.report()

p.unloadPlugin(plugin)
