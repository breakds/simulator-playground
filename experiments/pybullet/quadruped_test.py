import numpy as np
import pybullet as p
import pybullet_data
import pkgutil
import time

HEADLESS = True

if HEADLESS:
    p.connect(p.DIRECT)
    egl = pkgutil.get_loader("eglRenderer")
    plugin = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
else:
    p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0.0, 0.0, -9.8)

GROUND_RADIUS = 2.0


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


projection_matrix = np.array(
    [
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
)


class QuadrupedEnvironment(object):
    def __init__(self):
        self._ground = p.loadURDF(
            "plane.urdf", [0.0, 0.0, 0.0], flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        )
        self._robots = []

        self._perf_physics = Perf("Physics")
        self._perf_render = Perf("Render")
        self._perf_reset = Perf("Reset")

    def reset(self, n: int = 10):
        positions = np.random.uniform(
            [-GROUND_RADIUS, -GROUND_RADIUS, 0.2],
            [GROUND_RADIUS, GROUND_RADIUS, 1.2],
            size=(n, 3),
        )
        self._perf_reset.start()
        for i in range(n):
            self._robots.append(
                p.loadURDF(
                    "a1/a1.urdf",
                    positions[i],
                    flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
                )
            )
        self._perf_reset.stop()

    def run(self, steps: int = 1000):
        n = len(self._robots)
        for step in range(steps):
            target_positions = np.random.uniform(-0.8, 1.5, size=(n, 4))
            for i in range(n):
                p.setJointMotorControlArray(
                    self._robots[i],
                    [3, 8, 13, 18],
                    p.POSITION_CONTROL,
                    targetPositions=target_positions[i],
                )

            self._perf_physics.start()
            p.stepSimulation()
            self._perf_physics.stop()

            focus_position, _ = p.getBasePositionAndOrientation(self._robots[0])

            view_matrix = p.computeViewMatrixFromYawPitchRoll(
                focus_position,
                2.0,  # distance
                0.0,  # yaw
                -40.0,  # pitch
                0.0,  # roll
                2,
            )  # up axis index (2 means z direction)

            self._perf_render.start()
            img = p.getCameraImage(
                600,
                400,
                viewMatrix=view_matrix,
                projectionMatrix=projection_matrix,
                shadow=1,
                lightDirection=[1, 1, 1],
            )
            self._perf_render.stop()

    def report(self):
        self._perf_physics.report()
        self._perf_reset.report()
        self._perf_render.report()


env = QuadrupedEnvironment()
env.reset(20)
env.run(1000)
env.report()
