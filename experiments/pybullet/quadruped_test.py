import numpy as np
import pybullet as p
import pybullet_data
import pkgutil
import time
from PIL import Image


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
    def __init__(self, action_freq=10):
        self._ground = p.loadURDF(
            "plane.urdf", [0.0, 0.0, 0.0], flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        )
        self._robots = []
        self._steps_per_action = int(240 / action_freq)
        self._perf_step = Perf("Step")
        self._perf_reset = Perf("Reset")
        self._step_count = 0

    @property
    def n(self):
        return len(self._robots)

    def step(self, target_positions: np.ndarray):
        self._perf_step.start()
        # Currently, each action step include 24 physical steps and 1 rendering.
        # Pybullet by default runs at 240 Hz, i.e. each physical step = 1 / 240
        # second in logical time.
        for i in range(self.n):
            p.setJointMotorControlArray(
                self._robots[i],
                np.arange(p.getNumJoints(self._robots[i])),
                p.POSITION_CONTROL,
                targetPositions=target_positions[i],
            )

        for _ in range(self._steps_per_action):
            p.stepSimulation()

        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            [0.01, -0.01, 0.4],
            2.0,  # distance
            0.0,  # yaw
            -40.0,  # pitch
            0.0,  # roll
            2,
        )  # up axis index (2 means z direction)
        snapshot = p.getCameraImage(
            600,
            400,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            shadow=1,
            lightDirection=[1, 1, 1],
        )
        self._perf_step.stop()
        self._steps_count += 1
        Image.fromarray(snapshot[2][:, :, :3]).save(
            f"/home/breakds/tmp/bullet_capture/p{self._steps_count}.jpg"
        )

    def reset(self, n: int = 10):
        self._perf_reset.start()
        self._steps_count = 0

        positions = np.random.uniform(
            [-GROUND_RADIUS, -GROUND_RADIUS, 0.2],
            [GROUND_RADIUS, GROUND_RADIUS, 1.2],
            size=(n, 3),
        )

        for i in range(n):
            self._robots.append(
                p.loadURDF(
                    "a1/a1.urdf",
                    positions[i],
                    flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
                )
            )
        self._perf_reset.stop()

    def run(self, steps: int = 100):
        for _ in range(steps):
            target_positions = np.random.uniform(
                -0.2, 0.5, size=(self.n, p.getNumJoints(self._robots[0]))
            )
            self.step(target_positions)
            if not HEADLESS:
                time.sleep(0.1)

    def report(self):
        self._perf_reset.report()
        self._perf_step.report()


env = QuadrupedEnvironment()
env.reset(1)
env.run(1000)
env.report()
