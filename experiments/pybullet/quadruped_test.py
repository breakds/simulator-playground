import numpy as np
import pybullet as p
import pybullet_data
from pybullet_utils import bullet_client as bc
import pkgutil
import time
from pathlib import Path
from PIL import Image


NUM_ENVS = 20
NUM_AGENTS = 1
ROBOT = "anymal_b"
HEADLESS = True
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
        if HEADLESS:
            self._client = bc.BulletClient(connection_mode=p.DIRECT)
            egl = pkgutil.get_loader("eglRenderer")
            self._plugin = self._client.loadPlugin(
                egl.get_filename(), "_eglRendererPlugin"
            )
            self._client.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
            self._client.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        else:
            self._client.connect(p.GUI)
        self._client.setAdditionalSearchPath(pybullet_data.getDataPath())
        self._client.setGravity(0.0, 0.0, -9.8)

        self._ground = self._client.loadURDF(
            "plane.urdf", [0.0, 0.0, 0.0], flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        )
        self._robots = []
        self._steps_per_action = int(240 / action_freq)
        self._perf_step = Perf("Step")
        self._perf_reset = Perf("Reset")
        self._step_count = 0

    def __del__(self):
        if HEADLESS:
            self._client.unloadPlugin(self._plugin)
            self._client.disconnect()

    @property
    def n(self):
        return len(self._robots)

    def step(self, target_positions: np.ndarray):
        self._perf_step.start()
        # Currently, each action step include 24 physical steps and 1 rendering.
        # Pybullet by default runs at 240 Hz, i.e. each physical step = 1 / 240
        # second in logical time.
        for i in range(self.n):
            self._client.setJointMotorControlArray(
                self._robots[i],
                np.arange(self._client.getNumJoints(self._robots[i])),
                p.POSITION_CONTROL,
                targetPositions=target_positions[i],
            )

        for _ in range(self._steps_per_action):
            self._client.stepSimulation()

        for i in range(self.n):
            view_matrix = self._client.computeViewMatrixFromYawPitchRoll(
                p.getBasePositionAndOrientation(self._robots[i])[0],
                2.0,  # distance
                0.0,  # yaw
                -40.0,  # pitch
                0.0,  # roll
                2,
            )  # up axis index (2 means z direction)

            self._client.getCameraImage(
                600,
                400,
                viewMatrix=view_matrix,
                projectionMatrix=projection_matrix,
                shadow=1,
                lightDirection=[1, 1, 1],
            )

        self._perf_step.stop()
        self._steps_count += 1

    def reset(self, n: int = 10):
        self._perf_reset.start()
        self._steps_count = 0

        positions = np.random.uniform(
            [-GROUND_RADIUS, -GROUND_RADIUS, 0.2],
            [GROUND_RADIUS, GROUND_RADIUS, 1.2],
            size=(n, 3),
        )

        if ROBOT != "a1":
            self._client.setAdditionalSearchPath(
                f"{Path(__file__).parent.parent.parent}/resources"
            )

        for i in range(n):
            if ROBOT == "a1":
                self._robots.append(
                    self._client.loadURDF(
                        "a1/a1.urdf",
                        positions[i],
                        flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
                    )
                )
            elif ROBOT == "anymal_b":
                self._robots.append(
                    self._client.loadURDF(
                        "anymal_b/urdf/anymal_b.urdf",
                        positions[i],
                        flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
                    )
                )
        self._perf_reset.stop()

    def run(self, steps: int = 100):
        for _ in range(steps):
            target_positions = np.random.uniform(
                -0.2, 0.5, size=(self.n, self._client.getNumJoints(self._robots[0]))
            )
            self.step(target_positions)
            if not HEADLESS:
                time.sleep(0.1)

    def report(self):
        self._perf_reset.report()
        self._perf_step.report()


def run_single_env(signal):
    env = QuadrupedEnvironment()
    env.reset(NUM_AGENTS)
    for _ in range(1000):
        signal.wait()
        target_positions = np.random.uniform(
            -0.2, 0.5, size=(env.n, env._client.getNumJoints(env._robots[0]))
        )
        env.step(target_positions)
    return env._perf_step


if __name__ == "__main__":
    from multiprocessing import Pool, Manager

    manager = Manager()

    with Pool(NUM_ENVS) as pool:
        signal = manager.Event()
        results = pool.map_async(run_single_env, [signal] * NUM_ENVS)
        for _ in range(1000):
            signal.set()
        n = 0
        t = 0.0
        for result in results.get():
            n += result._n
            t += result._t
        print(f"Average {t / n} seconds per step. n = {n}.")
