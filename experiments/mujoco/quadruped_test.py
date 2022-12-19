import mujoco
import numpy as np
import time

from multiprocessing import Manager, Pool


def worker(in_queue, out_queue):
    def _randomize_velocities(d):
        vel = np.random.randn(*d.qvel.shape)
        vel = vel * 0.3
        d.qvel[:] = vel

    model = mujoco.MjModel.from_xml_path(
        "/home/breakds/projects/other/mujoco_menagerie/unitree_a1/scene.xml"
    )

    data = mujoco.MjData(model)
    # https://github.com/deepmind/mujoco/blob/main/python/mujoco/renderer.py
    renderer = mujoco.Renderer(model, height=400, width=600)

    model.opt.timestep = 1 / 240.0
    render_timestep = 0.1
    mujoco.mj_resetData(model, data)  # Reset state and time.

    while True:
        done = in_queue.get()
        if done:
            break
        for i in range(int(render_timestep / model.opt.timestep)):
            mujoco.mj_step(model, data)
        renderer.update_scene(data)
        renderer.render()
        _randomize_velocities(data)  # take action
        out_queue.put(True)


m = Manager()
N = 40

pool = Pool(processes=N)
in_queue, out_queue = [], []
for i in range(N):
    in_queue.append(m.Queue())
    out_queue.append(m.Queue())
    pool.apply_async(func=worker, args=[in_queue[i], out_queue[i]])

t0 = time.time()
action_steps = 1000
for j in range(action_steps):
    for i in range(N):  # send
        in_queue[i].put(False)
    for i in range(N):  # retrieve
        out_queue[i].get()
action_step_time = time.time() - t0

# close workers
for i in range(N):
    in_queue[i].put(True)
pool.close()
pool.join()

print("Actual time per action step: ", action_step_time / action_steps)
