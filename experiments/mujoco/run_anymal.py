import numpy as np
import mujoco
import mujoco_menagerie
from PIL import Image


def inspect_model(model: mujoco.MjModel):
    print(f"Number of actuators: {model.nu}")
    print(f"Number of joints: {model.njnt}")
    print(f"Number of Cameras: {model.ncam}")
    print(f"Number of Bodies: {model.nbody}")
    print(f"Number of Sensors: {model.nsensor}")
    print(model.actuator_ctrllimited)
    print(model.actuator_forcelimited)
    print(model.actuator_ctrlrange)


def main():
    robot = mujoco_menagerie.query("anybotics", "anymal_b")
    assert robot is not None
    model = mujoco.MjModel.from_xml_path(str(robot.path / "scene.xml"))
    inspect_model(model)

    # Initialize the data object, with default configuration.
    data = mujoco.MjData(model)
    model.opt.timestep = 1 / 240.0
    renderer = mujoco.Renderer(model, height=400, width=600)
    mujoco.mj_step(model, data)
    print(data.time)
    renderer.update_scene(data)
    img = renderer.render()
    Image.fromarray(img).save("/home/breakds/tmp/anymal_b.jpg")


if __name__ == "__main__":
    main()
