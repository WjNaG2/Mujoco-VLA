import argparse
import os
import mujoco
from mujoco.viewer import launch

DEFAULT_XML = """
<mujoco model='simple_scene'>
  <compiler angle='degree'/>
  <option gravity='0 0 -9.81'/>
  <worldbody>
    <geom type='plane' size='5 5 0.1' rgba='0.8 0.8 0.8 1'/>
    <body name='robot' pos='0 0 0.5'>
      <joint name='root' type='free'/>
      <geom type='sphere' size='0.1' rgba='0.2 0.5 0.8 1'/>
    </body>
  </worldbody>
</mujoco>
"""

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

def load_model(scene_path: str = None):
    if scene_path:
        if os.path.isabs(scene_path):
            return mujoco.MjModel.from_xml_path(scene_path)

        candidate_paths = [
            os.path.abspath(scene_path),
            os.path.abspath(os.path.join(PROJECT_ROOT, scene_path)),
            os.path.abspath(os.path.join(os.path.dirname(__file__), scene_path)),
        ]
        for path in candidate_paths:
            if os.path.exists(path):
                return mujoco.MjModel.from_xml_path(path)
        raise FileNotFoundError(
            f"Scene file not found: {scene_path}\nTried: {candidate_paths}"
        )

    default_scene = os.path.join(PROJECT_ROOT, "simple_end_effector_scene.xml")
    if os.path.exists(default_scene):
        return mujoco.MjModel.from_xml_path(default_scene)
    return mujoco.MjModel.from_xml_string(DEFAULT_XML)

def main():
    parser = argparse.ArgumentParser(description="Launch Mujoco scene")
    parser.add_argument(
        "--scene",
        default="",
        help="Path to MuJoCo XML scene file, relative to envs/ or absolute",
    )
    args = parser.parse_args()

    model = load_model(args.scene)
    data = mujoco.MjData(model)
    launch(model, data)

if __name__ == "__main__":
    main()