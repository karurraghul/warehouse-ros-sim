#!/usr/bin/env python3
"""One-off dev script: generate ArUco marker PNGs and Gazebo model folders
under src/robo_ai/models/markers/.

Not installed or run at simulation time - run manually whenever the marker
set needs to change:

    python3 src/robo_ai/scripts/generate_markers.py
"""
import os

import cv2

MARKERS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "models", "markers",
)
IMAGE_SIZE_PX = 400
PLATE_SIZE_M = 0.15
PLATE_THICKNESS_M = 0.02

ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

# (model_name, aruco_id)
MARKERS = [
    ("marker_aruco_0", 0),
    ("marker_aruco_1", 1),
    ("marker_aruco_2", 2),
    ("marker_aruco_3", 3),
]

MATERIAL_TEMPLATE = """material {material_name}
{{
  technique
  {{
    pass
    {{
      lighting off
      texture_unit
      {{
        texture {texture_file}
      }}
    }}
  }}
}}
"""

MODEL_CONFIG_TEMPLATE = """<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>{description}</description>
</model>
"""

MODEL_SDF_TEMPLATE = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{model_name}'>
    <static>true</static>
    <link name='link'>
      <visual name='visual'>
        <geometry>
          <box>
            <size>{thickness} {size} {size}</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>model://{model_name}/materials/scripts</uri>
            <uri>model://{model_name}/materials/textures</uri>
            <name>{material_name}</name>
          </script>
        </material>
      </visual>
      <collision name='collision'>
        <geometry>
          <box>
            <size>{thickness} {size} {size}</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""


def render_aruco_png(marker_id, out_path):
    img = cv2.aruco.drawMarker(ARUCO_DICT, marker_id, IMAGE_SIZE_PX)
    pad = IMAGE_SIZE_PX // 8
    padded = cv2.copyMakeBorder(
        img, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(out_path, padded)


def write_model(model_name, texture_filename, description):
    model_dir = os.path.join(MARKERS_DIR, model_name)
    scripts_dir = os.path.join(model_dir, "materials", "scripts")
    textures_dir = os.path.join(model_dir, "materials", "textures")
    os.makedirs(scripts_dir, exist_ok=True)
    os.makedirs(textures_dir, exist_ok=True)

    material_name = f"Marker/{model_name}"

    with open(os.path.join(scripts_dir, "marker.material"), "w") as f:
        f.write(MATERIAL_TEMPLATE.format(
            material_name=material_name, texture_file=texture_filename))

    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write(MODEL_CONFIG_TEMPLATE.format(
            model_name=model_name, description=description))

    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(MODEL_SDF_TEMPLATE.format(
            model_name=model_name, material_name=material_name,
            size=PLATE_SIZE_M, thickness=PLATE_THICKNESS_M))

    return textures_dir


def main():
    for model_name, marker_id in MARKERS:
        texture_filename = f"{model_name}.png"
        description = f"ArUco marker plate, DICT_4X4_50 id={marker_id}"
        textures_dir = write_model(model_name, texture_filename, description)
        texture_path = os.path.join(textures_dir, texture_filename)
        render_aruco_png(marker_id, texture_path)
        print(f"Generated {model_name} (aruco={marker_id}) -> {texture_path}")


if __name__ == "__main__":
    main()
