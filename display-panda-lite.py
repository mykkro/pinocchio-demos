# Display just the robot in Mestcat.

import pinocchio as pin
import numpy as np
import sys
import os
from os.path import dirname, join, abspath

from pinocchio.visualize import MeshcatVisualizer

pinocchio_model_dir = dirname(str(abspath(__file__)))
model_path = join(pinocchio_model_dir, "example-robot-data/robots")
mesh_dir = pinocchio_model_dir
urdf_filename = "panda.urdf"
urdf_model_path = join(join(model_path, "panda_description/urdf"), urdf_filename)

model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
)

viz = MeshcatVisualizer(model, collision_model, visual_model)

# Start a new MeshCat server and client.
# Note: the server can also be started separately using the "meshcat-server" command in a terminal:
# this enables the server to remain active after the current script ends.
#
# Option open=True pens the visualizer.
# Note: the visualizer can also be opened seperately by visiting the provided URL.
try:
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)

# Load the robot in the viewer.
viz.loadViewerModel()

# Display a robot configuration.
q0 = pin.neutral(model)
viz.display(q0)
viz.displayCollisions(False)
viz.displayVisuals(True)
