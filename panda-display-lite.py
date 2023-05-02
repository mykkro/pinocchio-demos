# Display just the robot in Mestcat.

import pinocchio as pin
import numpy as np
import sys
import os
from os.path import dirname, join, abspath

from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as g

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

# add a cube
cube = g.Box([0.1, 0.1, 0.1])  # Create a cube with dimensions 0.1 x 0.1 x 0.1
# viz.static_objects.append(cube)
# AttributeError: 'Box' object has no attribute 'name'

#tr = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
#viz.viewer["cube1"].set_object(cube, None, tr)

# ??? How to position the cube properly in Meshcat viewer?
viz.viewer["cube1"].set_object(cube)

# Display a robot configuration.
q0 = pin.neutral(model)


# default position
q = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

q = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.57, 1.57/2, 0.03, 0.03]

# q = [0.22263882716278535, -0.2602131706709166, 1.716949196152973, -0.9272843987410893, 0.11406977599944451, -0.17540102034115665, 0.3104290774365284, -0.24181092739962473, 0.2582010853268569, -0.46318163428088455, 0.16229416915990053, -0.33703098132338355, 0.7471981871715188, -0.7367815786401621, 0.0, 0.0]

# computed by FK/IK by panda-kinematics, first 7 items are zero - these are parameters for root_joint
# q = [0, 0, 0, 0, 0, 0, 0, -0.24181092739962473, 0.2582010853268569, -0.46318163428088455, 0.16229416915990053, -0.33703098132338355, 0.7471981871715188, -0.7367815786401621, 0.0, 0.0]

# slightly different pose...
# q =  [0.5342580545452823, -0.5145906177381115, 1.1647532270471987, -0.9057774586050459, 0.1695076672234332, -0.20278650964189282, 0.3312279845404259, -0.2553023943757199, 0.30616741937461117, -0.5059698113978905, 0.2301444443140104, -0.311857011651677, 0.778659817716229, -0.6237281258210072, 0.0, 0.0]

q0 = np.array( q )
# q0[0:7] = 0

viz.display(q0)
viz.displayCollisions(False)
viz.displayVisuals(True)
