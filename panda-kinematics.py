import pinocchio as pin
import numpy as np
import sys
import os
from os.path import dirname, join, abspath
from numpy.linalg import norm, solve


if __name__ == "__main__":

    print("Testing Panda Kinematics...")

    print("Loading Panda model...")
    urdf_model_path = "example-robot-data/robots/panda_description/urdf/panda.urdf"
    mesh_dir = "."
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
    )
    print("Model loaded!", model)

    print("  Model name:", model.name)
    print("  Joint names:", model.names)
    print("  Joint models:", model.joints)
    print("  Link inertias:", model.inertias)
    print("  Joint placements:", model.jointPlacements)
    # len(model.jointPlacements) == 11
    print("  Frames:", model.frames)
    print("  No. of position variables (nq):", model.nq)
    print("  No. of velocity variables (nv):", model.nv)

    # Nb joints = 11 (nq=16,nv=15) 
    # ?? meaning of these parameters (nq, np?)

    # create a data structure for algorithm buffering
    data = model.createData()
    print("Model data created!", data)

    EE_JOINT_ID = 8 # index of end effector joint

    #  coordinate transformations in the 3D Euclidean space (aka SE3)
    #  params: R (Rotation), p
    desired_pose = pin.SE3(np.eye(3), np.array([0.8, 0.3, 1.]))

    # initial configuration
    q = pin.neutral(model)

    print("Desired pose:", desired_pose)
    print("q:", q)

    eps = 1e-4     # desired position precision
    IT_MAX = 1000  # maximum number of iterations
    DT = 1e-1      # time step (defining convergence rate)
    damp = 1e-12   # fixed damping factor for the pseudoinversion

    for i, joint_name in enumerate(model.joints):
        print(f"Joint: {i} name: {joint_name}")

    # BEGIN iteration
    i = 0
    while True:
        # placement of the EE joint
        plac = data.oMi[EE_JOINT_ID]
        print(f"Joint placement for joint {model.joints[EE_JOINT_ID]}:", plac)

        # Update the joint placements according to the current joint configuration
        pin.forwardKinematics(model, data, q)
        plac = data.oMi[EE_JOINT_ID]
        dMi = desired_pose.actInv(plac)
        err = pin.log(dMi).vector
        if norm(err) < eps:
            success = True
            break
        if i >= IT_MAX:
            success = False
            break

        J = pin.computeJointJacobian(model, data, q, EE_JOINT_ID)
        v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
        q = pin.integrate(model, q, v*DT)

        print(f"Iteration {i}", q)
        i += 1

    if success:
        print("Convergence achieved!")
    else:
        print("Did not converge!")

    print("Result:", q.flatten().tolist())
    print("Error:", err.T)
