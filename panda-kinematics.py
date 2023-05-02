import pinocchio as pin
import numpy as np
import sys
import os
from os.path import dirname, join, abspath
from numpy.linalg import norm, solve
import pandas as pd


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
    desired_pose = pin.SE3(np.array([[1,0,0], [0,1,0], [0,0,1]]), np.array([1.2, 0.0, 0.5]))

    # What exactly is SE(3) and SO(3)?
    # See also: https://natanaso.github.io/ece276a2020/ref/ECE276A_12_SO3_SE3.pdf
    # SO(3) - [for describing orientation] Special Orthogonal Group SO(3) : set of 3x3 matrices R: det(R)=1, Rt*R = I (hence Rt = R-1)
    # SE(3) - [for describing pose] Special Euclidean Group SE(3) : T = set of 4x4 matrices: [[R p] [0 1]], R is in SO(3), P is 3-vector
    # SO(3), SE(3) are matrix Lie groups

    # initial configuration
    # default (neutral) config
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
    outq = []
    while True:
        # placement of the EE joint
        plac = data.oMi[EE_JOINT_ID]
        print(f"Joint placement for joint {model.joints[EE_JOINT_ID]}:", plac)

        # Update the joint placements according to the current joint configuration
        pin.forwardKinematics(model, data, q)
        plac = data.oMi[EE_JOINT_ID]
        dMi = desired_pose.actInv(plac)  # transformation between the desired pose and the current one
        err = pin.log(dMi).vector # an error - compute error in SO(3) as a six-dim vector
        if norm(err) < eps:
            success = True
            break
        if i >= IT_MAX:
            success = False
            break

        J = pin.computeJointJacobian(model, data, q, EE_JOINT_ID)
        v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))  # damped pseudoinverse v = -Jt(J*Jt + lambdaI)-1*e
        q = pin.integrate(model, q, v*DT)  # add the obtained tangent vector to the current configuration

        outq.append(q.tolist())

        print(f"Iteration {i}", q)
        i += 1

    if success:
        print("Convergence achieved!")
    else:
        print("Did not converge!")

    print("Result:", q.flatten().tolist())
    print("Error:", err.T)

    output_table_path = "target/table.csv"
    with open(output_table_path, "w", encoding="utf-8") as outfile:
        txt = "\t".join([f"q{i}" for i in range(16)])
        outfile.write(txt + "\n")
        for q in outq:
            txt = "\t".join([str(it) for it in q])
            outfile.write(txt + "\n")
