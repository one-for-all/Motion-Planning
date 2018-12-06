import numpy as np

from pydrake.multibody import inverse_kinematics
from pydrake.util.eigen_geometry import Isometry3
from pydrake.trajectories import PiecewisePolynomial
from pydrake.examples.manipulation_station import ManipulationStation
from pydrake.math import RotationMatrix, RollPitchYaw


p_WQ_home = np.array([0.5, 0, 0.41])

station = ManipulationStation()
station.Finalize()

plant = station.get_mutable_multibody_plant()
tree = plant.tree()

iiwa_model = plant.GetModelInstanceByName("iiwa")
gripper_model = plant.GetModelInstanceByName("gripper")

world_frame = plant.world_frame()
gripper_frame = plant.GetFrameByName("body", gripper_model)

# orientation of end effector aligned frame
R_WEa_ref = RollPitchYaw(0, np.pi/180 * 135, 0).ToRotationMatrix()


'''
Ea, or End_Effector_world_aligned is a frame fixed w.r.t the gripper.
Ea has the same origin as the end effector's body frame, but
its axes are aligned with those of the world frame when the system
has zero state, i.e. the robot is upright with all joint angles
equal to zero.
This frame is defined so that it is convenient to define end effector orientation
relative to the world frame using RollPitchYaw.
'''
def GetEndEffectorWorldAlignedFrame():
    X_EEa = Isometry3.Identity()
    X_EEa.set_rotation(np.array([[0., 1, 0],
                                 [0, 0, 1],
                                 [1, 0, 0]]))
    return X_EEa

X_EEa = GetEndEffectorWorldAlignedFrame()
R_EEa = RotationMatrix(X_EEa.rotation())
p_EQ = GetEndEffectorWorldAlignedFrame().multiply(np.array([0, 0, 0.09]))

def GeneratePickupBrickPlansByTrajectory(p_goal, is_printing=True):
    plan_list, gripper_setpoint_list, q_final_full = \
        GeneratePickupBrickPlans(p_goal,is_printing=is_printing)
    return plan_list, gripper_setpoint_list


def GeneratePickupBrickPlans(p_goal, is_printing=True):

    def InterpolateStraightLine(p_WQ_start, p_WQ_end, num_knot_points, i):
        return (p_WQ_end - p_WQ_start) / num_knot_points * (i+1) + p_WQ_start

    num_knot_points = 10

    q_home_full = GetHomeConfiguration(is_printing)
    p_WQ_start = p_WQ_home
    p_WQ_end = p_goal
    qtraj_move_to_box, q_knots_full = InverseKinPointwise(
        p_WQ_start, p_WQ_end, duration=5.0,
        num_knot_points=num_knot_points, q_initial_guess=q_home_full,
        InterpolatePosition=InterpolateStraightLine,
        position_tolerance=0.001,
        is_printing=is_printing
    )

    # close gripper
    q_knots = np.zeros((2, 7))
    q_knots[0] = qtraj_move_to_box.value(qtraj_move_to_box.end_time()).squeeze()
    qtraj_close_gripper = PiecewisePolynomial.ZeroOrderHold([0, 1], q_knots.T)

    q_traj_list = [qtraj_move_to_box,
                   qtraj_close_gripper]

    plan_list = []
    for q_traj in q_traj_list:
        plan_list.append(JointSpacePlan(q_traj))

    gripper_setpoint_list = [0.1, 0.005]

    q_final_full = q_knots_full[-1]
    return plan_list, gripper_setpoint_list, q_final_full

# Helper functions
# ====================================================
def GetHomeConfiguration(is_printing=True):
    ik_scene = inverse_kinematics.InverseKinematics(plant)

    theta_bound = 0.005 * np.pi

    ik_scene.AddOrientationConstraint(
        frameAbar=world_frame, R_AbarA=R_WEa_ref,
        frameBbar=gripper_frame, R_BbarB=R_EEa,
        theta_bound=theta_bound
    )

    p_WQ0 = p_WQ_home
    p_WQ_lower = p_WQ0 - 0.005
    p_WQ_upper = p_WQ0 + 0.005
    ik_scene.AddPositionConstraint(
        frameB=gripper_frame, p_BQ=p_EQ,
        frameA=world_frame,
        p_AQ_lower=p_WQ_lower, p_AQ_upper=p_WQ_upper
    )

    prog = ik_scene.prog()
    prog.SetInitialGuess(ik_scene.q(), np.zeros(plant.num_positions()))
    result = prog.Solve()
    if is_printing:
        print(result)
    return prog.GetSolution(ik_scene.q())


def InverseKinPointwise(p_WQ_start, p_WQ_end, duration,
                        num_knot_points,
                        q_initial_guess,
                        InterpolatePosition=None,
                        position_tolerance=0.005,
                        theta_bound=0.005 * np.pi,
                        is_printing=True):
    q_knots = np.zeros((num_knot_points + 1, plant.num_positions()))
    q_knots[0] = q_initial_guess

    for i in range(num_knot_points):
        ik = inverse_kinematics.InverseKinematics(plant)
        q_variables = ik.q()

        # Orientation constraint
        ik.AddOrientationConstraint(
            frameAbar=world_frame, R_AbarA=R_WEa_ref,
            frameBbar=gripper_frame, R_BbarB=R_EEa,
            theta_bound=theta_bound
        )

        # Position constraint
        p_WQ = InterpolatePosition(p_WQ_start, p_WQ_end, num_knot_points, i)
        p_WQ_lower = p_WQ - position_tolerance
        p_WQ_upper = p_WQ + position_tolerance
        ik.AddPositionConstraint(
            frameB=gripper_frame, p_BQ=p_EQ,
            frameA=world_frame,
            p_AQ_lower=p_WQ_lower, p_AQ_upper=p_WQ_upper
        )

        prog = ik.prog()
        prog.SetInitialGuess(q_variables, q_knots[i])
        result = prog.Solve()
        if is_printing:
            print(i, ": ", result)
        q_knots[i+1] = prog.GetSolution(q_variables)

    t_knots = np.linspace(0, duration, num_knot_points+1)
    q_knots_kuka = GetKukaQKnots(q_knots)
    qtraj = PiecewisePolynomial.Cubic(
        t_knots, q_knots_kuka.T, np.zeros(7), np.zeros(7)
    )

    return qtraj, q_knots


def GetKukaQKnots(q_knots):
    if len(q_knots.shape) == 1:
        q_knots.resize(1, q_knots.size)
    n = q_knots.shape[0]
    q_knots_kuka = np.zeros((n, 7))
    for i, q_knot in enumerate(q_knots):
        q_knots_kuka[i] = tree.GetPositionsFromArray(iiwa_model, q_knot)

    return q_knots_kuka


# Plan Utils
# =======================================
import hashlib
plan_type_strings = [
    "JointSpacePlan",
    "JointSpacePlanRelative",
    "IiwaTaskSpacePlan",
    "PlanarTaskSpacePlan",
    "PlanarHybridPositionForcePlan",
    "OpenLeftDoorPositionPlan",
    "OpenLeftDoorImpedancePlan",
    "JointSpacePlanGoToTarget",
]
PlanTypes = dict()
for plan_types_string in plan_type_strings:
    PlanTypes[plan_types_string] = hashlib.sha1(plan_types_string).hexdigest()


class PlanBase:
    def __init__(self, type=None, trajectory=None):
        self.type = type
        self.traj = trajectory
        self.traj_d = None
        self.duration = None
        if trajectory is not None:
            self.traj_d = trajectory.derivative(1)
            self.duration = trajectory.end_time()

        self.start_time = None

    def get_duration(self):
        return self.duration

    def set_start_time(self, time):
        self.start_time = time


class JointSpacePlan(PlanBase):
    def __init__(self, trajectory=None):
        PlanBase.__init__(self,
                          type=PlanTypes["JointSpacePlan"],
                          trajectory=trajectory)