from random import random
import numpy as np

from pydrake.multibody import inverse_kinematics
from pydrake.multibody.multibody_tree import BodyIndex
from pydrake.systems.framework import DiagramBuilder
from pydrake.util.eigen_geometry import Isometry3
from pydrake.math import RotationMatrix, RollPitchYaw


def GetEndEffectorWorldAlignedFrame():
    X_EEa = Isometry3.Identity()
    X_EEa.set_rotation(np.array([[0., 1, 0],
                                 [0, 0, 1],
                                 [1, 0, 0]]))
    return X_EEa


X_EEa = GetEndEffectorWorldAlignedFrame()
R_EEa = RotationMatrix(X_EEa.rotation())
p_EQ = GetEndEffectorWorldAlignedFrame().multiply(np.array([0, 0, 0.09]))

# orientation of end effector aligned frame
R_WEa_ref = RollPitchYaw(0, np.pi/180 * 180, 0).ToRotationMatrix()


class Util:
    def __init__(self, station):
        self.station = station
        builder = DiagramBuilder()
        builder.AddSystem(station)

        self.scene_graph = station.get_mutable_scene_graph()
        self.diagram = builder.Build()
        self.diagram_context = self.diagram.CreateDefaultContext()

        plant = station.get_mutable_multibody_plant()
        self.plant = plant
        self.tree = plant.tree()
        self.joint_ranges = None

        self.gripper_model = plant.GetModelInstanceByName("gripper")
        self.iiwa_model = plant.GetModelInstanceByName("iiwa")

        self.gripper_frame = plant.GetFrameByName("body", self.gripper_model)
        self.world_frame = plant.world_frame()

        self.sg_context = self.diagram.GetMutableSubsystemContext(self.scene_graph,
                                                                  self.diagram_context)
        self.context = self.diagram.GetMutableSubsystemContext(self.station,
                                                               self.diagram_context)
        self.object_context = self.station.GetMutableSubsystemContext(self.plant, self.context)

        bodies = self.get_bodies()
        self.bodies = bodies

        self.robot_bodies = self.bodies_from_models([self.iiwa_model, self.gripper_model])

    def get_bodies(self):
        return [self.tree.get_body(BodyIndex(i)) for i in range(self.plant.num_bodies())]

    def get_model_bodies(self, model):
        return [body for body in self.get_bodies() if body.model_instance() == model]

    def bodies_from_models(self, models):
        return {body for model in models for body in self.get_model_bodies(model)}

    def get_joint_ranges(self):
        joints = []
        for i in range(1, 8):
            joints.append(self.plant.GetJointByName("iiwa_joint_{}".format(i)))

        limits = []
        for joint in joints:
            upper = joint.upper_limits()
            lower = joint.lower_limits()
            assert upper.shape == (1, ) and lower.shape == (1, )
            limits.append((lower[0], upper[0]))

        return limits

    def random_q(self):
        if self.joint_ranges is None:
            self.joint_ranges = self.get_joint_ranges()

        return [self.sample(r) for r in self.joint_ranges]

    @staticmethod
    def sample(limits):
        """
        :param limits: tuple of (lower, upper)
        :return: random value in range
        """
        lower = limits[0]
        upper = limits[1]
        return (upper-lower)*random() + lower

    def IK(self, p, q_initial_guess=None, position_tolerance=0.005, is_goal=False):
        """
        :param p:
        :param q_initial_guess: np array of shape (1, num positions for plant)
        :param position_tolerance:
        :return:
        """
        if q_initial_guess is None:
            q_initial_guess = np.random.rand(1, self.plant.num_positions()) * 2*np.pi - np.pi

        plant = self.plant
        ik = inverse_kinematics.InverseKinematics(plant)
        q_variables = ik.q()

        # Orientation constraint
        if is_goal:
            theta_bound = 0.005 * np.pi
            ik.AddOrientationConstraint(
                frameAbar=self.world_frame, R_AbarA=R_WEa_ref,
                frameBbar=self.gripper_frame, R_BbarB=R_EEa,
                theta_bound=theta_bound
            )

        # Position constraint
        p_lower = p - position_tolerance
        p_upper = p + position_tolerance
        ik.AddPositionConstraint(
            frameB=self.gripper_frame, p_BQ=p_EQ,
            frameA=self.world_frame,
            p_AQ_lower=p_lower, p_AQ_upper=p_upper
        )

        prog = ik.prog()
        prog.SetInitialGuess(q_variables, q_initial_guess)
        prog.Solve()
        q_plant = prog.GetSolution(q_variables)

        return self.get_kuka_q(q_plant)

    def FK(self, q_kuka):
        self.set_iiwa_position(q_kuka)
        # equivalent way of getting pose
        # print(self.tree.EvalBodyPoseInWorld(self.object_context, self.gripper_frame.body()))
        return self.tree.CalcRelativeTransform(self.object_context, self.world_frame,
                                               self.gripper_frame)

    def set_wsg_position(self, q):
        """
        :param q: scalar which denotes how open the gripper is
        """
        self.station.SetWsgPosition(q, self.context)

    def get_kuka_q(self, q_plant):
        q_kuka = self.tree.GetPositionsFromArray(self.iiwa_model, q_plant)
        return q_kuka

    def get_colliding_bodies(self, min_penetration=0.0):
        query_object = self.scene_graph.get_query_output_port().Eval(self.sg_context)
        inspector = query_object.inspector()
        colliding_bodies = set()
        for penetration in query_object.ComputePointPairPenetration():
            if min_penetration < penetration.depth:
                body1, body2 = [self.plant.GetBodyFromFrameId(inspector.GetFrameId(geometry_id))
                                 for geometry_id in [penetration.id_A, penetration.id_B]]
                colliding_bodies.update([(body1, body2), (body2, body1)])

        return colliding_bodies

    def collision(self, q_kuka):
        self.set_iiwa_position(q_kuka)
        colliding_bodies = self.get_colliding_bodies()
        for bodies in colliding_bodies:
            for robot_body in self.robot_bodies:
                if robot_body in bodies:
                    return True
        return False

    def set_object_pose(self, object_name, object, pose):
        self.tree.SetFreeBodyPoseOrThrow(
            self.plant.GetBodyByName(object_name, object),
            pose,
            self.object_context
        )

    def set_iiwa_position(self, q):
        self.station.SetIiwaPosition(q, self.context)
