from random import random
from math import ceil
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.examples.manipulation_station import ManipulationStation, IiwaCollisionModel
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
from pydrake.common.eigen_geometry import Isometry3
from pydrake.trajectories import PiecewisePolynomial

from util import Util

from pickup_brick import JointSpacePlan


class TreeNode:
    def __init__(self, value, parent=None):
        self.value = value
        self.parent = parent
        self.children = []


class RRT:
    def __init__(self, root, cspace):
        self.root = root
        self.cspace = cspace
        self.size = 1
        self.max_recursion = 5000

    def add_configuration(self, parent_node, child_value):
        child_node = TreeNode(child_value, parent_node)
        parent_node.children.append(child_node)
        self.size += 1
        return child_node

    def nearest(self, configuration):
        assert self.cspace.valid_configuration(configuration)
        def recur(node, depth=0):
            closest, distance = node, self.cspace.distance(node.value, configuration)
            if depth < self.max_recursion:
                for child in node.children:
                    child_closest, child_distance = recur(child, depth+1)
                    if child_distance < distance:
                        closest = child_closest
                        distance = child_distance
            # if depth >= self.max_recursion:
            #     print("reached max recursion depth")
            # else:
            #     print("success finding nearest")
            return closest, distance
        return recur(self.root)[0]




class Range:
    def __init__(self, low, high):
        self.low = low
        self.high = high

    def sample(self):
        return (self.high - self.low)*random() + self.low

    def contains(self, x):
        return self.low <= x <= self.high

    def difference(self, one, two):
        return two-one

    def in_range(self, value):
        if self.contains(value):
            return value
        else:
            return None


class ConfigurationSpace:
    def __init__(self, ranges):
        """
        :param cspace_ranges: a list of tuples
        """
        self.cspace_ranges = [Range(j[0], j[1]) for j in ranges]

    def sample(self):
        return [r.sample() for r in self.cspace_ranges]

    def valid_configuration(self, config):
        return len(config) == len(self.cspace_ranges) and \
            all([self.cspace_ranges[i].contains(config[i]) \
                 for i in range(len(self.cspace_ranges))])

    def distance(self, one, two):
        return np.linalg.norm(np.array(one) - np.array(two))

    def path(self, start, end):
        assert self.valid_configuration(start) and self.valid_configuration(end)
        diffs = [self.cspace_ranges[i].difference(start[i], end[i]) \
                 for i in range(len(self.cspace_ranges))]
        max_diff = 0.01
        samples = max([int(ceil(abs(diff)/max_diff)) \
                       for diff in diffs])
        samples = max(2, samples)
        linear_interpolation = [diff/(samples - 1.0) for diff in diffs]
        path = [start]
        for s in range(1, samples-1):
            sample = [self.cspace_ranges[i].in_range(start[i] + s*linear_interpolation[i]) \
                      for i in range(len(self.cspace_ranges))]
            path.append(sample)
        return path + [end]


class MotionPlanning:
    def __init__(self, q_initial, p_goal,
                 object_file_paths=None,
                 object_base_link_names=None,
                 X_WObject_list=None):
        self.q_initial=q_initial
        self.p_goal=p_goal

        self.station = ManipulationStation(collision_model=IiwaCollisionModel.kBoxCollision)
        self.station.AddCupboard()
        self.plant = self.station.get_mutable_multibody_plant()

        self.objects = []
        if object_file_paths is not None:
            for i, file_path in enumerate(object_file_paths):
                self.objects.append(AddModelFromSdfFile(
                    file_name=file_path,
                    model_name="object_{}".format(i),
                    plant=self.plant,
                    scene_graph=self.station.get_mutable_scene_graph()
                ))

        self.station.Finalize()

        # Object base link names for reference
        self.object_base_link_names = object_base_link_names

        # Initial pose of the object
        self.X_WObject_list = X_WObject_list

    def rrt_plan(self, max_iters=1000, goal_sample_prob=0.05, position_tolerance=0.09):
        util = Util(self.station)
        joint_ranges = util.get_joint_ranges()
        cspace = ConfigurationSpace(joint_ranges)
        rrt = RRT(TreeNode(self.q_initial), cspace)

        # set initial pose of the objects
        if self.object_base_link_names is not None:
            for link_name, object, X_WObject in zip(self.object_base_link_names,
                                                    self.objects,
                                                    self.X_WObject_list):
                util.set_object_pose(link_name, object, X_WObject)

        util.set_iiwa_position(self.q_initial)
        util.set_wsg_position(0.1)

        def sample_config():
            prob = random()
            if prob < goal_sample_prob:
                q_goal = None
                it = 0
                while q_goal is None:
                    print("goal sampling iter: ", it)
                    it += 1
                    q_goal_sample = util.IK(self.p_goal, is_goal=False)
                    if not util.collision(q_goal_sample):
                        q_goal = q_goal_sample
                return q_goal
            else:
                q = None
                it = 0
                while q is None:
                    print("non goal sampling iter: ", it)
                    it += 1
                    q_sample = util.random_q()
                    if not util.collision(q_sample):
                        q = q_sample
                return q

        def contruct_path(node):
            path = []
            while node is not None:
                path.append(node.value)
                node = node.parent
            return path[::-1]

        def bfs(rrt, target):
            queue = [rrt.root]
            while len(queue) > 0:
                node = queue.pop(0)
                if np.all(node.value == target):
                    return contruct_path(node)
                for child in node.children:
                    queue.append(child)
            raise Exception("Did not find a path when it is supposed to")

        for i in range(max_iters):
            print("iter: ", i)
            sample = sample_config()
            neighbor = rrt.nearest(sample)
            path = safe_path(cspace, neighbor.value, sample, util)
            if len(path) > 1:
                child_node = rrt.add_configuration(neighbor, path[0])
                for p in path[1:-1:5] + [path[-1]]:
                    child_node = rrt.add_configuration(child_node, p)
                # rrt.add_configuration(neighbor, path[-1])
                q_end = path[-1]
                translation = util.FK(q_end).translation()
                if within_tol(translation, self.p_goal, position_tolerance):
                    path = np.array(bfs(rrt, q_end)[1:])

                    duration = 5
                    num_knot_points = path.shape[0]
                    t_knots = np.linspace(0, duration, num_knot_points)
                    qtraj = PiecewisePolynomial.Cubic(
                        t_knots, path.T, np.zeros(7), np.zeros(7)
                    )
                    return [JointSpacePlan(qtraj)], [0.1]
        return None, None


def safe_path(cspace, start, end, util):
    path = cspace.path(start, end)
    safe_path = []
    for configuration in path:
        if util.collision(configuration):
            return safe_path
        safe_path.append(configuration)
    return safe_path


def within_tol(test, target, tol):
    return np.all(np.hstack((target-tol <= test, test <= target+tol)))