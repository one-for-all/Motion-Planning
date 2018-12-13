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
        """
        :param value: value of tree node, which in our use case is
                    np.array of shape (7,) which represents the joint angles
        :param parent: parent node
        """
        self.value = value
        self.parent = parent
        self.children = []

    def __eq__(self, other):
        """
        :param other: the other node to compare against
        :return: whether two nodes are considered equal (by equality of their values)
        """
        return np.all(self.value == other.value)

    def __hash__(self):
        return hash(self.value)


class RRT:
    def __init__(self, root, cspace):
        """
        :param root: root node, which is the initial configuration of the robot
        :param cspace: ConfigurationSpace of the robot (robot joint angles)
        """
        self.root = root
        self.cspace = cspace
        self.size = 1
        self.max_recursion = 5000

    def add_configuration(self, parent_node, child_value):
        """
        :param parent_node: parent node to add to
        :param child_value: child node value
        :return: child node that is created
        """
        child_node = TreeNode(child_value, parent_node)
        parent_node.children.append(child_node)
        self.size += 1
        return child_node

    def nearest(self, value):
        """
        :param value: configuration of the robot to find against
        :return: the existing node in rrt that is closest to value,
                 according to distance metric of self.cspace
        """
        assert self.cspace.valid_configuration(value)

        def recur(node, depth=0):
            closest, distance = node, self.cspace.distance(node.value, value)
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

    def construct_path(self, node, use_node=False):
        """
        :pre: node is reachable from root

        :param node: target node
        :return: the path (a list of values) that goes from root to node
        """
        path = []
        while node.parent is not None:
            if use_node:
                path.append(node)
            else:
                path.append(node.value)
            node = node.parent
        if use_node:
            path.append(node)
        else:
            path.append(node.value)
        assert node == self.root
        return path[::-1]

    def bfs(self, target, use_node=False):
        """
        :param target: target value
        :return: the path (a list of values) that goes from root to node with target value
        """
        assert self.cspace.valid_configuration(target)

        queue = [self.root]
        while len(queue) > 0:
            node = queue.pop(0)
            if np.all(node.value == target):
                return self.construct_path(node, use_node)
            for child in node.children:
                queue.append(child)
        raise Exception("Did not find a path when it is supposed to")

    def cost(self, node):
        """
        :param node: the node to reach
        :return: the cost of reaching that node from root
        """
        cost = 0
        while node.parent is not None:
            cost += self.cspace.distance(node.parent.value, node.value)
            node = node.parent
        assert node == self.root
        return cost

    def near(self, value, max_cost):
        """
        :param value: the value to evaluate against
        :param max_cost: the max cost between values
        :return: a list of nodes where each is within max_cost to value
        """
        near_nodes = []
        queue = [self.root]
        while len(queue) > 0:
            node = queue.pop(0)
            if self.cspace.distance(node.value, value) < max_cost:
                near_nodes.append(node)
            for child in node.children:
                queue.append(child)
        return near_nodes

    def remove_edge(self, parent, child):
        """
        :param parent: parent node
        :param child: child node
        :return: remove child from parent's children
        """
        try:
            parent.children.remove(child)
        except ValueError:
            raise Exception("Child is not a child node of parent")
        child.parent = None
        self.size -= 1

    def add_edge(self, parent, child):
        """
        :param parent: parent node
        :param child: child node
        :return: add child as child of parent
        """
        parent.children.append(child)
        child.parent = parent
        self.size += 1

    def change_parent(self, new_parent, child):
        """
        :param new_parent: new parent node of child
        :param child: child node
        :return: remove child from its original parent, and change to new parent
        """
        old_parent = child.parent
        self.remove_edge(old_parent, child)
        self.add_edge(new_parent, child)

    def shortest_path(self, goals):
        min_cost = None
        best_path = None
        for i, goal_node in enumerate(goals):
            try:
                path = self.bfs(goal_node.value, use_node=True)
                cost = self.cost(goal_node)
                if min_cost is None or cost < min_cost:
                    min_cost = cost
                    best_path = path
            except Exception as e: # No path to goal exists
                goals[i] = None
        return best_path, min_cost


class Range:
    def __init__(self, low, high):
        """
        :param low: low value of range
        :param high: high value of range
        """
        self.low = low
        self.high = high

    def sample(self):
        """
        :return: randomly sample a value between low and high
        """
        return (self.high - self.low)*random() + self.low

    def contains(self, x):
        """
        :param x: value
        :return: whether x is between low and high
        """
        return self.low <= x <= self.high

    def difference(self, one, two):
        """
        :param one: first valeue
        :param two: second value
        :return: second value - first value
                 (which could be non-trivial in a range that wraps around)
        """
        return two-one

    def in_range(self, value):
        """
        :param value: value
        :return: value if value is in range, None otherwise
        """
        if self.contains(value):
            return value
        else:
            return None


class ConfigurationSpace:
    def __init__(self, ranges, util):
        """
        :param ranges: a list of tuples that represent ranges
        :param util: util module for distance calculation
        """
        self.cspace_ranges = [Range(j[0], j[1]) for j in ranges]
        self.util = util

    def sample(self):
        """
        :return: a random point in the configuration space, (a list of values)
        """
        return [r.sample() for r in self.cspace_ranges]

    def valid_configuration(self, config):
        """
        :param config: a point
        :return: whether this point is in configuration space
        """
        return len(config) == len(self.cspace_ranges) and \
            all([self.cspace_ranges[i].contains(config[i]) \
                 for i in range(len(self.cspace_ranges))])

    def distance(self, one, two):
        """
        :param one: first point
        :param two: second point
        :return: distance metric in the configuration space
        """
        # return np.linalg.norm(np.array(one) - np.array(two))
        # a = self.util.FK(one).translation()
        # b = self.util.FK(two).translation()
        # return np.linalg.norm(a - b)
        diff = np.array(one) - np.array(two)
        diff *= np.array([7, 6, 5, 4, 3, 2, 1])
        # diff *= np.array([1, 1, 1, 1, 1, 1, 1])
        return np.linalg.norm(diff)

    def path(self, start, end, max_diff=0.01):
        """
        :param start: starting point
        :param end: end point
        :param: max_diff: maximum scalar difference between consecutive values
        :return: a linear path from start to end
        """
        assert self.valid_configuration(start) and self.valid_configuration(end)
        diffs = [self.cspace_ranges[i].difference(start[i], end[i]) \
                 for i in range(len(self.cspace_ranges))]
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
                 X_WObject_list=None,
                 steps=100):
        """
        :param q_initial: initial config of the robot
        :param p_goal: goal point in World space
        :param object_file_paths: a list of objects in the world, each a sdf file path
        :param object_base_link_names: a list of object base link names
        :param X_WObject_list: a list of initial pose of the objects
        """
        self.q_initial=q_initial
        self.p_goal=p_goal

        # Set up station
        self.station = ManipulationStation(collision_model=IiwaCollisionModel.kBoxCollision)
        self.station.AddCupboard()
        self.plant = self.station.get_mutable_multibody_plant()

        # Add objects into world
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

        # Set up util module
        self.util = Util(self.station)

        # Set up configuration space
        joint_ranges = self.util.get_joint_ranges()
        self.cspace = ConfigurationSpace(joint_ranges, self.util)

        # Set initial pose of the objects
        if self.object_base_link_names is not None:
            for link_name, object, X_WObject in zip(self.object_base_link_names,
                                                    self.objects,
                                                    self.X_WObject_list):
                self.util.set_object_pose(link_name, object, X_WObject)

        # Set robot initial pose
        self.util.set_iiwa_position(self.q_initial)
        self.util.set_wsg_position(0.1)

        # book keeping
        self.rewires = 0
        self.edge_evaluation = 0
        self.steps = steps
        self.edge_evaluations = []

    def lazy_sp(self, max_iters=1000, goal_sample_prob=0.05, position_tolerance=0.09, duration=5):
        util = self.util
        cspace = self.cspace
        rrt = RRT(TreeNode(self.q_initial), cspace)

        q_goals = []

        for i in range(max_iters):
            print("iter: ", i)
            sample = self.sample_config(util, goal_sample_prob)
            neighbor = rrt.nearest(sample)
            # rrt.add_configuration(neighbor, sample)

            # Find best node for reaching q_new
            min_cost = rrt.cost(neighbor) + cspace.distance(neighbor.value, sample)
            q_min_node = neighbor
            # Stores q near nodes, and dist to q_new
            Q_near = [[q_node, None] for q_node in rrt.near(sample, max_cost=10)]
            print("num near nodes: {}".format(len(Q_near)))

            for i, v in enumerate(Q_near):
                q_near_node, _ = v
                dist_to_new = cspace.distance(q_near_node.value, sample)
                Q_near[i][1] = dist_to_new
                cost = rrt.cost(q_near_node) + dist_to_new
                if cost < min_cost:
                    min_cost = cost
                    q_min_node = q_near_node

            q_new_node = rrt.add_configuration(q_min_node, sample)

            # Rewiring existing nodes
            new_node_cost = rrt.cost(q_new_node)
            rewires = 0
            for q_near_node, dist_to_new in Q_near:
                if (q_near_node != q_min_node and
                        rrt.cost(q_near_node) > new_node_cost + dist_to_new):
                    rewires += 1
                    rrt.change_parent(q_new_node, q_near_node)
            print("Num rewires: {}".format(rewires))
            self.rewires += rewires

            # check for goal
            translation = util.FK(sample).translation()
            if self.within_tol(translation, self.p_goal, position_tolerance):
                q_goals.append(q_new_node)

        def filter(goals):
            return [g for g in goals if g is not None]

        def in_free_edges(path, edges):
            for s, e in zip(path[:-1], path[1:]):
                if (s, e) not in edges:
                    return False
            return True

        def edge_selector(path, collision_free_edges):
            for s, e in zip(path[:-1], path[1:]):
                if (s, e) not in collision_free_edges:
                    return s, e
            raise Exception("Unexpected that all edges are collision free")

        collision_free_edges = []
        while len(q_goals) > 0:
            best_path, min_cost = rrt.shortest_path(q_goals)
            # print(best_path)
            if best_path is None:
                break
            q_goals = filter(q_goals)
            if in_free_edges(best_path, collision_free_edges):
                best_path = np.array([node.value for node in best_path])
                num_knot_points = best_path.shape[0]
                t_knots = np.linspace(0, duration, num_knot_points)
                qtraj = PiecewisePolynomial.Cubic(t_knots, best_path.T, np.zeros(7), np.zeros(7))
                print("Path cost: {}".format(min_cost))
                print("Total number of rewires: {}".format(self.rewires))
                return [JointSpacePlan(qtraj)], [0.1]

            start, end = edge_selector(best_path, collision_free_edges)
            if self.obstacle_free(cspace, start.value, end.value, util):
                collision_free_edges.append((start, end))
            else:
                rrt.remove_edge(start, end)

        raise Exception("Did not find path to goal")

    def rrt(self, star=False, max_iters=1000, goal_sample_prob=0.05, position_tolerance=0.09, duration=5):
        util = self.util
        cspace = self.cspace
        rrt = RRT(TreeNode(self.q_initial), cspace)

        q_goals = []

        for i in range(max_iters):
            print("iter: ", i)
            sample = self.sample_config(util, goal_sample_prob)
            neighbor = rrt.nearest(sample)
            path = self.safe_path(cspace, neighbor.value, sample, util)
            if len(path) > 1:
                q_end = path[-1]
                # if np.all(path[-1] == sample):
                #     q_end = path[-1]
                # else:
                #     if len(path) > 10:
                #         q_end = path[-10]
                #     else:
                #         continue

                q_new_node = neighbor
                add_middle_nodes = not star
                if add_middle_nodes:
                    middle_path = path[1:-1:10]
                else:
                    middle_path = []
                for q in middle_path + [q_end]:
                    if not star:
                        q_new_node = self.rrt_extend(rrt, q_new_node, q)
                    else:
                        q_new_node = self.rrt_star_extend(rrt, q_new_node, q)

                # check for goal
                translation = util.FK(q_end).translation()
                if self.within_tol(translation, self.p_goal, position_tolerance):
                    q_goals.append(q_new_node)

                if ((not star and len(q_goals) > 0) or
                    (star and len(q_goals) > 0 and i == max_iters-1)):
                # if len(q_goals) > 0:
                    min_cost = None
                    best_path = None
                    for q_goal_node in q_goals:
                        path = np.array(rrt.bfs(q_goal_node.value))
                        cost = rrt.cost(q_goal_node)
                        if min_cost is None or cost < min_cost:
                            min_cost = cost
                            best_path = path

                    num_knot_points = best_path.shape[0]
                    t_knots = np.linspace(0, duration, num_knot_points)
                    qtraj = PiecewisePolynomial.Cubic(t_knots, best_path.T, np.zeros(7), np.zeros(7))
                    print("Path cost: {}".format(min_cost))
                    print("Total number of rewires: {}".format(self.rewires))
                    if i + 1 % self.steps == 0:
                        self.edge_evaluations.append(self.edge_evaluation)
                    return [JointSpacePlan(qtraj)], [0.1]
                if i + 1 % self.steps == 0:
                    self.edge_evaluations.append(self.edge_evaluation)

        raise Exception("Did not find path to goal")

    def rrt_extend(self, rrt, neighbor, q_new):
        return rrt.add_configuration(neighbor, q_new)

    def rrt_star_extend(self, rrt, neighbor, q_new):
        cspace = rrt.cspace
        # Find best node for reaching q_new
        min_cost = rrt.cost(neighbor) + cspace.distance(neighbor.value, q_new)
        q_min_node = neighbor
        # Stores q near nodes, whether obstacle free, and dist to q_new
        Q_near = [[q_node, None, None] for q_node in rrt.near(q_new, max_cost=10)]
        print("num near nodes: {}".format(len(Q_near)))

        for i, v in enumerate(Q_near):
            q_near_node, _, _ = v

            dist_to_new = cspace.distance(q_near_node.value, q_new)
            Q_near[i][2] = dist_to_new
            cost = rrt.cost(q_near_node) + dist_to_new
            if cost < min_cost:
                if self.obstacle_free(cspace, q_near_node.value, q_new, self.util):
                    Q_near[i][1] = True
                    min_cost = cost
                    q_min_node = q_near_node

            # if obstacle_free(cspace, q_near_node.value, q_new, self.util):
            #     Q_near[i][1] = True
            #     dist_to_new = cspace.distance(q_near_node.value, q_new)
            #     Q_near[i][2] = dist_to_new
            #     cost = rrt.cost(q_near_node) + dist_to_new
            #     if cost < min_cost:
            #         min_cost = cost
            #         q_min_node = q_near_node

        q_new_node = rrt.add_configuration(q_min_node, q_new)

        # Rewiring existing nodes
        new_node_cost = rrt.cost(q_new_node)
        rewires = 0
        for q_near_node, collision_free, dist_to_new in Q_near:
            if (q_near_node != q_min_node and
                    rrt.cost(q_near_node) > new_node_cost + dist_to_new):
                if collision_free or (collision_free is None and
                                      self.obstacle_free(cspace, q_near_node.value, q_new, self.util)):
                    rewires += 1
                    rrt.change_parent(q_new_node, q_near_node)

            # if (q_near_node != q_min_node and
            #         collision_free and
            #         rrt.cost(q_near_node) > new_node_cost + dist_to_new):
            #     rewires += 1
            #     rrt.change_parent(q_new_node, q_near_node)
        print("Num rewires: {}".format(rewires))
        self.rewires += rewires
        return q_new_node

    def sample_config(self, util, goal_sample_prob):
            prob = random()
            if prob < goal_sample_prob:
                q_goal = None
                it = 0
                while q_goal is None:
                    # print("goal sampling iter: ", it)
                    it += 1
                    q_goal_sample = util.IK(self.p_goal, is_goal=False)
                    if not util.collision(q_goal_sample):
                        q_goal = q_goal_sample
                return q_goal
            else:
                q = None
                it = 0
                while q is None:
                    # print("non goal sampling iter: ", it)
                    it += 1
                    q_sample = util.random_q()
                    if not util.collision(q_sample):
                        q = q_sample
                return q

    def safe_path(self, cspace, start, end, util):
        self.edge_evaluation += 1
        path = cspace.path(start, end)
        safe_path = []
        for configuration in path:
            if util.collision(configuration):
                return safe_path
            safe_path.append(configuration)
        return safe_path

    def within_tol(self, test, target, tol):
        return np.all(np.hstack((target-tol <= test, test <= target+tol)))

    def obstacle_free(self, cspace, start, end, util):
        self.edge_evaluation += 1
        path = cspace.path(start, end)
        for config in path:
            if util.collision(config):
                return False
        return True