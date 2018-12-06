# Written by Caelan Garrett, modified a bit by Tomas Lozano-Perez
from geometry import Point, Line, Polygon, Object, AABB, convex_hull
from DrawingWindowStandalone import DrawingWindow
from random import random, randint, uniform, choice
from time import time
from heapq import heappush, heappop
from math import pi, cos, sin
import pdb

class TreeNode:
  def __init__(self, value, parent = None):
    self.value = value  # tuple of floats representing a configuration
    self.parent = parent  # another TreeNode
    self.children = []  # list of TreeNodes

class RRT:                      # RRT Tree
  def __init__(self, root, cspace):
    self.root = root  # root TreeNode
    self.cspace = cspace  # robot.ConfigurationSpace
    self.size = 1  # int length of path
    self.max_recursion = 1000  # int length of longest possible path

  def add_configuration(self, parent_node, child_value):
    child_node = TreeNode(child_value, parent_node)
    parent_node.children.append(child_node)
    self.size +=1 
    return child_node

  # Brute force nearest, handles general distance functions
  def nearest(self, configuration):
    '''Finds the nearest node by distance to configuration in the
       configuration space.

    Args:
      configuration: tuple of floats representing a configuration of a
        robot

    Returns:
      closest: TreeNode. the closest node in the configuration space
        to configuration
      distance: float. distance from configuration to closest

    '''
    assert self.cspace.valid_configuration(configuration)
    def recur(node, depth=0):
      closest, distance = node, self.cspace.distance(node.value, configuration)
      if depth < self.max_recursion:
        for child in node.children:
          (child_closest, child_distance) = recur(child, depth+1)
          if child_distance < distance:
            closest = child_closest
            distance = child_distance
      return closest, distance
    return recur(self.root)[0]

  def draw(self, window, color = 'black'):
    def recur(node, depth=0):
      node_point = Point(node.value[0], node.value[1])
      if depth < self.max_recursion:
        for child in node.children:
          child_point = Point(child.value[0], child.value[1])
          Line(node_point, child_point).draw(window, color=color, width=1)
          recur(child, depth+1)
    recur(self.root)

class Problem:                    # defines a motion-planning problem
  def __init__(self, x, y, robot, obstacles, start, goal, cspace, display_tree=False):
    '''Defines a motion planning problem.

    Args:
      x: float. the width of the map's area
      y: float. the height of the map's area
      robot: a robot.Robot instance
      obstacles: list of geometry.Objects self.robot can't move through
      start: tuple of floats: starting configuration of self.robot
      goal: tuple of floats: goal configuration of self.robot
      cspace: robot.Configuration space of self.robot
      display_tree: bool. if True, draw the generated plan trees
    '''

    self.x = x
    self.y = y
    self.robot = robot
    self.obstacles = obstacles
    self.start = start
    self.goal = goal
    self.region = AABB(Point(0, 0), Point(x, y))
    self.cspace = cspace
    self.display_tree = display_tree

    assert self.valid_configuration(self.start)
    assert self.valid_configuration(self.goal)

  # Generate a regular polygon that does not collide with other
  # obstacles or the robot at start and goal.
  def generate_random_regular_poly(self, num_verts, radius,
                                   angle = uniform(-pi, pi)):
    '''Generates a regular polygon that does not collide with other
       obstacles or the robot at start and goal. This polygon is added
       to self.obstacles. To make it random, keep the default angle
       argument.

    Args:
      num_verts: int. the number of vertices of the polygon >= 3
      radius: float. the distance from the center of the polygon
        to any vertex > 0
      angle: float. the angle in radians between the origin and
        the first vertex. the default is a random value between
        -pi and pi.
    '''
    (min_verts, max_verts) = num_verts
    (min_radius, max_radius) = radius
    assert not (min_verts < 3 or min_verts > max_verts or \
                min_radius <= 0 or min_radius > max_radius)
    reference = Point(random()*self.x, random()*self.y)
    distance = uniform(min_radius, max_radius)
    sides = randint(min_verts, max_verts)
    obj = Object(reference,
                 [Polygon([Point(distance*cos(angle + 2*n*pi/sides),
                                 distance*sin(angle + 2*n*pi/sides)) \
                           for n in range(sides)])])
    if any([obj.collides(current) for current in self.obstacles]) or \
       obj.collides(self.robot.configuration(self.start)) or \
       obj.collides(self.robot.configuration(self.goal)) or \
       not self.region.contains(obj):
      self.generate_random_regular_poly(num_verts, radius, angle=angle)
    else:
      self.obstacles.append(obj)

  # Generate a polygon that does not collide with other
  # obstacles or the robot at start and goal.
  def generate_random_poly(self, num_verts, radius):
    '''Generates a random polygon that does not collide with other
       obstacles or the robot at start and goal. This polygon is added
       to self.obstacles.

    Args:
      num_verts: int. the number of vertices of the polygon >= 3
      radius: float. a reference distance between the origin and some
        vertex of the polygon > 0
    '''
    (min_verts, max_verts) = num_verts
    (min_radius, max_radius) = radius
    assert not (min_verts < 3 or min_verts > max_verts or \
                min_radius <= 0 or min_radius > max_radius)

    reference = Point(random()*self.x, random()*self.y)
    verts = randint(min_verts, max_verts)
    points = [Point(0, 0)]
    for i in range(verts):
      angle = 2*pi*random()
      points.append(((max_radius - min_radius)*random() + min_radius)*Point(cos(angle), sin(angle)))
    obj = Object(reference, [Polygon(convex_hull(points))])
    if any([obj.collides(current) for current in self.obstacles]) or \
       obj.collides(self.robot.configuration(self.start)) or \
       obj.collides(self.robot.configuration(self.goal)) or \
       not self.region.contains(obj):
      self.generate_random_poly(num_verts, radius)
    else:
      self.obstacles.append(obj)

  def valid_configuration(self, configuration):
    '''Checks if the given configuration is valid in this
       Problem's configuration space and doesn't collide with
       any obstacles.

    Args:
      configuration: tuple of floats - tuple describing a robot
        configuration

    Returns:
      bool. True if the given configuration is valid, False otherwise

    '''
    return self.cspace.valid_configuration(configuration) \
      and not self.collide(configuration)
            
  def collide(self, configuration): # Check collision for configuration
    '''Checks if the given configuration collides with
       any of this Problem's obstacles.

    Args:
      configuration: tuple of floats - tuple describing a robot
        configuration

    Returns:
      bool. True if the given configuration is in collision, False otherwise
    '''
    config_robot = self.robot.configuration(configuration)
    return any([config_robot.collides(obstacle) \
                for obstacle in self.obstacles]) or \
      not self.region.contains(config_robot)

  def test_path(self, start, end): # check path for collisions
    '''Checks if the path from start to end collides with any
       obstacles.

    Args:

      start: tuple of floats - tuple describing robot's start configuration
      end: tuple of floats - tuple describing robot's end configuration

    Returns:
      bool. False if the path collides with any obstacles, True otherwise

    '''
    path = self.cspace.path(start, end)
    for configuration in path:
      if self.collide(configuration):
        return False
    return True 

  def safe_path(self, start, end): # returns subset of path that is safe
    '''Checks if the path from start to end collides with any
       obstacles.

    Args:
      start: tuple of floats - tuple describing robot's start configuration
      end: tuple of floats - tuple describing robot's end configuration

    Returns:
      list of tuples along the path that are not in collision.
    '''
    path = self.cspace.path(start, end)
    safe_path = []
    for configuration in path:
      if self.collide(configuration):
        return safe_path
      safe_path.append(configuration)
    return safe_path 

  def path_distance(self, path): # add up distances along path
    '''Adds up the distance along the given path.

    Args:
      path: list of tuples describing configurations of the robot

    Returns:
      float. the total distance along the path

    '''
    distance = 0
    for i in range(len(path)-1):
      distance += self.cspace.distance(path[i], path[i+1])
    return distance

  # Given a path (list of configurations) return a (shorter) path
  def smooth_path(self, path, attempts = 100): # random short-cutting
    '''Given a path (list of configurations), return a possibly shorter
       path.

    Args:

      path: list of tuples describing configurations of the robot
      attemps: int. the number of times to try to smooth the path

    Returns:
      list of tuples describing configurations of the robot that is
      possibly shorter than the given path

    '''
    #########################
    # Your code here
    for _ in range(attempts):
      ind = choice(range(len(path)-1))
      conf = path[ind]
      target_ind = choice(range(ind+1, len(path)))
      target_conf = path[target_ind]
      if self.test_path(conf, target_conf):
        path = path[:ind+1] + path[target_ind:]
    return path
    #########################

  # Uni-directional RRT.
  # Returns a collision free path from self.start to self.goal
  # Sample the goal with goal_sample probability
  def rrt_planning(self, max_iterations = 1000, goal_sample = .05):
    '''Uni-directional RRT. Tries to find a collision free path from
       start to goal.

    Args:
      max_iterations: int. the maximum number of configurations to sample
      goal_sample: float. the probability of sampling the goal point

    Returns:
      a list of tuples representing a path of collision-free
      configurations of the robot
    '''

    rrt = RRT(TreeNode(self.start), self.cspace)
    self.rrts = [rrt]
    #########################
    # Your code here
    def sample_config():
      prob = random()
      if prob < goal_sample:
        return self.goal
      else:
        return self.cspace.sample()

    for i in range(max_iterations):
      sample = sample_config()
      neighbor = rrt.nearest(sample)
      path = self.safe_path(neighbor.value, sample)
      if len(path) > 1:
        child_node = rrt.add_configuration(neighbor, path[0])
        for p in path[1:]:
          child_node = rrt.add_configuration(child_node, p)
        # rrt.add_configuration(neighbor, path[-1])
        if path[-1] == self.goal:
          return self.bfs(rrt, self.goal)
    return None
    #########################


  @staticmethod
  def construct_path(node):
    path = []
    while node is not None:
      path.append(node.value)
      node = node.parent
    return path[::-1]

  def bfs(self, rrt, target):
    queue = [rrt.root]
    while len(queue) > 0:
      node = queue.pop(0)
      if node.value == target:
        return self.construct_path(node)
      for child in node.children:
        queue.append(child)
    raise Exception("Did not find a path when it is expected")

  # Bi-directional RRT
  # Returns a collision free path from self.start to self.goal
  def bidirectional_rrt_planning(self, max_iterations = 1000):
    '''Bidirectional RRT. Tries to find a collision free path from
       start to goal.

    Args:
      max_iterations: int. the maximum number of configurations to sample

    Returns:
      a list of tuples representing a path of collision-free
      configurations of the robot
    '''
    rrt_start = RRT(TreeNode(self.start), self.cspace)
    rrt_goal = RRT(TreeNode(self.goal), self.cspace)
    self.rrts = [rrt_start, rrt_goal]
    #########################
    # Your code here
    def explore(rrt):
      sample = self.cspace.sample()
      neighbor = rrt.nearest(sample)
      path = self.safe_path(neighbor.value, sample)
      for conf in path:
          rrt.add_configuration(neighbor, conf)

    def connect(rrt, rrt_target):
      nodes = []
      queue = [rrt_target.root]
      while len(queue) > 0:
        node = queue.pop(0)
        nodes.append(node)
        for child in node.children:
          queue.append(child)
      target = choice(nodes).value
      neighbor = rrt.nearest(target)
      path = self.safe_path(neighbor.value, target)
      if len(path) > 1:
        rrt.add_configuration(neighbor, path[-1])
        if path[-1] == target:
          return self.bfs(rrt_start, target) + self.bfs(rrt_goal, target)[-2::-1]
      return None

    for _ in range(max_iterations):
      if randint(0, 1) == 0:
        explore(rrt_start)
      else:
        explore(rrt_goal)
      if randint(0, 1) == 0:
        result = connect(rrt_start, rrt_goal)
      else:
        result = connect(rrt_goal, rrt_start)
      if result is not None:
        return result
    return None
    #########################

  def draw_robot_path(self, path, color = None):
    '''Draws the robot in each configuration along a path in a Tkinter window.

    Args:
      path: list of tuples describing configurations
        of the robot
      color: string. Tkinter color of the robot
    '''
    for i in range(1, len(path) - 1): # don't draw start and end
      self.robot.configuration(path[i]).draw(self.window, color)
      self.window.update()
      raw_input('Next?')
 
  def run_and_display(self, method, display=True):    # runs the problem and displays result
    '''Runs the Problem with the given methods and draws the path in a Tkinter window.

    Args:
      method: list of strings of the methods to try, must be "rrt" or "birrt"
      display: bool. if True, draw the Problem in a Tkinter window

    Returns:
      bool. True if a collision free path is found, False otherwise
    '''
    def draw_problem():
      for obs in self.obstacles:
        obs.draw(self.window, 'red')
      self.robot.configuration(self.start).draw(self.window, 'orange')
      self.robot.configuration(self.goal).draw(self.window, 'green')
    if display:
      self.window = DrawingWindow(600, 600, 0, self.x, 0, self.y, 'RRT Planning')  
      draw_problem()
    t1 = time()
    if method == 'rrt':
      path = self.rrt_planning()
    elif method == 'birrt':
      path = self.bidirectional_rrt_planning()
    else:
      raise NotImplemented
    print 'RRT took', time() - t1, 'seconds'
    if display and self.display_tree:
      for rrt in self.rrts:
        rrt.draw(self.window)

    if path == None:
      print 'No path found'
      return False
    else:
      print 'Path found with ' + str(len(path)-1) + \
        ' movements of distance ', self.path_distance(path)
      smooth_path = self.smooth_path(path)
      print 'Smoothed path found with ' + str(len(smooth_path)-1) + \
        ' movements of distance ', self.path_distance(smooth_path)

      # print("start:")
      # print(self.start)
      # print("goal:")
      # print(self.goal)
      # print("path:")
      # print(path)

      # interpolated smooth path
      spath = []
      for i in range(1, len(smooth_path)):
        spath.extend(self.cspace.path(smooth_path[i-1], smooth_path[i]))
      # make sure path is collision free
      if any([self.collide(c) for c in spath]):
        print 'Collision in smoothed path'
        return False
      if display:
        print(path)
        self.draw_robot_path(path, color = 'yellow')
        self.window.clear()
        draw_problem()
        self.draw_robot_path(spath, color = 'gold')
        while raw_input('End? (y or n)') != 'y':
          pass
    return True

