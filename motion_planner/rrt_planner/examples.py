from geometry import Point, Polygon, Object
from robot import Robot, RobotArm, Range, ConfigurationSpace
from rrt_planning import Problem
from math import pi

def random_robot_problem(methods):
  print 'Random Robot Problem'
  robot = Robot([ \
    Polygon([Point(-2, -1), Point(2, -1), Point(2, 1), Point(-2, 1)]),
    Polygon([Point(-1, 1), Point(1, 1), Point(0, 2)])
  ])

  start = (5, 5, 0)
  goal = (15, 15, pi/2)

  cspace = ConfigurationSpace([ \
    Range(0, 20), \
    Range(0, 20), \
    Range(-pi, pi, wrap_around = True)
  ], start, robot.distance, 3*[0.25])

  obstacles = []   
  problem = Problem(20, 20, robot, obstacles, start, goal, cspace, display_tree = True)
  for i in range(3):
    problem.generate_random_poly((3, 15), (1, 4))
  for method in methods:
      print 'Method', method
      problem.run_and_display(method, DISPLAY)

def robot_problem(methods):
  print 'Robot Problem'
  robot = Robot([ \
    Polygon([Point(-2, -1), Point(2, -1), Point(2, 1), Point(-2, 1)]),
    Polygon([Point(-1, 1), Point(1, 1), Point(0, 2)])
  ])

  start = (5.0, 5.0, 0)
  goal = (15.0, 15.0, pi/2)

  cspace = ConfigurationSpace([ \
    Range(0, 20), \
    Range(0, 20), \
    Range(-pi, pi, wrap_around = True)
  ], start, robot.distance, 3*[0.25])

  obstacles = [Object(Point(10, 10),
                      [Polygon([Point(0, -2), Point(2, 0),
                                Point(0, 2), Point(-2, 0)])])]   
  problem = Problem(20, 20, robot, obstacles, start, goal, cspace, display_tree = True)
  for method in methods:
      print 'Method', method
      problem.run_and_display(method, DISPLAY)
    
def create_link(length, width):
  return Polygon([Point(0, -width/2.0), Point(length, -width/2.0),
                  Point(length, width/2.0), Point(0, width/2.0)])

def robot_arm_problem(methods):
  print 'Robot Arm Problem'
  robot = RobotArm(Point(10, 10), [ \
    (Point(-.5, 0), create_link(4, 1)), \
    (Point(3.5, 0), create_link(3, 1)), \
    (Point(2.5, 0), create_link(2, 1))
  ])

  start = (pi/2, pi/2, -pi/4)
  goal = (pi/4, -pi/6, -pi/3)

  cspace = ConfigurationSpace([ \
    Range(-pi/2, pi/2), \
    Range(-pi/2, pi/2), \
    Range(-pi/2, pi/2)
  ], start, robot.distance, 3*[0.25])
  
  """
  cspace = ConfigurationSpace([ \
    Range(-pi, pi, wrap_around = True), \
    Range(-pi, pi, wrap_around = True), \
    Range(-pi, pi, wrap_around = True)
  ], start, robot.distance, 3*[0.25])
  """

  obstacles = [Object(Point(12, 17.5),
                      [Polygon([Point(0, -2), Point(2, 0),
                                Point(0, 2), Point(-2, 0)])]),
               Object(Point(16, 16),
                      [Polygon([Point(0, -2), Point(2, 0),
                                Point(0, 2), Point(-2, 0)])])]   
  problem = Problem(20, 20, robot, obstacles, start, goal, cspace)
  for method in methods:
      print 'Method', method
      return problem.run_and_display(method, DISPLAY)

def random_robot_arm_problem(methods):
  print 'Random Robot Arm Problem'
  robot = RobotArm(Point(10, 10), [ \
    (Point(-.5, 0), create_link(4, 1)), \
    (Point(3.5, 0), create_link(2, 1)), \
    (Point(1.5, 0), create_link(4, 1))
  ])

  start = (pi/2, pi/2, -pi/4)
  goal = (-pi/4, -pi/6, -pi/3)
  
  cspace = ConfigurationSpace([ \
    Range(-pi/2, pi/2), \
    Range(-pi/2, pi/2), \
    Range(-pi/2, pi/2)
  ], start, robot.distance, 3*[0.25])
  
  """
  cspace = ConfigurationSpace([ \
    Range(-pi, pi, wrap_around = True), \
    Range(-pi, pi, wrap_around = True), \
    Range(-pi, pi, wrap_around = True)
  ], start, robot.distance, 3*[0.25])
  """

  obstacles = []    
  problem = Problem(20, 20, robot, obstacles, start, goal, cspace)
  for i in range(3):
    problem.generate_random_poly((3, 15), (1, 4))
  for method in methods:
      print 'Method', method
      problem.run_and_display(method, DISPLAY)

if __name__ == "__main__":
    DISPLAY = True              # set to True to do displays
    methods = ['rrt', 'birrt']

    robot_problem(methods)
    # random_robot_problem(methods)
    # robot_arm_problem(methods)
    # random_robot_arm_problem(methods)
