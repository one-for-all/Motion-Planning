import argparse
import time
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.eigen_geometry import Isometry3

from plan_runner.manipulation_station_simulator import ManipulationStationSimulator
from plan_runner.plan_utils import PlotExternalTorqueLog, PlotIiwaPositionLog

from motion_planner.pickup_brick import GeneratePickupBrickPlansByTrajectory
from motion_planner.motion_plan import MotionPlanning


if __name__ == '__main__':
    # Parse commandline arguments
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-a", "--algorithm", type=str, default="LazySP",
        choices=['RRT', 'RRTStar', 'LazySP', 'trajectory'],
        help="Specify the algorithm used for motion planning"
    )
    parser.add_argument(
        "-m", "--max_iter", type=int, default=200,
        help="Specify the maximum iterations the algorithm is allowed to run"
    )
    parser.add_argument(
        "-n", "--num_runs", type=int, default=10,
        help="Number of runs for experiment"
    )
    args = parser.parse_args()

    object_file_path = FindResourceOrThrow(
        'drake/examples/manipulation_station/models/061_foam_brick.sdf'
    )
    obstacle_file_path = "motion_planner/models/cracker_box.sdf"

    X_WObject = Isometry3.Identity()
    X_WObject.set_translation([.6, 0, 0])
    X_WObstacle = Isometry3.Identity()
    X_WObstacle.set_translation([0.4, 0, 0])

    manip_station_sim = ManipulationStationSimulator(
        time_step=2e-3,
        object_file_paths=[object_file_path, obstacle_file_path],
        object_base_link_names=['base_link', 'base_link_cracker'],
        X_WObject_list=[X_WObject, X_WObstacle]
    )

    # initial q and goal p
    q0 = [0, 0, 0, -1.75, 0, 1.0, 0]
    p_WC_box = np.array([0.6, 0.05 / 2, 0.025 + 0.025])

    algo = args.algorithm
    max_iter = args.max_iter
    num_runs = args.num_runs

    motion_planning = MotionPlanning(q_initial=q0,
                                     p_goal=p_WC_box,
                                     object_file_paths=[object_file_path, obstacle_file_path],
                                     object_base_link_names=['base_link', 'base_link_cracker'],
                                     X_WObject_list=[X_WObject, X_WObstacle],)

    times = []
    for _ in range(num_runs):
        start = time.time()
        try:
            if algo == 'RRT':
                # Generate with RRT
                plan_list, gripper_setpoint_list = motion_planning.rrt(star=False, max_iters=max_iter)
            elif algo == 'RRTStar':
                # Generate with RRT Star
                plan_list, gripper_setpoint_list = motion_planning.rrt(star=True, max_iters=max_iter)
            elif algo == 'LazySP':
                # Generate with lazySP
                plan_list, gripper_setpoint_list = motion_planning.lazy_sp(max_iters=max_iter)
            else:
                # Generate with Trajectory
                plan_list, gripper_setpoint_list = GeneratePickupBrickPlansByTrajectory(p_WC_box)
        except Exception as e:
            print(e)
        end = time.time()
        times.append(end-start)

    print("")
    print(algo)
    print("===========================")
    print("Max Iterations: {}".format(max_iter))
    print("Average Running Time: {}".format(np.mean(times)))
