import argparse
import time
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.eigen_geometry import Isometry3

from plan_runner.manipulation_station_simulator import ManipulationStationSimulator

from motion_planner.reach_brick import GeneratePickupBrickPlansByTrajectory
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
        "-s", "--steps", type=int, default=100,
        help="Specify the step between iterations for experiments"
    )
    parser.add_argument(
        "-n", "--num_runs", type=int, default=1,
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
    steps = args.steps
    num_runs = args.num_runs

    t_v = []
    e_v = []

    for iters in range(steps, max_iter+steps, steps):
        motion_planning = MotionPlanning(q_initial=q0,
                                         p_goal=p_WC_box,
                                         object_file_paths=[object_file_path, obstacle_file_path],
                                         object_base_link_names=['base_link', 'base_link_cracker'],
                                         X_WObject_list=[X_WObject, X_WObstacle])

        times = []
        edge_evals = []
        for _ in range(num_runs):
            start = time.time()
            try:
                if algo == 'RRT':
                    # Generate with RRT
                    plan_list, gripper_setpoint_list = motion_planning.rrt(star=False, max_iters=iters)
                elif algo == 'RRTStar':
                    # Generate with RRT Star
                    plan_list, gripper_setpoint_list = motion_planning.rrt(star=True, max_iters=iters)
                elif algo == 'LazySP':
                    # Generate with lazySP
                    plan_list, gripper_setpoint_list = motion_planning.lazy_sp(max_iters=iters)
                else:
                    # Generate with Trajectory
                    plan_list, gripper_setpoint_list = GeneratePickupBrickPlansByTrajectory(p_WC_box)
            except Exception as e:
                print(e)
            end = time.time()
            times.append(end-start)
            edge_evals.append(motion_planning.edge_evaluation)

        print("")
        print(algo)
        print("===========================")
        print("Max Iterations: {}".format(max_iter))
        mean_time = np.mean(times)
        t_v.append(mean_time)
        print("Average Running Time: {}".format(mean_time))
        mean_edges = np.mean(edge_evals)
        e_v.append(mean_edges)
        print("edge evaluation: {}".format(mean_edges))

    print('')
    print("Summary:")
    print("===========================")
    print("{} iterations at {} iters per step for experiment".format(max_iter, steps))
    print("{} runs for each experiment".format(num_runs))
    print("+++++++++++++++++++++++++++")
    print("Average running times")
    print(t_v)
    print("Average number of edge evaluations")
    print(e_v)
    with open("{}.data".format(algo), 'w+') as f:
        f.write(str(t_v))
        f.write("\n")
        f.write(str(e_v))
