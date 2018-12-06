import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.eigen_geometry import Isometry3

from plan_runner.manipulation_station_simulator import ManipulationStationSimulator
from plan_runner.plan_utils import PlotExternalTorqueLog, PlotIiwaPositionLog

from motion_planner.pickup_brick import GeneratePickupBrickPlansByTrajectory
from motion_planner.rrt import MotionPlanning


if __name__ == '__main__':
    object_file_path = FindResourceOrThrow(
        'drake/examples/manipulation_station/models/061_foam_brick.sdf'
    )
    obstacle_file_path = "motion_planner/models/cracker_box.sdf"

    X_WObject = Isometry3.Identity()
    X_WObject.set_translation([.6, 0, 0])
    X_WObstacle = Isometry3.Identity()
    X_WObstacle.set_translation([.4, 0, 0])

    manip_station_sim = ManipulationStationSimulator(
        time_step=2e-3,
        object_file_paths=[object_file_path, obstacle_file_path],
        object_base_link_names=['base_link', 'base_link_cracker'],
        X_WObject_list=[X_WObject, X_WObstacle]
    )

    # initial q and goal p
    q0 = [0, 0, 0, -1.75, 0, 1.0, 0]
    p_WC_box = np.array([0.6, 0.05 / 2, 0.025 + 0.025])

    RRT = True
    if RRT:
        # Generate with RRT
        motion_planning = MotionPlanning(q_initial=q0,
                                         p_goal=p_WC_box,
                                         object_file_paths=[object_file_path, obstacle_file_path],
                                         object_base_link_names=['base_link', 'base_link_cracker'],
                                         X_WObject_list=[X_WObject, X_WObstacle])
        plan_list, gripper_setpoint_list = motion_planning.rrt_plan()

    else:
        # Generate with Trajectory
        plan_list, gripper_setpoint_list = GeneratePickupBrickPlansByTrajectory(p_WC_box)

    # Simulation

    iiwa_position_command_log, iiwa_position_measured_log, iiwa_external_torque_log, \
        state_log = manip_station_sim.RunSimulation(
        plan_list, gripper_setpoint_list, extra_time=0.0, real_time_rate=1.0, q0_kuka=q0)
    PlotExternalTorqueLog(iiwa_external_torque_log)
    PlotIiwaPositionLog(iiwa_position_command_log, iiwa_position_measured_log)