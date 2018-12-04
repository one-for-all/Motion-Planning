from pydrake.common import FindResourceOrThrow
from pydrake.all import LeafSystem

from plan_runner.manipulation_station_simulator import ManipulationStationSimulator
from plan_runner.plan_utils import PlotExternalTorqueLog, PlotIiwaPositionLog

from motion_planner.pickup_brick import GeneratePickupBrickPlansByTrajectory
from motion_planner.rrt import GeneratePickupBrickPlansByRRT

if __name__ == '__main__':
    object_file_path = FindResourceOrThrow(
        'drake/examples/manipulation_station/models/061_foam_brick.sdf'
    )

    manip_station_sim = ManipulationStationSimulator(
        time_step=2e-3,
        object_file_path=object_file_path,
        object_base_link_name='base_link'
    )

    print(LeafSystem()._DeclareAbstractInputPort)

    # Generate with Trajectory
    plan_list, gripper_setpoint_list = GeneratePickupBrickPlansByTrajectory()

    # Generate with RRT
    # plan_list, gripper_setpoint_list = GeneratePickupBrickPlansByRRT()

    # Simulation
    q0 = [0, 0, 0, -1.75, 0, 1.0, 0]
    iiwa_position_command_log, iiwa_position_measured_log, iiwa_external_torque_log, \
        state_log = manip_station_sim.RunSimulation(
        plan_list, gripper_setpoint_list, extra_time=0.0, real_time_rate=1.0, q0_kuka=q0)
    PlotExternalTorqueLog(iiwa_external_torque_log)
    PlotIiwaPositionLog(iiwa_position_command_log, iiwa_position_measured_log)