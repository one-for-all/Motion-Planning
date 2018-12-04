from pydrake.examples.manipulation_station import ManipulationStation

station = ManipulationStation()
plant = station.get_mutable_multibody_plant()


def GeneratePickupBrickPlansByRRT(is_printing=True):
    plan_list, gripper_setpoint_list, q_final_full = \
        GeneratePickupBrickPlans(is_printing=is_printing)
    return plan_list, gripper_setpoint_list


def GeneratePickupBrickPlans(is_printing=True):

    return None, None, None