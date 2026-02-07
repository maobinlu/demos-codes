from ..aloha.robot_utils import torque_on, torque_off, move_arms, move_grippers, get_arm_gripper_positions
from .constants import MASTER2PUPPET_JOINT_FN, DT, START_ARM_POSE, MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import time
import sys
import IPython
import numpy as np
import modern_robotics as mr
e = IPython.embed


def prep_robots(master_bot, puppet_bot):
    # reboot gripper motors, and set operating modes for all motors
    puppet_bot.dxl.robot_reboot_motors("single", "gripper", True)
    puppet_bot.dxl.robot_set_operating_modes("group", "arm", "position")
    puppet_bot.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")
    master_bot.dxl.robot_set_operating_modes("group", "arm", "position")
    master_bot.dxl.robot_set_operating_modes("single", "gripper", "position")
    torque_on(puppet_bot)
    torque_on(master_bot)

    # move arms to starting position
    start_arm_qpos = START_ARM_POSE[:6]
    move_arms([master_bot, puppet_bot], [start_arm_qpos] * 2, move_time=1)
    # move grippers to starting position
    move_grippers([master_bot, puppet_bot], [MASTER_GRIPPER_JOINT_MID, PUPPET_GRIPPER_JOINT_CLOSE], move_time=0.5)


def press_to_start(master_bot):
    # press gripper to start data collection
    # disable torque for only gripper joint of master robot to allow user movement
    master_bot.dxl.robot_torque_enable("single", "gripper", False)
    print(f'Close the gripper to start')
    close_thresh = -0.2
    pressed = False
    while not pressed:
        gripper_pos = get_arm_gripper_positions(master_bot)
        if gripper_pos < close_thresh:
            pressed = True
        time.sleep(DT/10)
    torque_off(master_bot)
    print(f'Started!')


def teleop(robot_side):
    """ A standalone function for experimenting with teleoperation. No data recording. """
    puppet_bot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=f'puppet_{robot_side}', init_node=True)
    master_bot = InterbotixManipulatorXS(robot_model="wx250s", group_name="arm", gripper_name="gripper", robot_name=f'master_{robot_side}', init_node=False)

    prep_robots(master_bot, puppet_bot)
    press_to_start(master_bot)

    # Teleoperation loop
    gripper_command = JointSingleCommand(name="gripper")
    first_iter = True
    while True:
        # sync joint positions
        master_state_joints = master_bot.dxl.joint_states.position[:6]
        puppet_ee_pose = mr.FKinSpace(puppet_bot.arm.robot_des.M, puppet_bot.arm.robot_des.Slist, master_state_joints)
        if first_iter:
            puppet_state_joints = puppet_bot.dxl.joint_states.position[:6]
            puppet_bot.arm.set_ee_pose_matrix(puppet_ee_pose, blocking=False, custom_guess=puppet_state_joints)
            first_iter = False
        puppet_state_joints_last = np.array(puppet_state_joints)
        puppet_state_joints = np.array(puppet_bot.dxl.joint_states.position[:6])
        puppet_state_joints_guess = puppet_state_joints - (puppet_state_joints_last - master_state_joints)
        puppet_bot.arm.set_ee_pose_matrix(puppet_ee_pose, blocking=False, custom_guess=puppet_state_joints_guess)

        # sync gripper positions
        master_gripper_joint = master_bot.dxl.joint_states.position[6]
        puppet_gripper_joint_target = MASTER2PUPPET_JOINT_FN(master_gripper_joint)
        gripper_command.cmd = puppet_gripper_joint_target
        puppet_bot.gripper.core.pub_single.publish(gripper_command)
        # sleep DT
        time.sleep(DT)


if __name__ == '__main__':
    side = sys.argv[1]
    teleop(side)
