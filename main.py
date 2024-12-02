from dofbot import DofbotEnv
import numpy as np
import time
import copy
from scipy.spatial.transform import Rotation as R
import time
import math


if __name__ == '__main__':
    env = DofbotEnv()
    env.reset()
    Reward = False

    # Gripper angle constants
    GRIPPER_DEFAULT_ANGLE = 20. / 180. * 3.1415
    GRIPPER_CLOSE_ANGLE = -20. / 180. * 3.1415

    # Define state machine
    INITIAL_STATE = 0
    GRASP_STATE = 1
    LIFT_STATE = 2
    PUT_STATE = 3
    MOVE_STATE = 4
    BACK_STATE = 5

    # Position offsets
    POSITION_OFFSETS = {
        'GRASP': np.array([-0.023, -0.023, 0.09]),
        'LIFT': np.array([-0.032, 0.032, 0.13]),
        'PUT': np.array([-0.025, 0.025, 0.09 - 0.085]),
        'MOVE_BACK': np.array([-0.025, 0.025, -0.15])
    }

    # Initial setup
    current_state = INITIAL_STATE
    initial_joint_positions = [1.57, 0.0, 1.57, 1.57, 1.57]
    block_position, block_orientation = env.get_block_pose()
    start_time = time.time()


    def calculate_distance(pos1, pos2):
        """Calculate Euclidean distance between two 3D points."""
        return np.sqrt(np.sum((np.array(pos1) - np.array(pos2)) ** 2))


    def execute_gripper_movement(joint_positions, gripper_angle, repetitions=1):
        """Execute gripper movement with specified parameters."""
        for _ in range(repetitions):
            env.dofbot_control(joint_positions, gripper_angle)


    while not Reward:
        if current_state == INITIAL_STATE:
            # Move to initial position
            execute_gripper_movement(initial_joint_positions, GRIPPER_DEFAULT_ANGLE)
            current_state = GRASP_STATE

        elif current_state == GRASP_STATE:
            # Calculate and move to grasp position
            target_position = np.array(block_position) + POSITION_OFFSETS['GRASP']
            joint_positions = env.dofbot_setInverseKine(target_position)
            execute_gripper_movement(joint_positions, GRIPPER_DEFAULT_ANGLE)

            # Check distance to block
            robot_position, _ = env.get_dofbot_pose()
            block_position, _ = env.get_block_pose()
            grasp_distance = calculate_distance(robot_position, block_position)

            print(f'Grasping - Distance: {grasp_distance:.4f}')

            if 0.075 < grasp_distance < 0.079:
                execute_gripper_movement(joint_positions, GRIPPER_CLOSE_ANGLE)
            elif grasp_distance < 0.075:
                current_state = LIFT_STATE

        elif current_state == LIFT_STATE:
            # Lift block to intermediate position
            lift_target = [0.2, 0.1, 0.2]
            joint_positions = env.dofbot_setInverseKine(lift_target)
            execute_gripper_movement(joint_positions, GRIPPER_CLOSE_ANGLE)

            # Check lift position
            block_position, _ = env.get_block_pose()
            lift_distance = calculate_distance(block_position, lift_target)

            print(f'Lifting - Distance: {lift_distance:.4f}')

            if lift_distance < 0.105:
                execute_gripper_movement(joint_positions, GRIPPER_DEFAULT_ANGLE)
                current_state = PUT_STATE

        elif current_state == PUT_STATE:
            # Move to target position
            target_position = env.get_target_pose()
            put_target = np.array(target_position) + POSITION_OFFSETS['PUT']
            joint_positions = env.dofbot_setInverseKine(put_target)

            robot_position, _ = env.get_dofbot_pose()
            put_distance = calculate_distance(robot_position, put_target)

            print(f'Putting - Distance: {put_distance:.4f}, Target: {put_target}')

            execute_gripper_movement(joint_positions, GRIPPER_CLOSE_ANGLE)
            if put_distance < 0.03:
                current_state = MOVE_STATE

        elif current_state == MOVE_STATE:
            # Move back and release
            target_position = env.get_target_pose()
            final_target = np.array(target_position) + POSITION_OFFSETS['MOVE_BACK']
            joint_positions = env.dofbot_setInverseKine(final_target)
            execute_gripper_movement(joint_positions, GRIPPER_CLOSE_ANGLE)

            robot_position, _ = env.get_dofbot_pose()
            final_distance = calculate_distance(robot_position, final_target)

            print(f'Moving back - Distance: {final_distance:.4f}, Target: {final_target}')

            if final_distance < 0.88:
                # Execute final gripper movements
                execute_gripper_movement(joint_positions, GRIPPER_CLOSE_ANGLE, 100)
                execute_gripper_movement(joint_positions, GRIPPER_DEFAULT_ANGLE, 100)

                if Reward:
                    print('Task completed successfully!')
                    break

        Reward = env.reward()
