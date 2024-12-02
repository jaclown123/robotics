

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


    '''
    constants here
    '''
    GRIPPER_DEFAULT_ANGLE = 20. / 180. * 3.1415
    GRIPPER_CLOSE_ANGLE = -20. / 180. * 3.1415

    # define state machine
    INITIAL_STATE = 0
    GRASP_STATE = 1
    LIFT_STATE = 2
    PUT_STATE = 3
    MOVE_STATE = 4
    BACK_STATE = 5
    current_state = INITIAL_STATE


    initial_jointposes = [1.57, 0., 1.57, 1.57, 1.57]

    # offset to grasp object
    obj_offset = [-0.023, -0.023, 0.09]
    obj_offset2 = [-0.032, 0.032, 0.13]
    obj_offset3 = [-0.025, 0.025, 0.09-0.085]
    obj_offset4 = [-0.025, 0.025, -0.15]
    block_pos, block_orn = env.get_block_pose()

    start_time = time.time()

    while not Reward:



        '''
        #获取物块位姿、目标位置和机械臂位姿，计算机器臂关节和夹爪角度，使得机械臂夹取绿色物块，放置到紫色区域。
        '''

        '''
        code here
        '''
        if current_state == INITIAL_STATE:
            # Move to initial position
            jointPoses = initial_jointposes
            env.dofbot_control(jointPoses, GRIPPER_DEFAULT_ANGLE)
            current_state = GRASP_STATE
        elif current_state == GRASP_STATE:
            # Move to block position

            target_pos = np.array(block_pos) + np.array(obj_offset)
            target_orn = block_orn
            jointPoses = env.dofbot_setInverseKine(target_pos)
            env.dofbot_control(jointPoses, GRIPPER_DEFAULT_ANGLE)
            pos_t, orn_t = env.get_dofbot_pose()
            pos_b, orn_b = env.get_block_pose()
            distance0 = np.sqrt((pos_t[0] - pos_b[0]) ** 2 + (pos_t[1] - pos_b[1]) ** 2)
            print('now grasp', distance0)
            if 0.008 > distance0 > 0.001:
                env.dofbot_control(jointPoses, GRIPPER_CLOSE_ANGLE)
                #env.dofbot_control(jointPoses, GRIPPER_CLOSED_ANGLE)
            elif distance0 < 0.001:
                current_state = LIFT_STATE
        elif current_state == LIFT_STATE:
            # Lift the block
            target_pos = [0.2, 0.1, 0.2]
            jointPoses = env.dofbot_setInverseKine(target_pos)
            env.dofbot_control(jointPoses, GRIPPER_CLOSE_ANGLE)
            jointPoses_t = env.get_dofbot_jointPoses()[0]
            # if jointPoses_t == jointPoses:
            #     env.dofbot_control(jointPoses, GRIPPER_DEFAULT_ANGLE)
            #     current_state = PUT_STATE
            pos_t, orn_t = env.get_block_pose()
            distance1 = np.sqrt((pos_t[0] - target_pos[0]) ** 2 + (pos_t[1] - target_pos[1]) ** 2 + (pos_t[2] - target_pos[2])**2)
            print('now lift', jointPoses, jointPoses_t,target_pos,distance1)
            if distance1 < 0.095:
                 env.dofbot_control(jointPoses, GRIPPER_DEFAULT_ANGLE)
                 current_state = PUT_STATE
        elif current_state == PUT_STATE:
            # Move to put position
            temp_pos = env.get_target_pose()
            put_pos = np.array(temp_pos) + np.array(obj_offset3)
            jointPoses = env.dofbot_setInverseKine(put_pos)
            put_pos_t = env.get_dofbot_pose()[0]
            distance2 = np.sqrt((put_pos_t[0] - put_pos[0]) ** 2 + (put_pos_t[1] - put_pos[1]) ** 2 + (put_pos_t[2] - put_pos[2])**2)
            print('now put', distance2,put_pos)
            env.dofbot_control(jointPoses, GRIPPER_CLOSE_ANGLE)
            if distance2 < 0.03:
                current_state = MOVE_STATE
                #env.dofbot_control(jointPoses, GRIPPER_DEFAULT_ANGLE)
            #current_state = MOVE_STATE
        elif current_state == MOVE_STATE:
            # Move back to initial position
            temp_pos = np.array(env.get_target_pose()) + np.array(obj_offset4)
            jointPoses = env.dofbot_setInverseKine(temp_pos)
            env.dofbot_control(jointPoses, GRIPPER_CLOSE_ANGLE)
            distance3 = np.sqrt((temp_pos[0] - env.get_dofbot_pose()[0][0]) ** 2 + (temp_pos[1] - env.get_dofbot_pose()[0][1]) ** 2 + (temp_pos[2] - env.get_dofbot_pose()[0][2]) ** 2)
            print('now move', distance3, temp_pos, env.get_dofbot_pose()[0])
            if distance3 < 0.03:#0.02887:
                for i in range(100):
                    env.dofbot_control(jointPoses, GRIPPER_CLOSE_ANGLE)
                for i in range(100):
                    env.dofbot_control(jointPoses, GRIPPER_DEFAULT_ANGLE)




        Reward = env.reward()
        if Reward:
            print('success!')
            break