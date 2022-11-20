from collections import namedtuple

import numpy as np
from icecream import ic
import random

from examples.discrete_tamp.viewer import MAX_COLS, MAX_ROWS
from pddlstream.language.generator import outputs_from_boolean

GRASP = np.array([0, 0])


def is_valid(p):
    return np.greater_equal(p, [0, 0]) and np.greater([MAX_COLS, MAX_ROWS], p)


def get_length(vec, ord=1):
    return np.linalg.norm(vec, ord=ord)


def get_difference(p1, p2):
    return np.array(p2) - p1


def collision_test(p1, p2):
    return get_length(get_difference(p1, p2)) < 1e-3


def noisy_collision_gen_fn(*args):
    while True:
        if np.random.random() < 0.75:
            yield outputs_from_boolean(False)
        else:
            yield outputs_from_boolean(not collision_test(*args))


def distance_fn(q1, q2):
    return get_length(get_difference(q1, q2))

##################################################

DiscreteTAMPState = namedtuple('DiscreteTAMPState', ['conf', 'holding', 'block_poses'])
DiscreteTAMPProblem = namedtuple('DiscreteTAMPProblem', ['initial', 'poses', 'goal_poses'])
BLOCK_TEMPLATE = 'b{}'
INITIAL_CONF = np.array([0, -1])


def get_k_shift_problem_3d(n_blocks=2, n_poses=15,shift_size=1,right_left="right",start_point=0,k=1):
    assert(n_blocks + shift_size <= n_poses)
    if right_left == "right":
        blocks = [BLOCK_TEMPLATE.format(i) for i in range(n_blocks)]
    else :
        blocks = [BLOCK_TEMPLATE.format(i) for i in range(n_poses-n_blocks,n_poses)]
    poses = [np.array([x, 0]) for x in range(start_point,n_poses)]
    #goal_poses = [np.array([x, 0]) for x in range(start_point,n_poses)]
    random.seed(10)

    tamp_3d = True
    #goal_poses = {}
    goal_poses = {'on':[]}
    max_blocks_to_move = k
    if tamp_3d == True :
        #all_blocks = ['b'+ str(i) for i in range(n_blocks) ]
        all_blocks = [BLOCK_TEMPLATE.format(i) for i in range(n_blocks)]
        top_blocks = []
        bottom_blocks = []
        if len(all_blocks) > 1 :
            number_random = int (len(all_blocks)/4)
            if len(all_blocks) == 3 :
                number_random = 1
            for i in range(0,number_random):
                block1_pos = random.randint(0,len(all_blocks)-1)
                #block2_pos = random.randint(0,len(all_blocks)-2)
                while True :
                    block2_pos = random.randint(0,len(all_blocks)-2)
                    if all_blocks[block2_pos] not in bottom_blocks :
                        bottom_blocks.append(all_blocks[block2_pos])
                        break

                if block1_pos < k :
                    max_blocks_to_move += 1

                block_1 = all_blocks[block1_pos]
                top_blocks.append(block_1)
                all_blocks.remove(block_1)
                block_2 = all_blocks[block2_pos]
                #goal_poses['on'] = [block_1,block_2]
                block_combinations = (block_1,block_2)
                goal_poses['on'].append(block_combinations)

            #goal_poses = {'on': ['b0', 'b1']}
    #ic (poses)

    block_poses = dict(zip(blocks, poses))
    #ic (block_poses)
    initial = DiscreteTAMPState(INITIAL_CONF, None, block_poses)
    #ic (blocks)
    #ic (poses)
    if right_left == "right":
        #goal_poses = dict(zip(blocks[:k], poses[shift_size:k+shift_size]))
        #goal_poses = dict(zip(blocks[:k], goal_poses[shift_size:k+shift_size]))
        goal_poses['atpose'] = [(blocks[i],poses[shift_size+i]) for i in range (0,min(n_blocks,max_blocks_to_move)) if BLOCK_TEMPLATE.format(i) not in top_blocks]
        #goal_poses['atpose'] = [(blocks[i],poses[shift_size+i]) for i in range (0,n_blocks) if BLOCK_TEMPLATE.format(i) not in top_blocks]
        #ic (goal_poses)
        #exit()
        #goal_poses = goal_poses
        #ic (goal_poses)
        #exit()
    else :
        goal_poses = dict(zip(blocks, poses[n_poses-(n_blocks+shift_size):]))
    #ic (goal_poses)
    #exit()
    '''
    '''
    #exit()
    #return DiscreteTAMPProblem(initial, poses[n_blocks+1:], goal_poses)
    return DiscreteTAMPProblem(initial, poses[n_blocks:], goal_poses)

def get_shift_one_problem(n_blocks=2, n_poses=9,shift_size=1,right_left="right",start_point=0):
    assert(1 <= n_blocks + shift_size <= n_poses)
    blocks = [BLOCK_TEMPLATE.format(i) for i in range(n_blocks)]
    #poses = [np.array([x, 0]) for x in range(n_poses)]
    poses = [np.array([x, 0]) for x in range(start_point,n_poses)]

    block_poses = dict(zip(blocks, poses))
    initial = DiscreteTAMPState(INITIAL_CONF, None, block_poses)
    goal_poses = {blocks[0]: poses[shift_size]}
    #goal_poses = {blocks[0]: poses[1],blocks[1]:poses[0]}
    #ic (block_poses)
    #ic (goal_poses)
    #ic (poses[n_blocks:])
    return DiscreteTAMPProblem(initial, poses[n_blocks:], goal_poses)
    #return DiscreteTAMPProblem(initial, poses, goal_poses)

def get_k_shift_problem(n_blocks=2, n_poses=15,shift_size=1,right_left="right",start_point=0,k=1):
    assert(n_blocks + shift_size <= n_poses)
    if right_left == "right":
        blocks = [BLOCK_TEMPLATE.format(i) for i in range(n_blocks)]
    else :
        blocks = [BLOCK_TEMPLATE.format(i) for i in range(n_poses-n_blocks,n_poses)]
    poses = [np.array([x, 0]) for x in range(start_point,n_poses)]
    goal_poses = [np.array([x, 1]) for x in range(start_point,n_poses)]
    #ic (poses)

    block_poses = dict(zip(blocks, poses))
    #ic (block_poses)
    initial = DiscreteTAMPState(INITIAL_CONF, None, block_poses)
    #ic (blocks)
    #ic (poses)
    if right_left == "right":
        goal_poses = dict(zip(blocks[:k], goal_poses[shift_size:k+shift_size]))
    else :
        goal_poses = dict(zip(blocks, poses[n_poses-(n_blocks+shift_size):]))

    #ic (poses)
    #ic (goal_poses)
    #ic (n_blocks)
    #ic (poses[n_blocks:])
    #exit()
    #return DiscreteTAMPProblem(initial, poses[n_blocks+1:], goal_poses)
    return DiscreteTAMPProblem(initial, poses[n_blocks:], goal_poses)

def get_shift_all_problem(n_blocks=2, n_poses=9,shift_size=1,right_left="right",start_point=0):
    assert(n_blocks + shift_size <= n_poses)
    if right_left == "right":
        blocks = [BLOCK_TEMPLATE.format(i) for i in range(n_blocks)]
    else :
        blocks = [BLOCK_TEMPLATE.format(i) for i in range(n_poses-n_blocks,n_poses)]
    poses = [np.array([x, 0]) for x in range(start_point,n_poses)]
    #ic (poses)

    block_poses = dict(zip(blocks, poses))
    #ic (block_poses)
    initial = DiscreteTAMPState(INITIAL_CONF, None, block_poses)
    if right_left == "right":
        goal_poses = dict(zip(blocks, poses[shift_size:]))
    else :
        goal_poses = dict(zip(blocks, poses[n_poses-(n_blocks+shift_size):]))

    return DiscreteTAMPProblem(initial, poses[n_blocks+1:], goal_poses)