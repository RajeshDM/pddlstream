from collections import namedtuple

import numpy as np

from examples.discrete_tamp.viewer import MAX_COLS, MAX_ROWS
from pddlstream.language.generator import outputs_from_boolean
from icecream import ic

GRASP = np.array([0, 0])


def is_valid(p):
    return np.greater_equal(p, [0, 0]) and np.greater([MAX_COLS, MAX_ROWS], p)


def get_length(vec, ord=1):
    return np.linalg.norm(vec, ord=ord)


def get_difference(p1, p2):
    '''
    ic (p2)
    ic (p1)
    if type(p1) == str :
        #p1 = np.array(list(p1))
        p1 = np.array([int(elem) for elem in p1 if type(elem) == str if elem.isdigit()])
    if type(p2) == str :
        p2 = np.array([int(elem) for elem in p2 if type(elem) == str if elem.isdigit()])
    ic (p2)
    ic (p1)
    ic (type(p2))
    ic (type(p1))
    return np.array(p2) - np.array(p1)
    '''
    return np.array(p2) - p1


def collision_test(p1, p2):
    return get_length(get_difference(p1, p2)) < 1e-3
    #return get_length(get_difference(p1, p2)) > 1e-3


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

def get_shift_one_problem(n_blocks=2, n_poses=9):
    assert(1 <= n_blocks <= n_poses)
    blocks = [BLOCK_TEMPLATE.format(i) for i in range(n_blocks)]
    poses = [np.array([x, 0]) for x in range(n_poses)]

    block_poses = dict(zip(blocks, poses))
    initial = DiscreteTAMPState(INITIAL_CONF, None, block_poses)
    goal_poses = {blocks[0]: poses[1]}
    #goal_poses = {blocks[0]: poses[1],blocks[1]:poses[0]}

    return DiscreteTAMPProblem(initial, poses[n_blocks:], goal_poses)
    #return DiscreteTAMPProblem(initial, poses, goal_poses)

def get_shift_all_problem(n_blocks=2, n_poses=9):
    assert(n_blocks + 1 <= n_poses)
    blocks = [BLOCK_TEMPLATE.format(i) for i in range(n_blocks)]
    poses = [np.array([x, 0]) for x in range(n_poses)]

    block_poses = dict(zip(blocks, poses))
    initial = DiscreteTAMPState(INITIAL_CONF, None, block_poses)
    goal_poses = dict(zip(blocks, poses[1:]))

    return DiscreteTAMPProblem(initial, poses[n_blocks+1:], goal_poses)