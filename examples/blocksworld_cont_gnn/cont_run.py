#!/usr/bin/env python

from __future__ import print_function

import os
import numpy as np
import pddlgym

from pddlstream.algorithms.meta import solve, create_parser
from examples.discrete_tamp_gnn.primitives import GRASP, collision_test, distance_fn, DiscreteTAMPState, \
    get_shift_one_problem,get_shift_all_problem,get_shift_all_problem
from examples.discrete_tamp.viewer import DiscreteTAMPViewer, COLORS
# from pddlstream.algorithms.serialized import solve_serialized
from pddlstream.algorithms.focused import generate_new_facts,initialize_useful_structures
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.language.constants import And, Equal, TOTAL_COST, print_solution, PDDLProblem
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.utils import user_input, read, INF
from pddlstream.utils import generate_pddl_from_init_goal,pddlstream_from_pddlgym,is_action_possible
from PLOI.main import _create_guider,_create_planner
from icecream import ic
from planning import validate_strips_plan#, IncrementalPlanner,pyperplan_planner
import tempfile
import pickle
from planning import PlanningTimeout, PlanningFailure, FD, \
    validate_strips_plan, IncrementalPlanner,pyperplan_planner

# TODO: Can infer domain from usage or from specification

##################################################

def pddlstream_from_tamp(tamp_problem):
    initial = tamp_problem.initial
    assert(initial.holding is None)

    known_poses = list(initial.block_poses.values()) + \
                  list(tamp_problem.goal_poses.values())

    directory = os.path.dirname(os.path.abspath(__file__))
    #ic (directory)
    domain_pddl = read(os.path.join(directory, 'domain.pddl'))
    #ic (domain_pddl)
    stream_pddl = read(os.path.join(directory, 'stream.pddl'))

    q100 = np.array([100, 100])
    constant_map = {
        'q100': q100, # As an example
    }
    p3 = np.array([3,0])
    p4 = np.array([4,0])

    init = [
               #Type(q100, 'conf'),
               ('CanMove',),
               ('Conf', q100),
               ('Conf', initial.conf),
               ('AtConf', initial.conf),
               ('HandEmpty',),
               #('Block', 'b2'),
               #('OnTable', 'b2'),
               #('On', 'b1', 'b2'),
               #('Clear', 'b1'),
               ('Block', 'b3'),
               #('Pose', p3),
               ('Clear','b3'),
               ('Block', 'b4'),
               ('Pose', p4),
               ('AtPose','b4',p4),
               ('OnTable','b4'),
               ('On','b3','b4'),
               ('Kin',p4,p4),
               ('Conf',p4),
               ('Conf',p3),
               Equal((TOTAL_COST,), 0)] + \
           [('Block', b) for b in initial.block_poses.keys()] + \
           [('Clear', b) for b in initial.block_poses.keys()] + \
           [('OnTable', b) for b in initial.block_poses.keys()] + \
           [('Pose', p) for p in known_poses] + \
           [('AtPose', b, p) for b, p in initial.block_poses.items()]
    # [('Pose', p) for p in known_poses + tamp_problem.poses] + \

    goal = And(*[
        #('AtPose', b, p) for b, p in tamp_problem.goal_poses.items()
        ('AtPose', 'b3',p3) for b, p in tamp_problem.goal_poses.items()
    ])

    # TODO: convert to lower case
    #ic (tamp_problem.poses)
    #exit()
    stream_map = {
        #'sample-pose': from_gen_fn(lambda: ((np.array([x, 0]),) for x in range(len(poses), n_poses))),
        'sample-pose': from_gen_fn(lambda: ((p,) for p in tamp_problem.poses)),
        'inverse-kinematics':  from_fn(lambda p: (p + GRASP,)),
        'test-cfree': from_test(lambda *args: not collision_test(*args)),
        #'test-cfree': from_test(lambda *args: collision_test(*args)),
        'collision': collision_test,
        'distance': distance_fn,
    }
    #ic (goal)
    #exit()

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)


def extract_continuous_values(state):

    #continuous_params_locations = {}
    numerical_positions = {"conf":[0],"atconf":[0],"pose":[0],"atpose":[1],"block":[],"unsafe":[0]}

    ic (state)
    for predicate in state.literals:
        if predicate.predicate in numerical_positions:
            #ic (predicate[numerical_positions[fact[0]]])
            ic (predicate.__dict__)

    exit()

def draw_state(viewer, state, colors):
    viewer.clear()
    viewer.draw_environment()
    viewer.draw_robot(*state.conf[::-1])
    for block, pose in state.block_poses.items():
        r, c = pose[::-1]
        viewer.draw_block(r, c, name=block, color=colors[block])
    if state.holding is not None:
        pose = state.conf - GRASP
        r, c = pose[::-1]
        viewer.draw_block(r, c, name=state.holding, color=colors[state.holding])

def apply_action(state, action):
    conf, holding, block_poses = state
    # TODO: don't mutate block_poses?
    name, args = action
    if name == 'move':
        _, conf = args
    elif name == 'pick':
        holding, _, _ = args
        del block_poses[holding]
    elif name == 'place':
        block, pose, _ = args
        holding = None
        block_poses[block] = pose
    elif name == 'push':
        block, _, _, pose, conf = args
        holding = None
        block_poses[block] = pose
    else:
        raise ValueError(name)
    return DiscreteTAMPState(conf, holding, block_poses)

def apply_plan(tamp_problem, plan):
    colors = dict(zip(tamp_problem.initial.block_poses, COLORS))
    viewer = DiscreteTAMPViewer(1, len(tamp_problem.poses), title='Initial')
    state = tamp_problem.initial
    print(state)
    draw_state(viewer, state, colors)
    for action in plan:
        user_input('Continue?')
        state = apply_action(state, action)
        print(state)
        draw_state(viewer, state, colors)
    user_input('Finish?')

##################################################

def collect_data(pddlstream_problem,algorithm,unit):

    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = pddlstream_problem
    ic (init)
    ic (goal)
    solution = solve(pddlstream_problem, algorithm=algorithm, unit_costs=unit, debug=False)
    plan, cost, evaluations = solution
    ic (plan)
    #ic ("returning from collect data function")
    return
    #exit()
    domain_name = 'Discrete_tamp'
    domain_name_test = 'Discrete_tampTest'
    starting_problem_idx = 0
    file_number = 0
    starting_init_goal_pddl = generate_pddl_from_init_goal(init, goal, 'discrete_tamp')
    path = "/Users/rajesh/anaconda3/envs/35_vision_2/lib/python3.7/site-packages/pddlgym/pddl/discrete_tamp/"
    init_goal_pddl = generate_pddl_from_init_goal(init, goal, 'discrete_tamp')
    f = open(path + "problem0" + str(file_number) + ".pddl", "w")
    f.write(init_goal_pddl)
    f.close()
    problem_idx = 1
    file_number = 1
    #f = open(path + "problem0" + str(file_number) + ".pddl", "w")
    #f.close()
    env = pddlgym.make("PDDLEnv{}-v0".format(domain_name))
    old_plan = None
    num_iterations = 0
    new_state = None
    pddlgym_plan = []
    training_data = []
    ic (init)


    #for action in plan :
    while True :
        num_iterations += 1
        plan,pddlstream_action = get_pddlstream_action(pddlstream_problem,algorithm,unit,old_plan)
        ic (pddlstream_action)
        ic (pddlstream_action.name)
        ic (pddlstream_action.args)
        gym_state = get_pddlgym_state(file_number,init,goal,problem_idx,env,new_state)

        action_to_take = is_action_possible(env,pddlstream_action,gym_state)
        #ic (action_to_take)

        if action_to_take != None:
            new_state = env.step(action_to_take)[0]
            old_plan = plan[1:]
            pddlgym_plan.append(action_to_take)
            training_data.append(new_state,action_to_take)
        else :
            old_plan = None
            new_state = None
            num_iterations = eager_calls = 0
            evaluations, goal_exp, domain, externals, eager_externals, positive_externals, has_optimizers, store = \
                initialize_useful_structures(pddlstream_problem, stream_info={}, constraints=PlanConstraints(), unit_costs=False, unit_efforts=False, max_time=INF,
                                             success_cost=INF, max_memory=INF, verbose=False)

            ic (evaluations)
            ic (goal_exp)
            ic (externals)
            complexity_limit = 0
            eager_externals = list(filter(lambda e: e.info.eager, externals))
            effort_weight = None
            eager_disabled = (effort_weight is None)  # No point if no stream effort biasing

            init = generate_new_facts(evaluations, domain, max_time=INF, success_cost=INF, max_memory=INF,
                                      has_optimizers=has_optimizers, positive_externals=externals, eager_disabled=eager_disabled, eager_externals=eager_externals,
                                      complexity_limit=complexity_limit, max_effort=INF, goal_exp = goal, verbose=False)

            pddlstream_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map,init, goal)
            #solution = solve(pddlstream_problem, algorithm=algorithm, unit_costs=unit, debug=False)
            #plan, cost, evaluations = solution
            #ic (plan)
            #exit()
            #break
        if validate_strips_plan(
                domain_file=env.domain.domain_fname,
                problem_file=env.problems[starting_problem_idx].problem_fname,
                plan=pddlgym_plan):
            ic ("Plan executed in PDDLGym successfully")
            break
        if num_iterations > 10:
            break

    return training_data
    #exit()




def get_pddlgym_state(file_number, init,goal, problem_idx,env,new_state):
    if new_state != None :
        return new_state

    ic (init)
    ic (goal)
    path = "/Users/rajesh/anaconda3/envs/35_vision_2/lib/python3.7/site-packages/pddlgym/pddl/discrete_tamp/"
    init_goal_pddl = generate_pddl_from_init_goal(init, goal, 'discrete_tamp')
    f = open(path + "problem0" + str(file_number) + ".pddl", "w")
    f.write(init_goal_pddl)
    f.close()
    ic (init_goal_pddl)
    #exit()
    env.fix_problem_index(problem_idx)
    state, _ = env.reset()
    return state

def get_pddlstream_action(pddlstream_problem, algorithm, unit,old_plan):
    if old_plan is None :
        solution = solve(pddlstream_problem, algorithm=algorithm, unit_costs=unit, debug=False)
        plan, cost, evaluations = solution
        action = plan[0]
        return plan,action
    else :
        return old_plan[0]



def collect_pddl_stream_data(pddlstream_problem,algorithm,unit,initial_state,domain_name=None):
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = pddlstream_problem
    #ic (pddlstream_problem)
    ic (init)
    ic (len(init))
    '''
    solution = solve(pddlstream_problem, algorithm=algorithm, unit_costs=unit, debug=False)
    plan, cost, evaluations = solution
    ic (plan)
    exit()
    '''
    domain_name = 'Discrete_tamp'
    domain_name_test = 'Discrete_tampTest'
    init_goal_pddl = generate_pddl_from_init_goal(init, goal,'discrete_tamp')
    #ic (unit)
    #exit()
    solution = solve(pddlstream_problem, algorithm=algorithm, unit_costs=unit, debug=False,
                     #complexity_step=INF, max_complexity=0,
                     )
    plan, cost, evaluations = solution
    ic (plan)
    #exit()
    #init_goal_pddl_str = generate_pddl_from_init_goal(init_str, goal_str,'discrete_tamp')
    #ic (init,init_str)
    #ic (init_goal_pddl)
    #ic (init_goal_pddl_str)
    #ic (domain_pddl)
    #exit()
    problem_idx = 0
    file_number = 0
    path = "/Users/rajesh/anaconda3/envs/35_vision_2/lib/python3.7/site-packages/pddlgym/pddl/discrete_tamp/"
    f = open(path+"problem0" +str(file_number) +".pddl", "w")
    f.write(init_goal_pddl)
    #f.write(init_goal_pddl_str)
    f.close()
    env = pddlgym.make("PDDLEnv{}-v0".format(domain_name))
    #action_space = env.action_space._action_predicate_to_operators
    env.fix_problem_index(problem_idx)
    state, _ = env.reset()
    #ic (action_space)
    #ic (state)
    #init = pddlstream_from_pddlgym(state)
    #init.append(("Pose",np.array([1,0])))
    #ic (init)
    #ic (len(init))
    #exit()
    ic (initial_state)
    groundings = env.action_space.all_ground_literals(state)
    ic (groundings)
    groundings_list = []
    #ic (type(groundings))
    for grounding in groundings :
        action = grounding.predicate
        objects = grounding.variables
        groundings_list.append(pddlgym.structs.Literal(action,objects))
    #exit()
    ic (groundings_list)
    new_action = groundings_list[1]
    state = env.step(new_action)

    init = pddlstream_from_pddlgym(state[0])
    ic (init)
    ic (len(init))
    pddlstream_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = pddlstream_problem
    ic (init)
    ic (len(init))
    #ic (pddlstream_problem)
    #exit()
    solution = solve(pddlstream_problem, algorithm=algorithm, unit_costs=unit, debug=False,
                     #complexity_step=INF, max_complexity=0,
                     )
    plan, cost, evaluations = solution
    ic (plan)
    exit()

    #ic (solution)
    state = initial_state
    #ic ()

    for action in plan:
        state = apply_action(state,action)
        ic (state)

    #ic (init)
    #ic (init_str)
    #ic (init_goal_pddl_str)
    #ic (solution)
    ic (solution.certificate.all_facts)
    exit()

def pddlstream_solve(state,env):
    pddlstream_init, goal = pddlstream_from_pddlgym(state)
    #constant_map = None
    q100 = np.array([100, 100])
    constant_map = {
        'q100': q100,
    }
    poses = [elem[1] for elem in pddlstream_init if elem[0] == "Pose"]
    poses_2 = [elem[1] for elem in goal if elem[0] == "Pose"]
    poses = poses + poses_2
    ic(poses)
    stream_map = {
        # 'sample-pose': from_gen_fn(lambda: ((np.array([x, 0]),) for x in range(len(poses), n_poses))),
        'sample-pose': from_gen_fn(lambda: ((p,) for p in poses)),
        'inverse-kinematics': from_fn(lambda p: (p + GRASP,)),
        'test-cfree': from_test(lambda *args: not collision_test(*args)),
        # 'test-cfree': from_test(lambda *args: collision_test(*args)),
        'collision': collision_test,
        'distance': distance_fn,
    }
    #directory = os.path.dirname(
    #    '/Users/rajesh/Rajesh/Subjects/Research/affordance_learning/full_repos/6_pddl_stream/pddlstream/examples/discrete_tamp_gnn/')
    directory = os.path.dirname(os.path.abspath(__file__))
    domain_pddl = read(os.path.join(directory, 'domain.pddl'))
    stream_pddl = read(os.path.join(directory, 'stream.pddl'))
    ic(pddlstream_init)
    ic(goal)
    # exit()
    pddlstream_problem = PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, pddlstream_init, goal)
    # pddlstream_problem = get_pddlstream_problem(init) #PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)
    solution = solve(pddlstream_problem, algorithm='adaptive', unit_costs=False, debug=False)
    '''
    solution = solve_adaptive(
        pddlstream_problem, constraints=PlanConstraints(),
        stream_info={}, replan_actions=set(),
        unit_costs=False, success_cost=INF,
        max_time=INF, max_iterations=INF, max_memory=INF,
        initial_complexity=0, complexity_step=1,
        max_complexity=INF,
        max_skeletons=INF, search_sample_ratio=1,
        # bind=bind, max_failures=max_failures,
        unit_efforts=False, max_effort=INF, effort_weight=None, reorder=True,
        visualize=False, verbose=True)
    '''

    plan, cost, evaluations = solution
    pddlgym_plan = []
    for pddlstream_action in plan:
        # action_to_take =
        action_to_take = is_action_possible(env, pddlstream_action, state)
        pddlgym_plan.append(action_to_take)



def collect_pddlgym_training_data( num_train_problems,train_env_name):
    """Returns X, Y where X are States and Y are sets of objects
    """
    ic (train_env_name)
    dataset_file_prefix = '/Users/rajesh/Rajesh/Subjects/Research/affordance_learning/PLOI/model/training_data'
    outfile = dataset_file_prefix + "_{}.pkl".format(train_env_name)
    env = pddlgym.make("PDDLEnv{}-v0".format(train_env_name))
    #ic (env.domain.domain_name)
    dom_file = tempfile.NamedTemporaryFile(delete=False).name
    env.domain.write(dom_file)
    #if not self._load_dataset_from_file or not os.path.exists(outfile):
    if not os.path.exists(outfile):
        inputs = []
        outputs = []
        plans = []
        groundings = []
        action_space = env.action_space
        #ic (action_space)
        #ic (action_space.__dict__)
        #ic (action_space._lit_valid_test.__dict__)
        assert env.operators_as_actions
        for idx in range(min(num_train_problems, len(env.problems))):
            idx += 4
            ic(env.problems[idx].problem_fname)
            state_sequence = []
            print("Collecting training data problem {}".format(idx),
                  flush=True)
            env.fix_problem_index(idx)
            state, _ = env.reset()
            #ic (env.action_space.all_ground_literals(state))
            try:
                #plan = env.action_space.all_ground_literals(state)[]
                #plan = self._planner(env.domain, state, timeout=60)
                #pddlstream_state = #pddlstream.examples.discrete_tamp_gnn.run.pddlstream_from_pddlgym(state)
                pddlgym_plan = pddlstream_solve(state,env)

            except (PlanningTimeout, PlanningFailure):
                #ic (plan)
                print("Warning: planning failed, skipping: {}".format(
                    env.problems[idx].problem_fname))
                continue
            plan = pddlgym_plan[:]
            ic (plan)
            #ic (plan[0])
            #ic (type(plan[0]))
            state_grounding = []
            objects_in_plan = {o for act in plan for o in act.variables}
            #ic (list(objects_in_plan)[0])
            #ic (type(list(objects_in_plan)[0]))

            #ic (state)
            state_sequence.append(state)
            state_grounding.append(env.action_space.all_ground_literals(state))
            for action in plan:
                new_state = env.step(action)
                state_sequence.append(new_state[0])
                state_grounding.append(env.action_space.all_ground_literals(new_state[0]))

            #inputs.append(state)
            inputs.append(state_sequence)
            outputs.append(objects_in_plan)
            plans.append(plan)
            groundings.append(state_grounding)
        training_data = (inputs, outputs,plans,action_space._action_predicate_to_operators,groundings)
        #ic (training_data)

        with open(outfile, "wb") as f:
            pickle.dump(training_data, f)

    with open(outfile, "rb") as f:
        training_data = pickle.load(f)

    #ic (training_data)
    #ic (len(training_data))
    #ic (len(training_data[0][1:49]))
    #ic (len(training_data[1][1:49]))
    #ic (len(training_data[2][1:49]))
    #ic (len(training_data[3]))
    #exit()
    return training_data,dom_file,env.domain.domain_name

def main():
    parser = create_parser()
    #parser.add_argument('-p', '--problem', default='blocked', help='The name of the problem to solve')
    args = parser.parse_args()
    #args.algorithm = 'incremental'
    #args.algorithm = 'focused'
    #args.algorithm = 'focused'
    print('Arguments:', args)

    problem_fn = get_shift_one_problem  # get_shift_one_problem | get_shift_all_problem # TODO: use --problem
    #problem_fn = get_shift_all_problem
    #tamp_problem = problem_fn()
    #for num_objects in range(2,5):
    shift_sizes = [1,2,3]
    max_problem_size = 5
    #shift_sizes = [1]
    problem_sizes = [i for i in range(2,max_problem_size+1)]
    start_points = [0,1,2,3,4]
    start_points  = [0]
    problem_sizes = [3]
    shift_sizes = [1]
    for direction in ['right']:
        for shift_size in shift_sizes:
            for start_point in start_points:
                for num_objects in problem_sizes:
                    #num_objects = 6
                    num_locations = num_objects +shift_size + start_point
                    tamp_problem = problem_fn(num_objects,num_locations,shift_size,direction,start_point)
                    #print(tamp_problem)
                    initial_state = tamp_problem.initial

                    pddlstream_problem = pddlstream_from_tamp(tamp_problem)
                    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = pddlstream_problem
                    collect_data(pddlstream_problem, args.algorithm, args.unit)
                    #ic("Done collecting data for one", num_objects)
                    #ic("Out of the for loop", num_objects)
                    #ic (init)
                    #ic (goal)
                ic ("#####################################")
            ic ("****************************************")
        ic ("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
    exit()

    #collect_pddl_stream_data(pddlstream_problem,args.algorithm,args.unit,init_str,goal_str,initial_state)
    #collect_pddlgym_training_data(3,'Discrete_tamp')
    #exit()
    exit()
    collect_pddl_stream_data(pddlstream_problem,args.algorithm,args.unit,initial_state)

    #solution = solve_serialized(pddlstream_problem, planner='max-astar', unit_costs=args.unit)
    solution = solve(pddlstream_problem, algorithm=args.algorithm, unit_costs=args.unit, debug=False,
                     #complexity_step=INF, max_complexity=0,
                     )

    init_goal_pddl = generate_pddl_from_init_goal(init, goal)
    ic (init_goal_pddl)
    # obj_scores = get_relevant_objects(init_goal_pddl,1)
    sorted_action_scores = get_relevant_objects(init_goal_pddl, 1, state, action_space)
    #ic (domain_pddl)
    print_solution(solution)
    #ic (pddlstream_problem)
    ic (domain_pddl)
    ic (constant_map)
    ic (stream_pddl)
    ic (stream_map)
    ic (init)
    ic (goal)
    ic (init)
    exit()
    plan, cost, evaluations = solution
    if plan is None:
        return
    apply_plan(tamp_problem, plan)


if __name__ == '__main__':
    main()
