from __future__ import print_function

import copy

from collections import defaultdict, namedtuple

from pddlstream.algorithms.downward import get_problem, task_from_domain_problem, get_cost_scale, \
    conditions_hold, apply_action, scale_cost, fd_from_fact, make_domain, make_predicate, evaluation_from_fd, \
    plan_preimage, fact_from_fd, USE_FORBID, pddl_from_instance, parse_action,is_valid_plan
from pddlstream.algorithms.instantiate_task import instantiate_task, sas_from_instantiated, FD_INSTANTIATE
from pddlstream.algorithms.scheduling.add_optimizers import add_optimizer_effects, \
    using_optimizers, recover_simultaneous
from pddlstream.algorithms.scheduling.apply_fluents import convert_fluent_streams
from pddlstream.algorithms.scheduling.negative import recover_negative_axioms, convert_negative
from pddlstream.algorithms.scheduling.postprocess import postprocess_stream_plan
from pddlstream.algorithms.scheduling.recover_axioms import recover_axioms_plans
from pddlstream.algorithms.scheduling.recover_functions import compute_function_plan
from pddlstream.algorithms.scheduling.recover_streams import get_achieving_streams, extract_stream_plan, \
    evaluations_from_stream_plan
from pddlstream.algorithms.scheduling.stream_action import add_stream_actions
from pddlstream.algorithms.scheduling.utils import partition_results, \
    add_unsatisfiable_to_goal, get_instance_facts
from pddlstream.algorithms.search import solve_from_task
from pddlstream.algorithms.advanced import UNIVERSAL_TO_CONDITIONAL
from pddlstream.language.constants import Not, get_prefix, EQ, FAILED, OptPlan, Action
from pddlstream.language.conversion import obj_from_pddl_plan, evaluation_from_fact, \
    fact_from_evaluation, transform_plan_args, transform_action_args, obj_from_pddl
from pddlstream.language.external import Result
from pddlstream.language.exogenous import get_fluent_domain
from pddlstream.language.function import Function
from pddlstream.language.stream import StreamResult
from pddlstream.language.optimizer import UNSATISFIABLE
from pddlstream.language.statistics import compute_plan_effort
from pddlstream.language.temporal import SimplifiedDomain, solve_tfd
from pddlstream.language.write_pddl import get_problem_pddl
from pddlstream.language.object import Object
from pddlstream.language.write_pddl import get_problem_pddl
from pddlstream.utils import Verbose, INF, topological_sort, get_ancestors
import pddlstream
from icecream import ic
from pddlstream.utils import generate_pddl_from_init_goal,get_init_from_evals,update_init_from_negative, \
    add_negative_to_init,get_init_from_opt_evals,pddlgym_plan_to_pddlstream_plan
import numpy as np
from datetime import datetime
import time
import pddlgym
import pickle
from PLOI.main import  _create_planner,_create_guider,IncrementalPlanner,discrepancy_search
from PLOI.planning import validate_strips_plan

RENAME_ACTIONS = False
#RENAME_ACTIONS = not USE_FORBID

OptSolution = namedtuple('OptSolution', ['stream_plan', 'opt_plan', 'cost']) # TODO: move to the below
#OptSolution = namedtuple('OptSolution', ['stream_plan', 'action_plan', 'cost', 'supporting_facts', 'axiom_plan'])

##################################################

def add_stream_efforts(node_from_atom, instantiated, effort_weight, **kwargs):
    if effort_weight is None:
        return
    # TODO: make effort just a multiplier (or relative) to avoid worrying about the scale
    # TODO: regularize & normalize across the problem?
    #efforts = []
    for instance in instantiated.actions:
        # TODO: prune stream actions here?
        # TODO: round each effort individually to penalize multiple streams
        facts = get_instance_facts(instance, node_from_atom)
        #effort = COMBINE_OP([0] + [node_from_atom[fact].effort for fact in facts])
        stream_plan = []
        extract_stream_plan(node_from_atom, facts, stream_plan)
        effort = compute_plan_effort(stream_plan, **kwargs)
        instance.cost += scale_cost(effort_weight*effort)
        # TODO: store whether it uses shared/unique outputs and prune too expensive streams
        #efforts.append(effort)
    #print(min(efforts), efforts)

##################################################

def rename_instantiated_actions(instantiated, rename):
    # TODO: rename SAS instead?
    actions = instantiated.actions[:]
    renamed_actions = []
    action_from_name = {}
    for i, action in enumerate(actions):
        renamed_actions.append(copy.copy(action))
        renamed_name = 'a{}'.format(i) if rename else action.name
        renamed_actions[-1].name = '({})'.format(renamed_name)
        action_from_name[renamed_name] = action # Change reachable_action_params?
    instantiated.actions[:] = renamed_actions
    return action_from_name

##################################################

def get_plan_cost(action_plan, cost_from_action):
    if action_plan is None:
        return INF
    # TODO: return cost per action instance
    #return sum([0.] + [instance.cost for instance in action_plan])
    scaled_cost = sum([0.] + [cost_from_action[instance] for instance in action_plan])
    return scaled_cost / get_cost_scale()

def instantiate_optimizer_axioms(instantiated, domain, results):
    # Needed for instantiating axioms before adding stream action effects
    # Otherwise, FastDownward will prune these unreachable axioms
    # TODO: compute this first and then apply the eager actions
    stream_init = {fd_from_fact(result.stream_fact)
                   for result in results if isinstance(result, StreamResult)}
    evaluations = list(map(evaluation_from_fd, stream_init | instantiated.atoms))
    temp_domain = make_domain(predicates=[make_predicate(UNSATISFIABLE, [])],
                              axioms=[ax for ax in domain.axioms if ax.name == UNSATISFIABLE])
    temp_problem = get_problem(evaluations, Not((UNSATISFIABLE,)), temp_domain)
    # TODO: UNSATISFIABLE might be in atoms making the goal always infeasible
    with Verbose():
        # TODO: the FastDownward instantiation prunes static preconditions
        use_fd = False if using_optimizers(results) else FD_INSTANTIATE
        new_instantiated = instantiate_task(task_from_domain_problem(temp_domain, temp_problem),
                                            use_fd=use_fd, check_infeasible=False, prune_static=False)
        assert new_instantiated is not None
    ic (new_instantiated.axioms)
    ic (new_instantiated.atoms)
    instantiated.axioms.extend(new_instantiated.axioms)
    instantiated.atoms.update(new_instantiated.atoms)

##################################################

def recover_partial_orders(stream_plan, node_from_atom):
    # Useful to recover the correct DAG
    partial_orders = set()
    for child in stream_plan:
        # TODO: account for fluent objects
        for fact in child.get_domain():
            parent = node_from_atom[fact].result
            if parent is not None:
                partial_orders.add((parent, child))
    #stream_plan = topological_sort(stream_plan, partial_orders)
    return partial_orders

def recover_stream_plan(evaluations, current_plan, opt_evaluations, goal_expression, domain, node_from_atom,
                        action_plan, axiom_plans, negative, replan_step):
    # Universally quantified conditions are converted into negative axioms
    # Existentially quantified conditions are made additional preconditions
    # Universally quantified effects are instantiated by doing the cartesian produce of types (slow)
    # Added effects cancel out removed effects
    # TODO: node_from_atom is a subset of opt_evaluations (only missing functions)
    real_task = task_from_domain_problem(domain, get_problem(evaluations, goal_expression, domain))
    opt_task = task_from_domain_problem(domain, get_problem(opt_evaluations, goal_expression, domain))
    negative_from_name = {external.blocked_predicate: external for external in negative if external.is_negated}
    real_states, full_plan = recover_negative_axioms(
        real_task, opt_task, axiom_plans, action_plan, negative_from_name)
    function_plan = compute_function_plan(opt_evaluations, action_plan)
    #ic (function_plan)

    full_preimage = plan_preimage(full_plan, []) # Does not contain the stream preimage!
    negative_preimage = set(filter(lambda a: a.predicate in negative_from_name, full_preimage))
    negative_plan = convert_negative(negative_preimage, negative_from_name, full_preimage, real_states)
    function_plan.update(negative_plan)
    #ic (full_preimage)
    #ic (negative_plan)
    #ic(function_plan)
    # TODO: OrderedDict for these plans

    # TODO: this assumes that actions do not negate preimage goals
    positive_preimage = {l for l in (set(full_preimage) - real_states[0] - negative_preimage) if not l.negated}
    steps_from_fact = {fact_from_fd(l): full_preimage[l] for l in positive_preimage}
    last_from_fact = {fact: min(steps) for fact, steps in steps_from_fact.items() if get_prefix(fact) != EQ}
    #stream_plan = reschedule_stream_plan(evaluations, target_facts, domain, stream_results)
    # visualize_constraints(map(fact_from_fd, target_facts))

    for result, step in function_plan.items():
        for fact in result.get_domain():
            last_from_fact[fact] = min(step, last_from_fact.get(fact, INF))

    # TODO: get_steps_from_stream
    stream_plan = []
    last_from_stream = dict(function_plan)
    #ic (current_plan)
    for result in current_plan: # + negative_plan?
        # TODO: actually compute when these are needed + dependencies
        last_from_stream[result] = 0
        if isinstance(result.external, Function) or (result.external in negative):
            if len(action_plan) > replan_step:
                raise NotImplementedError() # TODO: deferring negated optimizers
            # Prevents these results from being pruned
            function_plan[result] = replan_step
        else:
            stream_plan.append(result)
    #ic (function_plan)

    #ic (stream_plan)
    curr_evaluations = evaluations_from_stream_plan(evaluations, stream_plan, max_effort=None)
    extraction_facts = set(last_from_fact) - set(map(fact_from_evaluation, curr_evaluations))
    extract_stream_plan(node_from_atom, extraction_facts, stream_plan)

    # Recomputing due to postprocess_stream_plan
    stream_plan = postprocess_stream_plan(evaluations, domain, stream_plan, last_from_fact)
    node_from_atom = get_achieving_streams(evaluations, stream_plan, max_effort=None)
    fact_sequence = [set(result.get_domain()) for result in stream_plan] + [extraction_facts]
    #ic (stream_plan)
    #ic (node_from_atom)
    #ic (curr_evaluations)
    #ic (extraction_facts)
    #ic (function_plan)
    for facts in reversed(fact_sequence): # Bellman ford
        for fact in facts: # could flatten instead
            result = node_from_atom[fact].result
            if result is None:
                continue
            step = last_from_fact[fact] if result.is_deferrable() else 0
            last_from_stream[result] = min(step, last_from_stream.get(result, INF))
            for domain_fact in result.instance.get_domain():
                last_from_fact[domain_fact] = min(last_from_stream[result], last_from_fact.get(domain_fact, INF))
    #ic (stream_plan)
    #ic ("before extend")
    stream_plan.extend(function_plan)

    #ic (stream_plan)
    #ic ("in processing")
    partial_orders = recover_partial_orders(stream_plan, node_from_atom)
    bound_objects = set()
    for result in stream_plan:
        if (last_from_stream[result] == 0) or not result.is_deferrable(bound_objects=bound_objects):
            for ancestor in get_ancestors(result, partial_orders) | {result}:
                # TODO: this might change descendants of ancestor. Perform in a while loop.
                last_from_stream[ancestor] = 0
                if isinstance(ancestor, StreamResult):
                    bound_objects.update(out for out in ancestor.output_objects if out.is_unique())

    #local_plan = [] # TODO: not sure what this was for
    #for fact, step in sorted(last_from_fact.items(), key=lambda pair: pair[1]): # Earliest to latest
    #    print(step, fact)
    #    extract_stream_plan(node_from_atom, [fact], local_plan, last_from_fact, last_from_stream)

    # Each stream has an earliest evaluation time
    # When computing the latest, use 0 if something isn't deferred
    # Evaluate each stream as soon as possible
    # Option to defer streams after a point in time?
    # TODO: action costs for streams that encode uncertainty
    state = set(real_task.init)
    remaining_results = list(stream_plan)
    first_from_stream = {}
    #assert 1 <= replan_step # Plan could be empty
    for step, instance in enumerate(action_plan):
        for result in list(remaining_results):
            # TODO: could do this more efficiently if need be
            domain = result.get_domain() + get_fluent_domain(result)
            if conditions_hold(state, map(fd_from_fact, domain)):
                remaining_results.remove(result)
                certified = {fact for fact in result.get_certified() if get_prefix(fact) != EQ}
                state.update(map(fd_from_fact, certified))
                if step != 0:
                    first_from_stream[result] = step
        # TODO: assumes no fluent axiom domain conditions
        apply_action(state, instance)
    #assert not remaining_results # Not true if retrace
    if first_from_stream:
        replan_step = min(replan_step, *first_from_stream.values())

    eager_plan = []
    results_from_step = defaultdict(list)
    for result in stream_plan:
        earliest_step = first_from_stream.get(result, 0) # exogenous
        latest_step = last_from_stream.get(result, 0) # defer
        #assert earliest_step <= latest_step
        defer = replan_step <= latest_step
        if not defer:
            eager_plan.append(result)
        # We only perform a deferred evaluation if it has all deferred dependencies
        # TODO: make a flag that also allows dependencies to be deferred
        future = (earliest_step != 0) or defer
        if future:
            future_step = latest_step if defer else earliest_step
            results_from_step[future_step].append(result)

    # TODO: some sort of obj side-effect bug that requires obj_from_pddl to be applied last (likely due to fluent streams)

    eager_plan = convert_fluent_streams(eager_plan, real_states, action_plan, steps_from_fact, node_from_atom)
    combined_plan = []
    for step, action in enumerate(action_plan):
        combined_plan.extend(result.get_action() for result in results_from_step[step])
        combined_plan.append(transform_action_args(pddl_from_instance(action), obj_from_pddl))

    # TODO: the returned facts have the same side-effect bug as above
    # TODO: annotate when each preimage fact is used
    preimage_facts = {fact_from_fd(l) for l in full_preimage if (l.predicate != EQ) and not l.negated}
    for negative_result in negative_plan: # TODO: function_plan
        preimage_facts.update(negative_result.get_certified())
    for result in eager_plan:
        preimage_facts.update(result.get_domain())
        # Might not be able to regenerate facts involving the outputs of streams
        preimage_facts.update(result.get_certified()) # Some facts might not be in the preimage
    # TODO: record streams and axioms
    return eager_plan, OptPlan(combined_plan, preimage_facts)

##################################################

def solve_optimistic_temporal(domain, stream_domain, applied_results, all_results,
                              opt_evaluations, node_from_atom, goal_expression,
                              effort_weight, debug=False, **kwargs):
    # TODO: assert that the unused parameters are off
    assert domain is stream_domain
    #assert len(applied_results) == len(all_results)
    problem = get_problem(opt_evaluations, goal_expression, domain)
    with Verbose():
        instantiated = instantiate_task(task_from_domain_problem(domain, problem))
    if instantiated is None:
        return instantiated, None, None, INF
    problem = get_problem_pddl(opt_evaluations, goal_expression, domain.pddl)
    pddl_plan, makespan = solve_tfd(domain.pddl, problem, debug=debug, **kwargs)
    if pddl_plan is None:
        return instantiated, None, pddl_plan, makespan
    instance_from_action_args = defaultdict(list)
    for instance in instantiated.actions:
        name, args = parse_action(instance)
        instance_from_action_args[name, args].append(instance)
        #instance.action, instance.var_mapping
    action_instances = []
    for action in pddl_plan:
        instances = instance_from_action_args[action.name, action.args]
        if len(instances) != 1:
            for action in instances:
                action.dump()
        #assert len(instances) == 1 # TODO: support 2 <= case
        action_instances.append(instances[0])
    temporal_plan = obj_from_pddl_plan(pddl_plan) # pddl_plan is sequential
    return instantiated, action_instances, temporal_plan, makespan

def solve_optimistic_sequential(domain, stream_domain, applied_results, all_results,
                                opt_evaluations, node_from_atom, goal_expression,
                                effort_weight, debug=False, **kwargs):
    #print(sorted(map(fact_from_evaluation, opt_evaluations)))
    start_time = time.time()
    temporal_plan = None
    #ic (all_results)
    #ic (applied_results)
    problem = get_problem(opt_evaluations, goal_expression, stream_domain)  # begin_metric
    #ic (problem)
    #ic (domain)
    #ic (domain.type)
    pddl_prob = get_problem_pddl(opt_evaluations, goal_expression, domain.pddl, temporal=False)
    #ic (pddl_prob)
    #exit()

    #ic (stream_domain)
    #ic (opt_evaluations)
    #ic (goal_expression)

    with Verbose(verbose=debug):
        task = task_from_domain_problem(stream_domain, problem)
        #ic(task.__dict__)
        #if task != None :
        #    ic (task.axioms)
        #    ic (task.axioms[0].__dict__)
        instantiated = instantiate_task(task)
        #if instantiated != None :
        #    ic (instantiated.axioms)
    #ic(task.objects)
    obj_pddl_from_args = {}
    #ic (obj_pddl_from_args)

    #if instantiated != None :
    #    ic (instantiated.axioms)
    #for axiom in task.axioms:
    #    ic (axiom.__dict__)
    if instantiated is None:
        #print (" ** returning none due to instantiation benig none ** ")
        return instantiated, None, temporal_plan, INF

    cost_from_action = {action: action.cost for action in instantiated.actions}
    add_stream_efforts(node_from_atom, instantiated, effort_weight)
    #ic (instantiated.axioms)
    if using_optimizers(applied_results):
        add_optimizer_effects(instantiated, node_from_atom)
        # TODO: reachieve=False when using optimizers or should add applied facts
        instantiate_optimizer_axioms(instantiated, domain, all_results)
    #for axiom in task.axioms:
    #    ic (axiom.__dict__)
    #for task_function in task.functions:
    #    ic (task_function.__dict__)
    #ic (instantiated.axioms)
    action_from_name = rename_instantiated_actions(instantiated, RENAME_ACTIONS)
    # TODO: the action unsatisfiable conditions are pruned
    #ic (instantiated.axioms)
    #ic (instantiated.atoms)
    with Verbose(debug):
        sas_task = sas_from_instantiated(instantiated)
        #sas_task.metric = task.use_min_cost_metric
        sas_task.metric = True

    #ic (instantiated)
    # TODO: apply renaming to hierarchy as well
    # solve_from_task | serialized_solve_from_task | abstrips_solve_from_task | abstrips_solve_from_task_sequential
    #ic (task.__dict__)
    renamed_plan, _ = solve_from_task(sas_task, debug=debug, **kwargs)
    end_time = time.time()
    #ic (" * ** ** * * * * renamed plan ** **** ** " )
    init = []
    #ic (task.__dict__.items())
    obj_conversions_pddlstream = {}
    for object in task.objects:
        #ic (obj_from_pddl(object.name))
        obj_conversions_pddlstream[obj_from_pddl(object.name)] = object.name
    #ic (task.goal)
    #ic (obj_conversions_pddlstream)

    goal = str(task.goal)
    goal = goal[goal.find("Atom")+4:]
    goal_predicate = goal[1:goal.find("(")]
    goal_objects = goal[goal.find("(")+1:goal.find(")")]
    goal_objects = goal_objects.replace(" ","")
    goal_objects = goal_objects.split(",")
    if len(goal_objects) != 0:
        if goal_objects[0] != '':
            goal = (tuple([goal_predicate]+goal_objects))
        else:
            goal = (goal_predicate,)
    else:
        goal = (goal_predicate,)

    #ic (goal)
    '''
    ic (sas_task.init.__dict__)
    '''
    #ic (instantiated)
    #ic (instantiated)
    #ic (instantiated.task.atoms)
    #ic (sas_task.__dict__)
    '''
    for elem in sas_task.axioms:
        ic (elem)
        ic (elem.__dict__)
        #ic (elem.expression.__dict__)
        #ic (elem.fluent.__dict__)
    for elem in sas_task.operators:
        ic (elem)
        ic (elem.__dict__)
    for elem in task.init:
        ic (elem)
        if hasattr(elem,'pddl'):
            ic (elem.pddl)
        if bool (elem.__dict__):
            #ic (elem.__dict__)
            ic (elem.expression.__dict__)
            ic (elem.fluent.__dict__)
            #ic (elem.name)

    if renamed_plan is not None:
        for elem in task.init:
            if not bool(elem.__dict__):
                elem_string = str(elem)
                #ic (elem_string)
                new_elem_string = elem_string[elem_string.find("Atom")+4:]
                predicate = new_elem_string[1:new_elem_string.find("(")]
                #ic (predicate)
                #if "cfree" in predicate:
                #    predicate = "not cfree"

                if predicate != "=" and predicate != "identical":
                    objects = new_elem_string[new_elem_string.find("(") + 1:new_elem_string.find(")")]
                    objects = objects.replace(" ","")
                    objects = objects.split(",")
                    if len(objects) != 0:
                        if objects[0] != '' :
                            init.append(tuple([predicate]+objects))
                        else:
                            init.append((predicate,))
                    else:
                        init.append((predicate,))
                    #for object in objects:
                    #    #init.append([predicate,objects])

        #init = add_negative_to_init(init,negative)
        ic (init)
        init_goal_pddl = generate_pddl_from_init_goal(init, goal, 'discrete_tamp')

        path = "/Users/rajesh/anaconda3/envs/35_vision_2/lib/python3.7/site-packages/pddlgym/pddl/discrete_tamp/"
        file_number = 11
        f = open(path + "problem0" + str(file_number) + ".pddl", "w")
        f.write(init_goal_pddl)
        f.close()
        #exit()

    '''
    #ic (init)

    #ic (" * ** ** * * * * renamed plan ** **** ** " , renamed_plan)
    if renamed_plan is None:
        return instantiated, None, temporal_plan, INF

    #ic ("time taken for planning",end_time-start_time)
    action_instances = [action_from_name[name if RENAME_ACTIONS else '({} {})'.format(name, ' '.join(args))]
                        for name, args in renamed_plan]
    #ic("Plan validity")
    #plan_validity = is_valid_plan(task.init+task.axioms, action_instances)
    #ic (renamed_plan)
    #ic (action_instances)
    cost = get_plan_cost(action_instances, cost_from_action)
    return instantiated, action_instances, temporal_plan, cost

def solve_optimistic_learned(domain, stream_domain, applied_results, all_results,
                                opt_evaluations, node_from_atom, goal_expression,
                                effort_weight, obj_conversions,domain_name, debug=False, **kwargs):
    #print(sorted(map(fact_from_evaluation, opt_evaluations)))
    #ic (node_from_atom)
    planner_helper_build = time.time()
    temporal_plan = None
    problem = get_problem(opt_evaluations, goal_expression, stream_domain)  # begin_metric
    #ic (problem.__dict__)
    with Verbose(verbose=debug):
        task = task_from_domain_problem(stream_domain, problem)
        instantiated = instantiate_task(task)
    #ic(task.__dict__)
    if instantiated is None:
        print (" ** returning none due to instantiation benig none ** ")
        return instantiated, None, temporal_plan, INF

    cost_from_action = {action: action.cost for action in instantiated.actions}
    add_stream_efforts(node_from_atom, instantiated, effort_weight)
    if using_optimizers(applied_results):
        add_optimizer_effects(instantiated, node_from_atom)
        # TODO: reachieve=False when using optimizers or should add applied facts
        instantiate_optimizer_axioms(instantiated, domain, all_results)
    action_from_name = rename_instantiated_actions(instantiated, RENAME_ACTIONS)
    # TODO: the action unsatisfiable conditions are pruned
    sas_task_time = time.time()
    with Verbose(debug):
        sas_task = sas_from_instantiated(instantiated)
        #sas_task.metric = task.use_min_cost_metric
        sas_task.metric = True
    ic ("Planner b4 time" , time.time()-planner_helper_build)
    #ic (instantiated)
    # TODO: apply renaming to hierarchy as well
    # solve_from_task | serialized_solve_from_task | abstrips_solve_from_task | abstrips_solve_from_task_sequential
    #ic (task.__dict__)
    obj_conversions_pddlstream = {}
    for object in task.objects:
        #ic (obj_from_pddl(object.name))
        obj_conversions_pddlstream[obj_from_pddl(object.name)] = object.name
    #path = "/Users/rajesh/anaconda3/envs/35_vision_2/lib/python3.7/site-packages/pddlgym/pddl/discrete_tamp_test/"
    #init_goal_pddl = generate_pddl_from_task(task, 'discrete_tamp')
    #f = open(path + "problem0" + str(file_number) + ".pddl", "w")
    #f.write(init_goal_pddl)
    #f.close()
    #start_time = time.time()
    #domain_name = 'Discrete_tamp'
    #test_env_name = "Discrete_tampTest"
    test_env_name = domain_name + "Test"

    env = pddlgym.make("PDDLEnv{}-v0".format(test_env_name))
    idx = -1
    env.fix_problem_index(idx)
    #ic(env.problems[idx].problem_fname)
    action_space = env.action_space
    state, _ = env.reset()
    curr_plan_states = []
    new_plan = []
    ensemble = False
    max_plan_length_permitted = 50
    seed = 0
    is_strips_domain = True
    test_planner_name = 'fd-lama-first'
    planner = _create_planner(test_planner_name,debug)

    num_train_problems = 1
    num_epochs = 20
    train_planner_name = test_planner_name
    guider_name = 'gnn-bce-10'
    #guider_name = 'Discrete_tamp_seed_0'
    guider = _create_guider(guider_name, train_planner_name,
                            num_train_problems, is_strips_domain,
                            num_epochs, seed,250,2)
    guider.seed(seed)
    guider.train(domain_name)
    test_planner = IncrementalPlanner(
        is_strips_domain=is_strips_domain,
        base_planner=planner, search_guider=guider, seed=seed)

    start_time = time.time()
    while True:
        #grounding_time = time.time()
        groundings = env.action_space.all_ground_literals(state)
        action_space = env.action_space
        #ic ("grounding time", time.time()-grounding_time)
        prev_actions = None
        prev_graph = None
        inference_time = time.time()
        action_param_list, prev_graph = test_planner._guidance.get_action_object_scores_ensemble(state,action_space._action_predicate_to_operators,
                                                                                            groundings,prev_graph,prev_graph, ensemble=ensemble)
        #ic ("inference time",time.time()-inference_time)
        action_selection_time_1 = time.time()
        groundings = list(groundings)
        groundings_list = []
        for grounding in groundings:
            grounding_action = grounding.predicate
            objects = grounding.variables
            groundings_list.append(pddlgym.structs.Literal(grounding_action, objects))

        #ic (action_param_list)
        action_selection_time_2= time.time()
        for action_data in action_param_list:
            in_grounding = False
            decoded_action, decoded_action_parameters = action_data[0], action_data[1]
            new_action = pddlgym.structs.Literal(decoded_action, decoded_action_parameters)
            for grounded_action in groundings_list:
                in_grounding_temp = True
                if new_action.predicate == grounded_action.predicate:
                    for grounded_var, action_var in zip(grounded_action.variables, new_action.variables):
                        if grounded_var != action_var:
                            in_grounding_temp = False
                            break
                    if in_grounding_temp == True:
                        in_grounding = True
                        break
            if in_grounding == False:
                continue
            step_taking_time = time.time()
            state = env.step(new_action)
            step_taking_end_time = time.time()
            #ic ("Step taking time", step_taking_end_time-step_taking_time)
            state = state[0]
            break
        action_selection_end_time = time.time()
        #ic ("action selection time time",action_selection_end_time-action_selection_time_1)
        #ic ("action selection time time",action_selection_end_time-action_selection_time_2)
        curr_plan_states.append(state[0])
        new_plan.append(new_action)
        #ic (new_plan)
        plan_val_time = time.time()
        #ic (env.metadata)
        #ic (state)
        #ic (state.goal)
        #ic (state.goal.__dict__)
        #ic (state.literals)
        plan_found = True

        for goal in state.goal.literals:
            if goal not in list (state.literals):
                plan_found = False

        if plan_found == True :
            break

        # if num_actions > 40 :
        # if len(new_plan) > 8:
        if len(new_plan) > max_plan_length_permitted:
            # output_plan_lengths.append(len(new_plan))
            #ic (new_plan)
            #new_plan = None
            #exit()
            break
    start_state, _ = env.reset()
    #new_plan = []
    discrepancy_search_bool = False
    discrepancy_search_bool = True
    if discrepancy_search_bool == True:
        if len(new_plan) >= max_plan_length_permitted:
            ic (new_plan)
            succesful_plans = discrepancy_search(test_planner, env, start_state, action_space, ensemble, new_plan)
            # for plan in succesful_plans:
            ic (succesful_plans)
            if len(succesful_plans) != 0:
                new_plan = succesful_plans[-1]
                ic (new_plan)
                if new_plan != None:
                    # ic ("Valid plan")
                    print("valid plan")
                else :
                    new_plan = None
            else :
                new_plan = None
    end_time = time.time()
    #ic ("Time taken by learned system", end_time-start_time)
    #renamed_plan, _ = solve_from_task(sas_task, debug=debug, **kwargs)
    #renamed_plan = learned_planner(task)
    #ic (" * ** ** * * * * renamed plan ** **** ** " )
    #ic (new_plan[0].__dict__)
    ic (new_plan)
    if new_plan == None :
        renamed_plan = None
    else :
        #renamed_plan = new_plan[:]
        #ic (list(state))
        renamed_plan = get_renamed_plan_from_pddlgym_plan(new_plan,obj_conversions,obj_conversions_pddlstream)
    #renamed_plan = pddl
    #start_time = time.time()
    ##renamed_plan, _ = solve_from_task(sas_task, debug=debug, **kwargs)
    #end_time = time.time()
    #ic ("Planner search time", end_time-start_time)
    ic (" * ** ** * * * * renamed plan ** **** ** " , renamed_plan)
    if renamed_plan is None:
        return instantiated, None, temporal_plan, INF

    #action_instances = [action_from_name[name if RENAME_ACTIONS else '({} {})'.format(name, ' '.join(args))]
    #                    for name, args in renamed_plan]
    action_instances = []
    #for action in renamed_plan:
    #    action_instances.append(action_from_name[action.])
    action_instances = [action_from_name[name if RENAME_ACTIONS else '({} {})'.format(name, ' '.join(args))]
                        for name, args in renamed_plan]
    ic (action_instances)
    #renamed_plan_2 = pddlgym_plan_to_pddlstream_plan(new_plan,renamed_plan,opt_evaluations,None)
    #ic (renamed_plan)
    #ic (action_instances)
    cost = get_plan_cost(action_instances, cost_from_action)
    return instantiated, action_instances, temporal_plan, cost

##################################################

def plan_streams(evaluations, goal_expression, domain, all_results, negative, effort_weight, max_effort,
                 simultaneous=False, reachieve=True, replan_actions=set(), **kwargs):
    # TODO: alternatively could translate with stream actions on real opt_state and just discard them
    # TODO: only consider axioms that have stream conditions?
    #reachieve = reachieve and not using_optimizers(all_results)
    #for i, result in enumerate(all_results):
    #    print(i, result, result.get_effort())
    #ic (all_results)
    #ic (evaluations)
    applied_results, deferred_results = partition_results(
        evaluations, all_results, apply_now=lambda r: not (simultaneous or r.external.info.simultaneous))
    #ic (applied_results)
    stream_domain, deferred_from_name = add_stream_actions(domain, deferred_results)

    #updated_init  = get_init_from_evals(evaluations)
    #ic (evaluations)
    #ic (len(evaluations))
    #ic (updated_init)
    #updated_init = add_negative_to_init (updated_init,negative)
    #ic (updated_init)

    if reachieve and not using_optimizers(all_results):
        achieved_results = {n.result for n in evaluations.values() if isinstance(n.result, Result)}
        init_evaluations = {e for e, n in evaluations.items() if n.result not in achieved_results}
        applied_results = achieved_results | set(applied_results)
        evaluations = init_evaluations # For clarity

    # TODO: could iteratively increase max_effort
    node_from_atom = get_achieving_streams(evaluations, applied_results, # TODO: apply to all_results?
                                           max_effort=max_effort)
    #ic (node_from_atom)
    #ic (node_from_atom)
    #ic(len(node_from_atom))
    '''
    #ic (type(node_from_atom))
    #ic (node_from_atom.keys())
    #file_write_str = generate_pddl_from_init_goal(node_from_atom,goal_expression)
    #ic (file_write_str)
    #path = "/Users/rajesh/anaconda3/envs/35_vision_2/lib/python3.7/site-packages/pddlgym/pddl/unity_1/"
    #f = open(path+"problem" +str(7) +".pddl", "w")
    #f.write(file_write_str)
    #f.close()
    '''
    opt_evaluations = {evaluation_from_fact(f): n.result for f, n in node_from_atom.items()}
    updated_opt_init = None
    opt_init = None
    '''
    REQUIRED FOR CONT
    '''

    opt_init,opt_obj_conversion = get_init_from_opt_evals(opt_evaluations)
    updated_opt_init = add_negative_to_init (opt_init,negative)
    #pddl_problem = get_problem_pddl(evaluations,goal_expression,domain.pddl,temporal=True)
    #ic (pddl_problem)
    #exit()
    #ic (opt_init)
    #ic (opt_evaluations)
    #ic (updated_opt_init)
    #ic (opt_init)
    #ic (negative)
    #ic (evaluations)
    #ic (opt_evaluations)
    #ic (len(opt_evaluations))
    #ic (len(opt_evaluations))
    #ic (opt)
    if UNIVERSAL_TO_CONDITIONAL or using_optimizers(all_results):
        goal_expression = add_unsatisfiable_to_goal(stream_domain, goal_expression)

    temporal = isinstance(stream_domain, SimplifiedDomain)
    optimistic_fn = solve_optimistic_temporal if temporal else solve_optimistic_sequential
    #ic (goal_expression)
    goal = []
    #ic (goal_expression)
    #ic (len(goal_expression))
    #ic (len(goal_expression[0]))
    #ic (type(goal_expression[0]))

    if goal_expression[0] == 'and' :
        for goal_exp in goal_expression:
            #ic (goal_exp)
            if goal_exp == 'and' :
                goal.append('and')
            else :
                goal_elem = []
                for elem in goal_exp:
                    '''
                    if type(elem) == str:
                        goal_elem.append(elem)
                    elif type(elem) == pddlstream.language.object.Object :
                        #ic (elem.__dict__)
                        goal_elem.append(elem.value)
                    '''
                    goal_elem.append(elem)

                goal.append(tuple(goal_elem))
                    #ic (elem)
                    #ic (type(elem))
    else :
        goal_elem = []
        for elem in goal_expression:
            '''
            if type(elem) == str:
                goal_elem.append(elem)
            elif type(elem) == pddlstream.language.object.Object:
                # ic (elem.__dict__)
                goal_elem.append(elem.value)
            '''
            goal_elem.append(elem)
        goal.append(tuple(goal_elem))
    #ic (goal)
    #exit()
    #ic (node_from_atom)
    #ic (optimistic_fn)
    #ic (evaluations)

    #domain_name = 'Discrete_tamp'
    domain_name = 'Discrete_tamp_3d'
    domain_name = 'Kuka'
    domain_str = domain_name.lower()
    domain_str_eval = domain_name.lower()
    domain_str_test = domain_name.lower() + "_test"
    #domain_str = 'pddlstream_tamp'
    test = True
    if test == True:
        domain_str_eval += '_test'
    use_learned = False
    #use_learned = True
    data_collection = False
    data_collection = True
    if use_learned == True :
        optimistic_fn = solve_optimistic_learned
        #ic(datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%f_%p"))
        #ic ("storing data for pddl gym planning")
        init_goal_pddl,obj_conversions = generate_pddl_from_init_goal(updated_opt_init, goal, domain_str)
        #train_path = "/Users/rajesh/anaconda3/envs/35_vision_2/lib/python3.7/site-packages/pddlgym/pddl/discrete_tamp/"
        test_path = "/Users/rajesh/anaconda3/envs/35_vision_2/lib/python3.7/site-packages/pddlgym/pddl/" + domain_str_eval + "/"
        path = test_path
        filename = path + "problem_" + datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%f_%p") + ".pddl"
        f = open(filename, "w")
        f.write(init_goal_pddl)
        f.close()

        instantiated, action_instances, temporal_plan, cost = optimistic_fn(
            domain, stream_domain, applied_results, all_results, opt_evaluations,
            node_from_atom, goal_expression, effort_weight,obj_conversions,domain_name, **kwargs)
    else :
        instantiated, action_instances, temporal_plan, cost = optimistic_fn(
            domain, stream_domain, applied_results, all_results, opt_evaluations,
            node_from_atom, goal_expression, effort_weight, **kwargs)

    #ic (action_instances)
    #ic (temporal_plan)
    #exit()

    #ic (action_instances)
    if action_instances is None:
        print ("* * ** ** ** * Action instances is none")
        return OptSolution(FAILED, FAILED, cost)

    else :
        #ic ("renamed plan not none")
        train = True
        test = False
        #test = True
        #for object in task.objects:
            # ic (obj_from_pddl(object.name))
        #    obj_conversions_pddlstream[obj_from_pddl(object.name)] = object.name
        #init_goal_pddl,obj_conversion = generate_pddl_from_init_goal(updated_opt_init, goal, 'discrete_tamp')
        #ic (obj_conversion)
        #ic (init_goal_pddl)
        if data_collection == True :
            paths = []
            #goal = ('AtPose', 'b0', np.array([2, 0]))
            #ic ("Collecting data")
            init_goal_pddl,obj_conversions = generate_pddl_from_init_goal(updated_opt_init,goal, domain_str)
            #ic (obj_conversions)
            path = None
            #train_path = "/Users/rajesh/anaconda3/envs/35_vision_2/lib/python3.7/site-packages/pddlgym/pddl/discrete_tamp/"
            #test_path = "/Users/rajesh/anaconda3/envs/35_vision_2/lib/python3.7/site-packages/pddlgym/pddl/discrete_tamp_test/"
            train_path = "/Users/rajesh/anaconda3/envs/35_vision_2/lib/python3.7/site-packages/pddlgym/pddl/" + domain_str + "/"# discrete_tamp/"
            test_path = "/Users/rajesh/anaconda3/envs/35_vision_2/lib/python3.7/site-packages/pddlgym/pddl/" + domain_str_test + "/"#discrete_tamp_test/"
            if train == True :
                paths.append(train_path)
            if test == True :
                paths.append(test_path)
            #else :
            for path in paths:
                current_time = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%f_%p")
                filename = path + "problem_" + current_time + ".pddl"
                solution_filename = path + "plans/plan_" + current_time + ".pddl"
                ic (domain_str)
                pddlgym_solution = pddlgym_from_pddlstream_plan(action_instances,obj_conversions,domain_str)
                ic (pddlgym_solution)
                f = open(filename,"w")
                f.write(init_goal_pddl)
                f.close()
                #f = open(solution_filename,"w")
                #f.write(pddlgym_solution)
                #f.close()
                with open(solution_filename, "wb") as f:
                    pickle.dump(pddlgym_solution, f)
                f.close()

    exit()
    action_instances, axiom_plans = recover_axioms_plans(instantiated, action_instances)
    # TODO: extract out the minimum set of conditional effects that are actually required
    #simplify_conditional_effects(instantiated.task, action_instances)
    stream_plan, action_instances = recover_simultaneous(
        applied_results, negative, deferred_from_name, action_instances)

    #ic (stream_plan)
    #ic (" ** ** ** * in plan streams *** * ** * ")
    #ic (action_instances, stream_plan)

    #ic (action_instances)
    #ic (pddl_from_instance(action_instances[0]))
    action_plan = transform_plan_args(map(pddl_from_instance, action_instances), obj_from_pddl)
    #ic (action_plan)
    replan_step = min([step+1 for step, action in enumerate(action_plan)
                       if action.name in replan_actions] or [len(action_plan)+1]) # step after action application

    #ic (evaluations)
    #ic (opt_evaluations)
    #ic (axiom_plans)
    #ic (replan_step)
    #ic (stream_plan)
    #for neg in negative:
    #    for instance in neg.instances:
    #        ic (instance)

    stream_plan, opt_plan = recover_stream_plan(evaluations, stream_plan, opt_evaluations, goal_expression, stream_domain,
        node_from_atom, action_instances, axiom_plans, negative, replan_step)
    #ic (stream_plan)
    #for neg in negative:
    #    for instance in neg.instances:
    #        ic (instance)
    if temporal_plan is not None:
        # TODO: handle deferred streams
        assert all(isinstance(action, Action) for action in opt_plan.action_plan)
        opt_plan.action_plan[:] = temporal_plan
    #ic (node_from_atom)
    #exit()
    return OptSolution(stream_plan, opt_plan, cost)


#def generate_pddl_from_task(task, 'discrete_tamp'):

def get_renamed_plan_from_pddlgym_plan(plan,obj_conversions_pddl_gym,obj_conversions_pddlstream):
    renamed_plan = []
    #ic (obj_conversions_pddl_gym)
    #ic (obj_conversions_pddlstream)
    for action in plan:
        #ic (action.__dict__)
        variables = []
        for var in action.variables :
            #ic(obj_conversions_pddl_gym[str(var.split(":")[0])])
            #ic(obj_conversions_pddlstream[obj_conversions_pddl_gym[str(var.split(":")[0])]])
            variables.append(obj_conversions_pddlstream[obj_conversions_pddl_gym[str(var.split(":")[0])]])

        new_action = Action(str(action.predicate),variables)
        renamed_plan.append(new_action)

    #ic (renamed_plan)
    return renamed_plan

def pddlgym_from_pddlstream_plan(action_instances,obj_conversions,domain):
    pddl_plan = []
    #obj_conversions_inverted =  {str(v):k for k, v in obj_conversions.items()}
    obj_conversions_inverted = {}
    for key,value in obj_conversions.items():
        if 'p' in key :
            obj_conversions_inverted['p'+str(value)] = key
        if 'q' in key:
            obj_conversions_inverted['q' + str(value)] = key
        if 'b' in key:
            obj_conversions_inverted['b' + str(value)] = key
        if 't' in key:
            obj_conversions_inverted['t' + str(value)] = key
        else :
            obj_conversions_inverted[str(value)] = key
    #ic (obj_conversions)
    #ic (obj_conversions_inverted)

    action_position_type = {}
    #domain = 'discrete_tamp'
    if domain.lower() == 'discrete_tamp':
        action_position_type = {'move':{0:'q',1:'q'},
                                'pick':{0:'b',1:'p',2:'q'},
                                'place':{0:'b',1:'p',2:'q'} }
    elif domain.lower() == 'discrete_tamp_3d':
        action_position_type = {'move':{0:'q',1:'t',2:'q'},
                                'pick':{0:'b',1:'p',2:'q'},
                                'place':{0:'b',1:'p',2:'q'},
                                'stack':{0:'b',1:'b',2:'p',3:'q'},
                                'unstack':{0:'b',1:'b',2:'p',3:'q'}}
    elif domain.lower() == 'kuka':
        '''
        action_position_type = {'move_free':{0:'q',1:'q',2:'t'},
                                'move_holding': {0:'q',1:'q',2:'b',3:'g',4:'t'},
                                'pick': {0: 'b', 1: 'p', 2: 'g',3:'q',4:'t'},
                                'place': {0: 'b', 1: 'p', 2: 'g', 3: 'q', 4: 't'},
                                'clean':{0:'b',1:'r'},
                                'cook' : {0:'b',1:'r'}
                                }
        '''
        action_position_type = {'move_free': {0: 'q', 1: 'q', 2: 't'},
                                'move_holding': {0: 'q', 1: 'q', 2: '', 3: 'g', 4: 't'},
                                'pick': {0: '', 1: 'p', 2: 'g', 3: 'q', 4: 't'},
                                'place': {0: '', 1: 'p', 2: 'g', 3: 'q', 4: 't'},
                                'clean': {0: '', 1: 'r'},
                                'cook': {0: '', 1: 'r'}
                                }
    ic (action_instances)
    ic (obj_conversions)
    ic (obj_conversions_inverted)
    #ic (action_instances)
    for action in action_instances:
        ic (action.__dict__)
        decoded_action = action.action.name
        decoded_action_parameters = []

        #ic (action.__dict__)
        #ic (action.action.__dict__)
        #for param in action.action.parameters:
        index = 0
        for key,param in action.var_mapping.items():

            #ic (key+obj_from_pddl(param)))
            ic (action_position_type[decoded_action][index])
            curr_type = (action_position_type[decoded_action][index])
            #decoded_action_parameters.append(obj_conversions_inverted[str(obj_from_pddl(param))])
            #ic (obj_conversions_inverted)
            ic (curr_type)
            ic (param)
            ic (curr_type+str(obj_from_pddl(param)))
            decoded_action_parameters.append(obj_conversions_inverted[curr_type + str(obj_from_pddl(param))])
            #ic (param.__dict__)

            index += 1
        #ic (new_action)
        new_action = [decoded_action,decoded_action_parameters]
        pddl_plan.append(new_action)

    ic (pddl_plan)
    exit()
    return pddl_plan



