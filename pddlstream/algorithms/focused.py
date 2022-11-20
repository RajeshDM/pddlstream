from __future__ import print_function

import time

from pddlstream.algorithms.algorithm import parse_problem
from pddlstream.algorithms.advanced import enforce_simultaneous, identify_non_producers
from pddlstream.algorithms.common import SolutionStore,add_facts
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.algorithms.disabled import push_disabled, reenable_disabled, process_stream_plan,process_stream_plan_partial
from pddlstream.algorithms.disable_skeleton import create_disabled_axioms
from pddlstream.algorithms.downward import get_problem,task_from_domain_problem
from pddlstream.algorithms.incremental import process_stream_queue
from pddlstream.algorithms.instantiation import Instantiator
from pddlstream.algorithms.instantiate_task import instantiate_task,instantiate_goal,sas_from_instantiated
from pddlstream.algorithms.refinement import iterative_plan_streams, get_optimistic_solve_fn
from pddlstream.algorithms.refinement import optimistic_process_streams,optimistic_stream_evaluation,\
                                                optimistic_stream_instantiation
from pddlstream.algorithms.scheduling.plan_streams import OptSolution,rename_instantiated_actions
from pddlstream.algorithms.reorder import reorder_stream_plan
from pddlstream.algorithms.search import solve_from_task
from pddlstream.algorithms.scheduling.recover_streams import get_achieving_streams
from pddlstream.algorithms.scheduling.utils import partition_results
from pddlstream.algorithms.scheduling.stream_action import add_stream_actions
from pddlstream.algorithms.scheduling.add_optimizers import using_optimizers
from pddlstream.algorithms.skeleton import SkeletonQueue
from pddlstream.algorithms.visualization import reset_visualizations, create_visualizations, \
    has_pygraphviz, log_plans
from pddlstream.language.constants import is_plan, get_length, str_from_plan, INFEASIBLE,FAILED
from pddlstream.language.conversion import obj_from_pddl_plan, evaluation_from_fact, \
    fact_from_evaluation, transform_plan_args, transform_action_args, obj_from_pddl
from pddlstream.language.external import Result
from pddlstream.language.fluent import compile_fluent_streams
from pddlstream.language.function import Function, Predicate
from pddlstream.language.optimizer import ComponentStream
from pddlstream.algorithms.recover_optimizers import combine_optimizers
from pddlstream.language.statistics import load_stream_statistics, \
    write_stream_statistics, compute_plan_effort
from pddlstream.language.stream import Stream, StreamResult
from pddlstream.utils import INF, implies, str_from_object, safe_zip,elapsed_time,generate_pddl_from_init_goal,get_init_from_evals
from icecream import ic
RENAME_ACTIONS = False

def get_negative_externals(externals):
    negative_predicates = list(filter(lambda s: type(s) is Predicate, externals)) # and s.is_negative()
    negated_streams = list(filter(lambda s: isinstance(s, Stream) and s.is_negated, externals))
    return negative_predicates + negated_streams

def partition_externals(externals, verbose=False):
    functions = list(filter(lambda s: type(s) is Function, externals))
    negative = get_negative_externals(externals)
    optimizers = list(filter(lambda s: isinstance(s, ComponentStream) and (s not in negative), externals))
    streams = list(filter(lambda s: s not in (functions + negative + optimizers), externals))
    if verbose:
        print('Streams: {}\nFunctions: {}\nNegated: {}\nOptimizers: {}'.format(
            streams, functions, negative, optimizers))
    return streams, functions, negative, optimizers

##################################################

def recover_optimistic_outputs(stream_plan):
    if not is_plan(stream_plan):
        return stream_plan
    new_mapping = {}
    new_stream_plan = []
    for result in stream_plan:
        new_result = result.remap_inputs(new_mapping)
        new_stream_plan.append(new_result)
        if isinstance(new_result, StreamResult):
            opt_result = new_result.instance.opt_results[0] # TODO: empty if disabled
            new_mapping.update(safe_zip(new_result.output_objects, opt_result.output_objects))
    return new_stream_plan

def check_dominated(skeleton_queue, stream_plan):
    if not is_plan(stream_plan):
        return True
    for skeleton in skeleton_queue.skeletons:
        # TODO: has stream_plans and account for different output object values
        if frozenset(stream_plan) <= frozenset(skeleton.stream_plan):
            print(stream_plan)
            print(skeleton.stream_plan)
    raise NotImplementedError()

##################################################

def solve_abstract(problem, constraints=PlanConstraints(), stream_info={}, replan_actions=set(),
                  unit_costs=False, success_cost=INF,
                  max_time=INF, max_iterations=INF, max_memory=INF,
                  initial_complexity=0, complexity_step=1, max_complexity=INF,
                  max_skeletons=INF, search_sample_ratio=0, bind=True, max_failures=0,
                  unit_efforts=False, max_effort=INF, effort_weight=None, reorder=True,
                  visualize=False, verbose=True, **search_kwargs):
    """
    Solves a PDDLStream problem by first planning with optimistic stream outputs and then querying streams
    :param problem: a PDDLStream problem
    :param constraints: PlanConstraints on the set of legal solutions
    :param stream_info: a dictionary from stream name to StreamInfo altering how individual streams are handled
    :param replan_actions: the actions declared to induce replanning for the purpose of deferred stream evaluation

    :param unit_costs: use unit action costs rather than numeric costs
    :param success_cost: the exclusive (strict) upper bound on plan cost to successfully terminate

    :param max_time: the maximum runtime
    :param max_iterations: the maximum number of search iterations
    :param max_memory: the maximum amount of memory

    :param initial_complexity: the initial stream complexity limit
    :param complexity_step: the increase in the stream complexity limit per iteration
    :param max_complexity: the maximum stream complexity limit

    :param max_skeletons: the maximum number of plan skeletons (max_skeletons=None indicates not adaptive)
    :param search_sample_ratio: the desired ratio of sample time / search time when max_skeletons!=None
    :param bind: if True, propagates parameter bindings when max_skeletons=None
    :param max_failures: the maximum number of stream failures before switching phases when max_skeletons=None

    :param unit_efforts: use unit stream efforts rather than estimated numeric efforts
    :param max_effort: the maximum amount of stream effort
    :param effort_weight: a multiplier for stream effort compared to action costs
    :param reorder: if True, reorder stream plans to minimize the expected sampling overhead

    :param visualize: if True, draw the constraint network and stream plan as a graphviz file
    :param verbose: if True, print the result of each stream application
    :param search_kwargs: keyword args for the search subroutine

    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan (INF if no plan), and evaluations is init expanded
        using stream applications
    """
    # TODO: select whether to search or sample based on expected success rates
    # TODO: no optimizers during search with relaxed_stream_plan
    # TODO: locally optimize only after a solution is identified
    # TODO: replan with a better search algorithm after feasible
    # TODO: change the search algorithm and unit costs based on the best cost
    #ic (search_sample_ratio)
    #exit()
    use_skeletons = (max_skeletons is not None)
    #assert implies(use_skeletons, search_sample_ratio > 0)
    eager_disabled = (effort_weight is None)  # No point if no stream effort biasing
    num_iterations = eager_calls = 0
    complexity_limit = initial_complexity

    evaluations, goal_exp, domain, externals = parse_problem(
        problem, stream_info=stream_info, constraints=constraints,
        unit_costs=unit_costs, unit_efforts=unit_efforts)
    #ic (evaluations)
    #ic (externals)
    identify_non_producers(externals)
    #ic (externals)
    #for stream in externals:
    #    ic ( stream.is_negated  )
    #exit()
    enforce_simultaneous(domain, externals)
    #ic (externals)
    compile_fluent_streams(domain, externals)
    #ic (externals)
    #ic (externals)
    #exit()
    # TODO: make effort_weight be a function of the current cost
    # if (effort_weight is None) and not has_costs(domain):
    #     effort_weight = 1

    load_stream_statistics(externals)
    if visualize and not has_pygraphviz():
        visualize = False
        print('Warning, visualize=True requires pygraphviz. Setting visualize=False')
    if visualize:
        reset_visualizations()
    streams, functions, negative, optimizers = partition_externals(externals, verbose=verbose)
    eager_externals = list(filter(lambda e: e.info.eager, externals))
    positive_externals = streams + functions + optimizers
    has_optimizers = bool(optimizers) # TODO: deprecate
    assert implies(has_optimizers, use_skeletons)

    #ic (positive_externals)
    #ic (streams)
    #ic (functions)
    #ic (negative)
    #ic (optimizers)

    #positive_externals.append(negative[0])
    #negative = []
    #ic(externals)
    #exit()
    '''

    generate_new_facts(evaluations, domain, max_time, success_cost, max_memory,
                           has_optimizers, externals, eager_disabled, eager_externals,
                           complexity_limit, max_effort, goal_exp, verbose=False)

    ic ("\n *************** \n \n ********************* \n")
    #generate_new_facts(evaluations, domain, max_time, success_cost, max_memory,
    #                   has_optimizers, externals, eager_disabled, eager_externals,
    #                   complexity_limit, max_effort, goal_exp, verbose=False)
    exit()
    generate_stream_instances(evaluations,domain,max_time,success_cost,max_memory,
                          has_optimizers,externals,eager_disabled,eager_externals,
                          complexity_limit,max_effort,goal_exp,verbose)
    '''

    #ic (goal_exp)
    #goal = goal_exp
    #goal = problem[-1]
    #ic (goal)
    #ic (goal_exp)
    #exit()
    store = SolutionStore(evaluations, max_time, success_cost, verbose, max_memory=max_memory)
    skeleton_queue = SkeletonQueue(store, domain, disable=not has_optimizers)
    disabled = set() # Max skeletons after a solution
    while (not store.is_terminated()) and (num_iterations < max_iterations) and (complexity_limit <= max_complexity):
        num_iterations += 1
        #if num_iterations >= 7:
        #    break
        #ic (evaluations)
        #ic (negative)

        #ic (num_iterations,"first", evaluations)
        #for evaluation in evaluations:
        #    for arg in evaluation.head.args:
        #        ic (arg.__dict__)
        eager_instantiator = Instantiator(eager_externals, evaluations) # Only update after an increase?
        if eager_disabled:
            push_disabled(eager_instantiator, disabled)
        if eager_externals:
            eager_calls += process_stream_queue(eager_instantiator, store,
                                                complexity_limit=complexity_limit, verbose=verbose)

        #ic (evaluations)
        #ic (len(evaluations))
        ################

        print('\nIteration: {} | Complexity: {} | Skeletons: {} | Skeleton Queue: {} | Disabled: {} | Evaluations: {} | '
              'Eager Calls: {} | Cost: {:.3f} | Search Time: {:.3f} | Sample Time: {:.3f} | Total Time: {:.3f}'.format(
            num_iterations, complexity_limit, len(skeleton_queue.skeletons), len(skeleton_queue), len(disabled),
            len(evaluations), eager_calls, store.best_cost, store.search_time, store.sample_time, store.elapsed_time()))

        optimistic_solve_fn = get_optimistic_solve_fn(goal_exp, domain, negative,
                                                      replan_actions=replan_actions, reachieve=use_skeletons,
                                                      max_cost=min(store.best_cost, constraints.max_cost),
                                                      max_effort=max_effort, effort_weight=effort_weight, **search_kwargs)
        #ic (eager_instantiator.__dict__)
        # TODO: just set unit effort for each stream beforehand
        print (" **** In focused b4 iterative **** ")
        if (max_skeletons is None) or (len(skeleton_queue.skeletons) < max_skeletons):
            disabled_axioms = create_disabled_axioms(skeleton_queue) if has_optimizers else []
            if disabled_axioms:
                domain.axioms.extend(disabled_axioms)
            #ic (domain)
            stream_plan, opt_plan, cost = iterative_plan_streams(evaluations, positive_externals,
                optimistic_solve_fn, complexity_limit, max_effort=max_effort)
            for axiom in disabled_axioms:
                domain.axioms.remove(axiom)
        else:
            stream_plan, opt_plan, cost = OptSolution(INFEASIBLE, INFEASIBLE, INF) # TODO: apply elsewhere

        #print (stream_plan)
        #print (opt_plan)
        #ic (eager_instantiator.__dict__)
        #exit()
        ################
        #stream_plan = replan_with_optimizers(evaluations, stream_plan, domain, externals) or stream_plan
        stream_plan = combine_optimizers(evaluations, stream_plan)
        #ic (positive_externals)
        #ic (evaluations)

        #ic ("Somewhere in between ")
        #stream_plan = get_synthetic_stream_plan(stream_plan, # evaluations
        #                                       [s for s in synthesizers if not s.post_only])
        #stream_plan = recover_optimistic_outputs(stream_plan)
        if reorder:
            # TODO: this blows up memory wise for long stream plans
            stream_plan = reorder_stream_plan(store, stream_plan)

        num_optimistic = sum(r.optimistic for r in stream_plan) if stream_plan else 0
        action_plan = opt_plan.action_plan if is_plan(opt_plan) else opt_plan
        '''
        print('Stream plan ({}, {}, {:.3f}): {}\nAction plan ({}, {:.3f}): {}'.format(
            get_length(stream_plan), num_optimistic, compute_plan_effort(stream_plan), stream_plan,
            get_length(action_plan), cost, str_from_plan(action_plan)))
        '''
        #ic ("Somewhere in between ")
        if is_plan(stream_plan) and visualize:
            log_plans(stream_plan, action_plan, num_iterations)
            create_visualizations(evaluations, stream_plan, num_iterations)

        ################

        #ic (stream_plan)
        #ic (stream_plan is INFEASIBLE)
        #ic (not eager_instantiator)
        #ic (not skeleton_queue)
        #ic (not disabled)

        if (stream_plan is INFEASIBLE) and (not eager_instantiator) and (not skeleton_queue) and (not disabled):
            ic (stream_plan is INFEASIBLE)
            ic (not eager_instantiator)
            ic (not skeleton_queue)
            ic (not disabled)
            ic ("INFEASIBLE OR PROBLEM WITH EAGER instantiator etc")
            break
        if not is_plan(stream_plan):
            print('No plan: increasing complexity from {} to {}'.format(complexity_limit, complexity_limit+complexity_step))
            complexity_limit += complexity_step
            if not eager_disabled:
                reenable_disabled(evaluations, domain, disabled)

        #print(stream_plan_complexity(evaluations, stream_plan))
        #ic (num_iterations,"second",evaluations)
        if not use_skeletons:
            process_stream_plan(store, domain, disabled, stream_plan, opt_plan, cost, bind=bind, max_failures=max_failures)
            continue

        ################

        #optimizer_plan = replan_with_optimizers(evaluations, stream_plan, domain, optimizers)
        optimizer_plan = None
        if optimizer_plan is not None:
            # TODO: post process a bound plan
            print('Optimizer plan ({}, {:.3f}): {}'.format(
                get_length(optimizer_plan), compute_plan_effort(optimizer_plan), optimizer_plan))
            skeleton_queue.new_skeleton(optimizer_plan, opt_plan, cost)

        allocated_sample_time = (search_sample_ratio * store.search_time) - store.sample_time \
            if len(skeleton_queue.skeletons) <= max_skeletons else INF
        #ic (evaluations)
        #ic (opt_plan,stream_plan)
        #skeleton_queue.process(stream_plan,opt_plan,cost,complexity_limit,allocated_sample_time)
        #ic ("After getting stream plan 1")
        if skeleton_queue.process(stream_plan, opt_plan, cost, complexity_limit, allocated_sample_time) is INFEASIBLE:
            #ic ("INFEASIBLE skeleton queue")
            break
        #ic ("After getting stream plan 2")
        '''
        '''
        #ic ("Afterwards in focused")
        #updated_init  = get_init_from_evals(evaluations)
        #ic (evaluations)
        #ic (updated_init)

        #init_goal_pddl = generate_pddl_from_init_goal(updated_init, goal, 'discrete_tamp')
        #path = "/Users/rajesh/anaconda3/envs/35_vision_2/lib/python3.7/site-packages/pddlgym/pddl/discrete_tamp/"
        #file_number = num_iterations
        #f = open(path + "problem0" + str(file_number) + ".pddl", "w")
        #f.write(init_goal_pddl)
        #f.close()
    ################

    summary = store.export_summary()
    summary.update({
        'iterations': num_iterations,
        'complexity': complexity_limit,
        'skeletons': len(skeleton_queue.skeletons),
    })
    #print('Summary: {}'.format(str_from_object(summary, ndigits=3))) # TODO: return the summary

    #exit()
    write_stream_statistics(externals, verbose)
    return store.extract_solution()

solve_focused = solve_abstract # TODO: deprecate solve_focused

##################################################

def solve_focused_original(problem, fail_fast=False, **kwargs):
    """
    Solves a PDDLStream problem by first planning with optimistic stream outputs and then querying streams
    :param problem: a PDDLStream problem
    :param fail_fast: whether to switch phases as soon as a stream fails
    :param kwargs: keyword args for solve_focused
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    max_failures = 0 if fail_fast else INF
    return solve_abstract(problem, max_skeletons=None, search_sample_ratio=None,
                          bind=False, max_failures=max_failures, **kwargs)

def solve_binding(problem, fail_fast=False, **kwargs):
    """
    Solves a PDDLStream problem by first planning with optimistic stream outputs and then querying streams
    :param problem: a PDDLStream problem
    :param fail_fast: whether to switch phases as soon as a stream fails
    :param kwargs: keyword args for solve_focused
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    max_failures = 0 if fail_fast else INF
    return solve_abstract(problem, max_skeletons=None, search_sample_ratio=None,
                          bind=True, max_failures=max_failures, **kwargs)

def solve_adaptive(problem, max_skeletons=INF, search_sample_ratio=1, **kwargs):
    """
    Solves a PDDLStream problem by first planning with optimistic stream outputs and then querying streams
    :param problem: a PDDLStream problem
    :param max_skeletons: the maximum number of plan skeletons to consider
    :param search_sample_ratio: the desired ratio of search time / sample time
    :param kwargs: keyword args for solve_focused
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    max_skeletons = INF if max_skeletons is None else max_skeletons
    #search_sample_ratio = clip(search_sample_ratio, lower=0) # + EPSILON
    #assert search_sample_ratio > 0
    return solve_abstract(problem, max_skeletons=max_skeletons, search_sample_ratio=search_sample_ratio,
                          bind=None, max_failures=None, **kwargs)

def solve_hierarchical(problem, **kwargs):
    """
    Solves a PDDLStream problem by first planning with optimistic stream outputs and then querying streams
    :param problem: a PDDLStream problem
    :param search_sample_ratio: the desired ratio of sample time / search time
    :param kwargs: keyword args for solve_focused
    :return: a tuple (plan, cost, evaluations) where plan is a sequence of actions
        (or None), cost is the cost of the plan, and evaluations is init but expanded
        using stream applications
    """
    return solve_adaptive(problem, max_skeletons=1, search_sample_ratio=INF, # TODO: rename to sample_search_ratio
                          bind=None, max_failures=None, **kwargs)


def initialize_useful_structures(problem,stream_info,constraints,unit_costs,unit_efforts,max_time,success_cost,max_memory,verbose=False):
    evaluations, goal_exp, domain, externals = parse_problem(
        problem, stream_info=stream_info, constraints=constraints,
        unit_costs=unit_costs, unit_efforts=unit_efforts)
    identify_non_producers(externals)
    enforce_simultaneous(domain, externals)
    compile_fluent_streams(domain, externals)
    streams, functions, negative, optimizers = partition_externals(externals, verbose=verbose)
    eager_externals = list(filter(lambda e: e.info.eager, externals))
    positive_externals = streams + functions + optimizers
    has_optimizers = bool(optimizers)  # TODO: deprecate
    store = SolutionStore(evaluations, max_time, success_cost, verbose, max_memory=max_memory)
    return evaluations,goal_exp,domain,externals,eager_externals,positive_externals,has_optimizers,store


#def generate_new_facts(evaluations,eager_externals,max_time,):
def generate_new_facts(evaluations, domain, max_time, success_cost, max_memory,
                       has_optimizers, positive_externals, eager_disabled, eager_externals,
                              complexity_limit, max_effort, goal_exp, verbose=False):

    complexity_limit = 3
    #ic (complexity_limit)
    #ic (positive_externals)
    externals = []
    for external in positive_externals:
        #ic (external.__dict__)
        if external.name != 'distance':
            externals.append(external)
    #exit()
    eager_instantiator = Instantiator(eager_externals, evaluations)  # Only update after an increase?
    num_iterations = eager_calls = 0
    store = SolutionStore(evaluations, max_time, success_cost, verbose, max_memory=max_memory)

    eager_calls += process_stream_queue(eager_instantiator, store,
                                        complexity_limit=complexity_limit, verbose=verbose)
    complexity_evals = {e: n for e, n in evaluations.items() if n.complexity <= complexity_limit}

    #results, exhausted = optimistic_process_streams(complexity_evals, positive_externals, complexity_limit, max_effort=max_effort)
    results, exhausted = optimistic_process_streams(complexity_evals, externals, complexity_limit, max_effort=max_effort)
    #node_from_atom = get_achieving_streams(evaluations, results, # TODO: apply to all_results?
    #                                       max_effort=max_effort)
    stream_plan = [result for result in results if result.optimistic]
    disabled = set()
    #ic (stream_plan)
    #ic (evaluations)
    #ic (store.best_cost)
    #next_results, _ = optimistic_process_streams(evaluations, positive_externals, complexity_limit,max_effort=max_effort)
    new_results, bindings = optimistic_stream_evaluation(evaluations, stream_plan)
    stream_plan = [result for result in new_results if result.optimistic]
    #ic (new_results)
    #ic (bindings)
    process_stream_plan_partial(store, domain, disabled, stream_plan, [], success_cost, bind=True, max_failures=100)
    #ic (evaluations)
    #add_fact(evaluations, fact, result=INIT_EVALUATION, complexity=0)

    updated_init = get_init_from_evals(evaluations)

    #return evaluations
    return updated_init

def generate_stream_instances(evaluations,domain,max_time,success_cost,max_memory,
                              has_optimizers,positive_externals,eager_disabled,eager_externals,
                              complexity_limit,max_effort,goal_exp, verbose=False):

    ################
    '''
    Stream experiemnts (Rajesh)
    '''

    store = SolutionStore(evaluations, max_time, success_cost, verbose, max_memory=max_memory)
    skeleton_queue = SkeletonQueue(store, domain, disable=not has_optimizers)
    disabled = set() # Max skeletons after a solution
    #optimistic_process_streams(evaluations, streams, complexity_limit=INF)
    iterations = 0
    num_iterations = eager_calls = 0
    while True:
        ic (evaluations)
        num_iterations += 1
        #ic (num_iterations,"first", evaluations)
        eager_instantiator = Instantiator(eager_externals, evaluations) # Only update after an increase?
        if eager_disabled:
            push_disabled(eager_instantiator, disabled)
        if eager_externals:
            eager_calls += process_stream_queue(eager_instantiator, store,
                                                complexity_limit=complexity_limit, verbose=verbose)

        #iterative_plan_streams_only()
        iterative_plan_streams_only(evaluations, positive_externals,generate_node_from_atom, complexity_limit,domain,
                                                            goal_expression=goal_exp,max_effort=max_effort)
        ic (eager_instantiator.__dict__)
        #complexity_evals = {e: n for e, n in evaluations.items() if n.complexity <= complexity_limit}
        #all_results, exhausted = optimistic_process_streams(complexity_evals, externals, complexity_limit, max_effort=max_effort)
        #skeleton_queue.new_skeleton(stream_plan, action_plan,0)
        stream_plan = []
        for stream in positive_externals:
            ic (stream.__dict__)
            if stream.name == 'distance':
                continue
            for instance in stream.instances:
                stream_plan.append(instance)
        ic (stream_plan)
        ic (positive_externals)
        ic (evaluations)
        ic (store.__dict__)
        #exit()
        #skeleton_queue.greedily_process()
        iterations += 1
        complexity_limit += 1
        if iterations >= 3:
            break
    exit()
    #ic (node_from_atom)
    #ic(node_from_atom)
    #exit()
    #######################

def generate_node_from_atom(evaluations,all_results,domain,goal_expression,**effort_args):
    simultaneous = False
    reachieve = True
    ic (all_results)
    applied_results, deferred_results = partition_results(
        evaluations, all_results, apply_now=lambda r: not (simultaneous or r.external.info.simultaneous))
    ic (applied_results)
    stream_domain, deferred_from_name = add_stream_actions(domain, deferred_results)
    ic (evaluations)

    if reachieve and not using_optimizers(all_results):
        achieved_results = {n.result for n in evaluations.values() if isinstance(n.result, Result)}
        init_evaluations = {e for e, n in evaluations.items() if n.result not in achieved_results}
        applied_results = achieved_results | set(applied_results)
        # evaluations = init_evaluations # For clarity
    node_from_atom = get_achieving_streams(init_evaluations, applied_results,  # TODO: apply to all_results?
                                           **effort_args)

    ic (init_evaluations)
    ic (node_from_atom)
    ic (applied_results)
    ic (init_evaluations)
    ic(node_from_atom)
    opt_evaluations = {evaluation_from_fact(f): n.result for f, n in node_from_atom.items()}
    ic (opt_evaluations)
    #for key,value in opt_evaluations.items():
    #    ic (key,value)
    #ic (opt_evaluations)
    problem = get_problem(opt_evaluations, goal_expression, stream_domain)  # begin_metric
    #ic (problem.__dict__)
    ic (type(problem))
    task = task_from_domain_problem(stream_domain, problem)
    ic(task.__dict__)
    instantiated = instantiate_task(task)
    if instantiated is None :
        return None
    ic (instantiated)
    action_from_name = rename_instantiated_actions(instantiated, RENAME_ACTIONS)
    ic (action_from_name)
    sas_task = sas_from_instantiated(instantiated)
    sas_task.metric = True
    #renamed_plan, _ = solve_from_task(sas_task, debug=False, **search_args)
    #if renamed_plan is None:
    #    return None
        #return instantiated, None, temporal_plan, INF
    renamed_plan = None
    action_instances = [action_from_name[name if RENAME_ACTIONS else '({} {})'.format(name, ' '.join(args))]
                        for name, args in renamed_plan]
    ic (node_from_atom)
    return node_from_atom

def hierarchical_plan_streams_only(evaluations, externals, results, optimistic_fn, complexity_limit,
                                   domain, depth, constraints, goal_expression, **effort_args):
                                    #domain,depth, constraints,goal_expression,max_effort,**search_args):
    MAX_DEPTH = 3
    if MAX_DEPTH <= depth:
        return OptSolution(None, None, INF), depth
    #stream_plan, opt_plan, cost = optimistic_solve_fn(evaluations, results, constraints)
    #is_refined_stream_plan = is_refined(stream_plan)
    node_from_atom = optimistic_fn(evaluations,results,domain,goal_expression,**effort_args)

    #ic (is_plan(opt_plan))
    #ic(is_refined_stream_plan)
    #ic (opt_plan)
    #ic (stream_plan)
    #if not is_plan(opt_plan) or is_refined(stream_plan):
    #if not is_plan(opt_plan) or is_refined_stream_plan:
    #    return OptSolution(stream_plan, opt_plan, cost), depth

    # TODO: identify control parameters that can be separated across actions
    new_depth = depth + 1
    #new_results, bindings = optimistic_stream_evaluation(evaluations, stream_plan)
    #if not (CONSTRAIN_STREAMS or CONSTRAIN_PLANS):
    #    return OptSolution(FAILED, FAILED, INF), new_depth
    #if CONSTRAIN_STREAMS:
    #    next_results = compute_stream_results(evaluations, new_results, externals, complexity_limit, **effort_args)
    #else:
    next_results, _ = optimistic_process_streams(evaluations, externals, complexity_limit, **effort_args)
    next_constraints = None
    #if CONSTRAIN_PLANS:
    #    next_constraints = compute_skeleton_constraints(opt_plan, bindings)
    return hierarchical_plan_streams_only(evaluations, externals, next_results, optimistic_fn, complexity_limit,
                                     domain,new_depth, next_constraints,goal_expression,**effort_args )#max_effort,**search_args)

#def iterative_plan_streams_only(all_evaluations, externals, optimistic_solve_fn, complexity_limit,domain, goal_expression,**effort_args):
# Previously didn't have unique optimistic objects that could be constructed at arbitrary depths
def iterative_plan_streams_only(all_evaluations, externals, optimistic_solve_fn, complexity_limit, domain,
                                goal_expression, **effort_args):

    # Previously didn't have unique optimistic objects that could be constructed at arbitrary depths
    start_time = time.time()
    complexity_evals = {e: n for e, n in all_evaluations.items() if n.complexity <= complexity_limit}
    num_iterations = 0
    prev_results = []
    results = []
    while True:
        num_iterations += 1
        #prev_results = results[:]
        #results, exhausted = optimistic_process_streams(complexity_evals, externals, complexity_limit, **effort_args)
        results, exhausted = optimistic_process_streams(complexity_evals, externals, complexity_limit, **effort_args)
        #results, exhausted = optimistic_process_streams(complexity_evals, externals, complexity_limit, max_effort)#,**search_args)
        ic (complexity_evals)
        ic (externals)
        ic (complexity_limit)
        #ic (**effort_args)
        ic (results)
        #exit()
        opt_solution, final_depth = hierarchical_plan_streams_only(
            complexity_evals, externals, results, optimistic_solve_fn, complexity_limit,domain,
            #depth=0, constraints=None,goal_expression=goal_expression,**effort_args)
            depth = 0, constraints = None, goal_expression = goal_expression,**effort_args)
        stream_plan, action_plan, cost = opt_solution
        print('Attempt: {} | Results: {} | Depth: {} | Success: {} | Time: {:.3f}'.format(
            num_iterations, len(results), final_depth, is_plan(action_plan), elapsed_time(start_time)))
        #print (results)
        #print (list(set(results) - set(prev_results)))
        #print (len(list(set(results) - set(prev_results))))
        #print (len(prev_results))

        if is_plan(action_plan):
            return OptSolution(stream_plan, action_plan, cost)
        if final_depth == 0:
            status = INFEASIBLE if exhausted else FAILED
            return OptSolution(status, status, cost)
        if num_iterations >= 0:
            return
    # TODO: should streams along the sampled path automatically have no optimistic value

