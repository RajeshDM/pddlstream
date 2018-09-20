from itertools import product

from pddlstream.language.constants import get_prefix, get_args, get_parameter_name
from pddlstream.language.conversion import substitute_expression, list_from_conjunction
from pddlstream.language.external import parse_lisp_list, get_procedure_fn
from pddlstream.language.stream import OptValue, StreamInfo, Stream
from pddlstream.language.generator import empty_gen
from pddlstream.utils import get_mapping, INF
from pddlstream.utils import elapsed_time, INF, get_mapping, find_unique, HeapElement
from pddlstream.algorithms.algorithm import neighbors_from_orders
from pddlstream.algorithms.reorder import get_partial_orders
from pddlstream.language.function import FunctionResult
from pddlstream.language.synthesizer import get_cluster_values
from collections import deque
import random


# TODO: augment state with the set of constraints required on the path
# TODO: axiom to disallow movement in states that use existing subset
# TODO: create additional variables in the event that the search

class Optimizer(object):
    def __init__(self, name, procedure):
        self.name = name
        self.procedure = procedure
        self.variables = []
        self.constraints = []
        self.objectives = []
    def get_streams(self):
        return self.variables + self.constraints
    def __repr__(self):
        return '{}'.format(self.name) #, self.streams)

class OptimizerStream(Stream):
    #_Instance = SynthStreamInstance
    #_Result = SynthStreamResult
    def __init__(self, optimizer, name, gen_fn, inputs, domain, outputs, certified, info):
        super(OptimizerStream, self).__init__(name, gen_fn, inputs, domain, outputs, certified, info)
        self.optimizer = optimizer
        # TODO: store list of variables & constraints

##################################################

def get_gen_fn(procedure, inputs, outputs, certified):
    def gen_fn(*input_values):
        mapping = get_mapping(inputs, input_values)
        targets = substitute_expression(certified, mapping)
        return procedure(outputs, targets)
    return gen_fn

def get_effort_fn(optimizer_name):
    # TODO: higher effort is the variable cannot be free for the testing process
    # This might happen if the variable is certified to have a property after construction
    def effort_fn(*input_values):
        free_indices = [i for i, value in enumerate(input_values) if isinstance(value, OptValue)
                        and value.stream.startswith(optimizer_name)]
        if not free_indices:
            return INF
        return 1
    return effort_fn

##################################################

def parse_variable(optimizer, lisp_list, stream_info):
    value_from_attribute = parse_lisp_list(lisp_list)
    variable = value_from_attribute[':variable'] # TODO: assume unique?
    outputs = [variable]
    inputs = value_from_attribute.get(':inputs', [])
    domain = list_from_conjunction(value_from_attribute.get(':domain'))
    certified = list_from_conjunction(value_from_attribute.get(':graph'))
    stream_name = '{}_{}'.format(optimizer.name, get_parameter_name(variable))
    gen_fn = get_gen_fn(optimizer.procedure, inputs, outputs, certified) # TODO: shouldn't need this...
    #gen_fn = empty_gen()
    #info = StreamInfo(effort_fn=get_effort_fn(optimizer_name, inputs, outputs))
    info = StreamInfo()
    return OptimizerStream(optimizer, stream_name, gen_fn, inputs, domain,
                           outputs, certified, info)

def parse_constraint(optimizer, lisp_list, stream_info):
    value_from_attribute = parse_lisp_list(lisp_list)
    outputs = []
    constraint = value_from_attribute[':constraint']
    certified = [constraint]
    inputs = get_args(constraint)
    domain = list_from_conjunction(value_from_attribute[':necessary'])
    stream_name = '{}_{}'.format(optimizer.name, get_prefix(constraint))
    gen_fn = get_gen_fn(optimizer.procedure, inputs, outputs, certified) # TODO: shouldn't need this...
    #gen_fn = empty_gen()
    info = StreamInfo(effort_fn=get_effort_fn(optimizer.name))
    return OptimizerStream(optimizer, stream_name, gen_fn, inputs, domain,
                           outputs, certified, info)

# TODO: convert optimizer into a set of streams? Already present within test stream

def parse_optimizer(lisp_list, stream_map, stream_info):
    _, optimizer_name = lisp_list[:2]
    procedure = get_procedure_fn(stream_map, optimizer_name)
    optimizer = Optimizer(optimizer_name, procedure)
    for sub_list in lisp_list[2:]:
        form = sub_list[0]
        if form == ':variable':
            optimizer.variables.append(parse_variable(optimizer, sub_list, stream_info))
        elif form == ':constraint':
            optimizer.constraints.append(parse_constraint(optimizer, sub_list, stream_info))
        elif form == ':objective':
            optimizer.objectives.append(sub_list[1])
        else:
            raise ValueError(form)
    return optimizer.get_streams()

##################################################

def get_optimizer(result):
    external = result.instance.external
    if isinstance(external, OptimizerStream):
        return external.optimizer
    return None

def get_connected_components(vertices, edges):
    #return [vertices]
    incoming, outgoing = neighbors_from_orders(edges)
    clusters = []
    processed = set()
    for v0 in vertices:
        if v0 in processed:
            continue
        processed.add(v0)
        cluster = {v0}
        queue = deque([v0])
        while queue:
            v1 = queue.popleft()
            for v2 in (incoming[v1] | outgoing[v1]):
                if v2 not in processed:
                    processed.add(v2)
                    cluster.add(v2)
                    queue.append(v2)
        clusters.append([v for v in vertices if v in cluster])
    return clusters

def combine_optimizer_plan(stream_plan, functions):
    if not stream_plan:
        return stream_plan
    optimizer = get_optimizer(stream_plan[-1])
    if optimizer is None:
        return stream_plan
    function_plan = list(filter(lambda r: get_prefix(r.instance.external.head) in optimizer.objectives, functions))
    external_plan = stream_plan + function_plan
    optimizer_plan = []
    for cluster in get_connected_components(external_plan, get_partial_orders(external_plan)):
        inputs, domain, outputs, certified, functions, _, \
            input_objects, output_objects = get_cluster_values(cluster)
        if not certified:
            continue
        gen_fn = get_gen_fn(optimizer.procedure, inputs, outputs, certified | functions)
        info = StreamInfo()
        stream = Stream(optimizer.name, gen_fn, inputs, domain, outputs, certified, info)
        instance = stream.get_instance(input_objects)
        optimizer_plan.append(stream._Result(instance, output_objects))
    return optimizer_plan

def combine_optimizers(external_plan):
    # Really the key thing is that a variable must be grounded before it can used in a non-stream thing
    # TODO: construct variables in order
    # TODO: graph cut algorithm to minimize the number of constraints that are excluded
    # TODO: reorder to ensure that constraints are done first since they are likely to fail as tests

    print(external_plan)
    #optimizers = {get_optimizer(r) for r in external_plan} # None is like a unique optimizer
    #print(optimizers)
    incoming_edges, outgoing_edges = neighbors_from_orders(get_partial_orders(external_plan))
    queue = []
    functions = []
    for v in external_plan:
        if not incoming_edges[v]:
            (functions if isinstance(v, FunctionResult) else queue).append(v)
    current = []
    ordering = []
    while queue:
        optimizer = get_optimizer(current[-1]) if current else None
        for v in queue:
            if optimizer == get_optimizer(v):
                current.append(v)
                break
        else:
            ordering.extend(combine_optimizer_plan(current, functions))
            current = [queue[0]]
        v1 = current[-1]
        #v1 = random.choice(queue)
        queue.remove(v1)
        for v2 in outgoing_edges[v1]:
            incoming_edges[v2].remove(v1)
            if not incoming_edges[v2]:
                (functions if isinstance(v2, FunctionResult) else queue).append(v2)
    ordering.extend(combine_optimizer_plan(current, functions))
    return ordering + functions

##################################################

# TODO: opt gen function
# TODO: custom optimistic objects

def parse_constraint2(optimizer_name, procedure, lisp_list, stream_info):
    value_from_attribute = parse_lisp_list(lisp_list)
    fixed = value_from_attribute.get(':fixed', [])
    mutex = value_from_attribute.get(':mutex', []) # Associating a variable to a constraint
    #free = value_from_attribute.get(':free', [])
    [constraint] = list_from_conjunction(value_from_attribute[':constraint'])
    constraint_name = get_prefix(constraint)
    necessary = list_from_conjunction(value_from_attribute[':necessary'])
    streams = []
    #for i, combo in enumerate(product([False, True], repeat=len(free))):
    #    outputs = [p for p, include in zip(free, combo) if include]
    # if not outputs:
    #    continue
    outputs_list = [tuple()] + [(p,) for p in mutex]
    for outputs in outputs_list:
        inputs = [p for p in get_args(constraint) if p not in outputs]
        certified = [constraint] + [f for f in necessary if any(p in outputs for p in get_args(f))]
        domain = [f for f in necessary if f not in certified]
        # TODO: don't include constraints without free params
        combo = [p in outputs for p in get_args(constraint)]
        stream_name = '{}_{}_{}'.format(optimizer_name, constraint_name, ''.join(map(str, map(int, combo))))
        info = StreamInfo(effort_fn=get_effort_fn(optimizer_name, inputs, outputs))
        #info = StreamInfo()
        gen_fn = get_gen_fn(procedure, inputs, outputs, certified)
        streams.append(Stream(stream_name, gen_fn, inputs, domain, outputs, certified, info))
        # TODO: prune implied facts
        # TODO: finite cost when inputs are from the same constraint
    return streams

def parse_optimizer2(lisp_list, stream_map, stream_info):
    _, name = lisp_list[:2]
    procedure = get_procedure_fn(stream_map, name)
    streams = []
    for constraint in lisp_list[2:]:
        streams.extend(parse_constraint2(name, procedure, constraint, stream_info))
    return streams