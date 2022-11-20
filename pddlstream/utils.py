from __future__ import print_function

import math
import os
import pickle
import shutil
import sys
import time
import random
import cProfile
import pstats
import io

from collections import defaultdict, deque, Counter, namedtuple
#from pddlstream.language.constants import Action
from heapq import heappush, heappop
import itertools

import numpy as np
from icecream import ic
#from pddlstream import pddlstream
import pddlstream
#from pddlstream.language.constants import And, Equal, TOTAL_COST, print_solution, PDDLProblem
import pddlgym
import examples

Action = namedtuple('Action', ['name', 'args'])

INF = float('inf')
SEPARATOR = '\n' + 80*'-'  + '\n'
AND = 'and'

try:
   user_input = raw_input
except NameError:
   user_input = input

##################################################

def int_ceil(f):
    return int(math.ceil(f))


def get_python_version():
    return sys.version_info[0]


def read(filename):
    with open(filename, 'r') as f:
        return f.read()


def write(filename, string):
    with open(filename, 'w') as f:
        f.write(string)


def write_pickle(filename, data):
    # Cannot pickle lambda or nested functions
    with open(filename, 'wb') as f:
        pickle.dump(data, f)


def read_pickle(filename):
    with open(filename, 'rb') as f:
        return pickle.load(f)


def safe_remove(p):
    if os.path.exists(p):
        os.remove(p)


def mkdir(d):
    if not os.path.exists(d):
        os.makedirs(d)


def ensure_dir(f):
    d = os.path.dirname(f)
    if not os.path.exists(d):
        os.makedirs(d)


def safe_rm_dir(d):
    if os.path.exists(d):
        shutil.rmtree(d)


def clear_dir(d):
    safe_rm_dir(d)
    ensure_dir(d)


def get_file_path(file, rel_path):
    directory = os.path.dirname(os.path.abspath(file))
    return os.path.join(directory, rel_path)


def open_pdf(filename):
    import subprocess
    # import os
    # import webbrowser
    subprocess.Popen('open {}'.format(filename), shell=True)
    # os.system(filename)
    # webbrowser.open(filename)
    user_input('Display?')
    # safe_remove(filename)
    # TODO: close output

##################################################

def elapsed_time(start_time):
    return time.time() - start_time


def safe_zip(sequence1, sequence2):
    assert len(sequence1) == len(sequence2)
    return zip(sequence1, sequence2)


def get_mapping(sequence1, sequence2):
    return dict(safe_zip(sequence1, sequence2))


def apply_mapping(sequence, mapping):
    return tuple(mapping.get(e, e) for e in sequence)


def safe_apply_mapping(sequence, mapping):
    # TODO: flip arguments order
    return tuple(mapping[e] for e in sequence)


def negate_test(test):
    return lambda *args, **kwargs: not test(*args, **kwargs)


def flatten(iterable_of_iterables):
    return (item for iterables in iterable_of_iterables for item in iterables)


def find(test, sequence):
    for item in sequence:
        if test(item):
            return item
    return None


def find_unique(test, sequence):
    found, value = False, None
    for item in sequence:
        if test(item):
            if found:
                raise RuntimeError('Both elements {} and {} satisfy the test'.format(value, item))
            found, value = True, item
    if not found:
        raise RuntimeError('Unable to find an element satisfying the test')
    return value


def implies(a, b):
    return not a or b


def irange(start, end=None, step=1):
    # TODO: combine with my other infinite generator
    if end is None:
        end = start
        start = 0
    n = start
    while n < end:
        yield n
        n += step


def argmin(fn, iterable):
    return min(iterable, key=fn)


def argmax(fn, iterable):
    return max(iterable, key=fn)


def invert_dict(d):
    return {v: k for k, v in d.items()}


def randomize(iterable):
    sequence = list(iterable)
    random.shuffle(sequence)
    return sequence

##################################################

BYTES_PER_KILOBYTE = math.pow(2, 10)
BYTES_PER_GIGABYTE = math.pow(2, 30)
KILOBYTES_PER_GIGABYTE = BYTES_PER_GIGABYTE / BYTES_PER_KILOBYTE

def get_peak_memory_in_kb():
    # TODO: use psutil instead
    import psutil
    # https://pypi.org/project/psutil/
    # https://psutil.readthedocs.io/en/latest/
    #rss: aka "Resident Set Size", this is the non-swapped physical memory a process has used. (bytes)
    #vms: aka "Virtual Memory Size", this is the total amount of virtual memory used by the process. (bytes)
    #shared: (Linux) memory that could be potentially shared with other processes.
    #text (Linux, BSD): aka TRS (text resident set) the amount of memory devoted to executable code.
    #data (Linux, BSD): aka DRS (data resident set) the amount of physical memory devoted to other than executable code.
    #lib (Linux): the memory used by shared libraries.
    #dirty (Linux): the number of dirty pages.
    #pfaults (macOS): number of page faults.
    #pageins (macOS): number of actual pageins.
    process = psutil.Process(os.getpid())
    #process.pid()
    #process.ppid()
    pmem = process.memory_info() # this seems to actually get the current memory!
    memory_in_kb = pmem.vms / BYTES_PER_KILOBYTE
    return memory_in_kb
    #print(process.memory_full_info())
    #print(process.memory_percent())
    # process.rlimit(psutil.RLIMIT_NOFILE)  # set resource limits (Linux only)
    #print(psutil.virtual_memory())
    #print(psutil.swap_memory())
    #print(psutil.pids())
    #try:
    #    # This will only work on Linux systems.
    #    with open("/proc/self/status") as status_file:
    #        for line in status_file:
    #            parts = line.split()
    #            if parts[0] == "VmPeak:":
    #                return float(parts[1])
    #except IOError:
    #    pass
    #return 0.

def check_memory(max_memory):
    if max_memory == INF:
        return True
    peak_memory = get_peak_memory_in_kb()
    #print('Peak memory: {} | Max memory: {}'.format(peak_memory, max_memory))
    if peak_memory <= max_memory:
        return True
    print('Peak memory of {} KB exceeds memory limit of {} KB'.format(
        int(peak_memory), int(max_memory)))
    return False


def get_write_string_for_predicate(predicate):
    predicate_write_string = ""
    return predicate_write_string
    pass

def update_object_score(obj_scores, problem):
    #for obj in problem.relevant_object :
    #    if obj in obj_scores
    ic (obj_scores)
    ic (problem.relevant_objects)
    for obj,score in obj_scores.items():
        for obstacle in problem.relevant_objects :
            if str(obstacle) in obj:
                obj_scores[obstacle] = 1
        ic (obj,score)
    ic (obj_scores)

def get_init_from_evals(evaluations):
    init_from_eval = []
    for evaluation in evaluations:
        #ic (evaluation.head)
        #ic (evaluation.head.function)
        #ic (evaluation.head.__dict__)
        #ic (evaluation.head.args)
        init_from_eval_curr = [None] * (len(evaluation.head.args) + 1)
        #if evaluation.head.function not in extra_:
        init_from_eval_curr[0] = evaluation.head.function
        for arg_idx,arg in enumerate(evaluation.head.args):
            #ic (type(arg))
            #ic (arg.__dict__)
            #ic (type(arg.value))
            '''
            if type(arg.value) == str :
                init_from_eval_curr[arg_idx+1] = arg
            elif type(ar) :
                ic ("Not string")
            '''
            init_from_eval_curr[arg_idx+1] = arg.value

        init_from_eval.append(init_from_eval_curr)

    return init_from_eval

def get_init_from_opt_evals(opt_evaluations):
    init_from_eval = []
    opt_obj_conversion = {}
    for evaluation in opt_evaluations:
        #ic (evaluation.head)
        #ic (evaluation.head.function)
        #ic (evaluation.head.__dict__)
        #ic (evaluation.head.args)
        init_from_eval_curr = [None] * (len(evaluation.head.args) + 1)
        #if evaluation.head.function not in extra_:
        init_from_eval_curr[0] = evaluation.head.function
        for arg_idx,arg in enumerate(evaluation.head.args):
            #ic (type(arg))
            '''
            if type(arg) == pddlstream.language.object.OptimisticObject:
                init_from_eval_curr[arg_idx+1] = arg.pddl[1:]
                opt_obj_conversion[arg.pddl[1:]] = arg.pddl
            else :
                init_from_eval_curr[arg_idx+1] = arg.value
            '''
            init_from_eval_curr[arg_idx+1] = arg
            #ic (arg.value)
            #ic (arg.__dict__)
            #ic (type(arg.value))
            '''
            if type(arg.value) == str :
                init_from_eval_curr[arg_idx+1] = arg
            elif type(ar) :
                ic ("Not string")
            '''

        init_from_eval.append(init_from_eval_curr)

    #ic (init_from_eval)
    #ic (opt_obj_conversion)
    #exit()
    return init_from_eval,opt_obj_conversion

def update_init_from_negative(init,negative):
    ic (negative)
    ic (init)
    negative_streams_from_init = []
    for elem in init :
        if elem[0] == '~test-cfree-negative':
            negative_streams_from_init.append(elem)

    for neg in negative:
        if hasattr(neg,'instances'):
            for instance in neg.instances:
                ic (instance)
                found = False
                #ic (instance[1].__dict__)
                #ic (instance[1])
                #ic (type(instance[0][0]))
                #ic (type(instance[0][1]))
                #ic (instance[0][0].__dict__)
                #ic (instance[0][1].__dict__)
                opt_type = pddlstream.language.object.OptimisticObject
                if True :
                #if type(instance[0][0]) != opt_type and  type(instance[0][1]) != opt_type :
                    #if ['~test-cfree-negative',instance[0][0].value,instance[0][1].value] in init :
                    #    continue
                    for elem in negative_streams_from_init:
                        if type(instance[0][0]) == opt_type or  type(instance[0][1]) == opt_type :
                            break
                        if (elem[1] == instance[0][0].value).all() and (elem[2] == instance[0][1].value).all():
                            found = True
                            break
                    if found == True:
                        continue

                    if type(instance[0][0]) == opt_type:
                        #ic (instance[0][0].__dict__)
                        arg_1 = instance[0][0].repr_name
                    else :
                        arg_1 = instance[0][0].value
                    if type(instance[0][1]) == opt_type:
                        arg_2 = instance[0][1].repr_name
                    else:
                        arg_2 = instance[0][1].value

                    #init.append(['cfree',instance[0][0].value,instance[0][1].value])
                    init.append(['cfree',arg_1, arg_2])

    return init


def add_negative_to_init_v2(init,negative):
    for neg in negative :
        negative_facts = []
        blocked_predicate = neg.blocked_predicate
        number_objects = len(neg.domain)
        domain_types = {}
        ic (neg.domain)
        ic (neg.domain[0])
        #domain_types_found = {}
        #domain_types_position = {}
        #ic (neg.certified)
        #ic (neg.certified[0])
        #ic (list(neg.certified[0][1:]))
        certified = list(neg.certified[0][1:])
        for i,domain in enumerate(neg.domain):
            #domain_types.append((domain[0],len(domain)-1))
            domain_types[domain[0]] = len(domain) -1
            #domain_types[domain[0]] = len(domain) -1
            domain_types[domain[0]] = []
            #ic (neg.certified[0][1:])
            for j,obj in enumerate(domain[1:]):
                if obj in certified:
                    domain_types[domain[0]].append(j)
                    certified.remove(obj)
            #for obj_info in  neg.certified[0][1:]:
            #ic (obj_info)
            #ic (ic (domain[1:]))
            #useful_pos =
            #    if obj_info in domain[1:]:
            #        domain_types[domain[0]].append(domain[1:].index(obj_info))
            '''
            domain_types_found[domain[0]] = -1
            if domain[0] in domain_types_position:
                domain_types_position[domain[0]].append(i)
            else :
                domain_types_position[domain[0]] = [i]
            '''

        #ic (negative)
        ic(domain_types)
        domain_type_objects = {}
        #ic (init)
        flag = 0
        for elem in init:
            #ic (elem)
            #if elem[0] == 'cfreeapproachpose':
            #if neg.certified[0][0] == 'cfreeapproachpose':
            #    flag = 1
            if elem[0] in domain_types:
                #domain_types_found[elem[0]] = 1
                domain_type_objects[elem[0]] = []
                if elem[0] in domain_type_objects:
                    #domain_type_objects[elem[0]].append(elem[1:1+domain_types[elem[0]]])
                    for pos in domain_types[elem[0]]:
                        domain_type_objects[elem[0]].append(elem[1+pos])
                #else :
                #    #domain_type_objects[elem[0]] = [elem[1]]
                #    for pos in domain_types[elem[0]]:
                #        domain_type_objects[elem[0]].append(elem[1+pos])
                #    #domain_type_objects[elem[0]] = [elem[1:1+domain_types[elem[0]]]]
            if elem[0] == blocked_predicate:
                negative_facts.append(elem)

        #ic (neg)
        facts = generate_object_combinations(neg.certified,domain_types,domain_type_objects,domain_types_position,len(neg.domain))

        for fact in facts :
            fact_in_negative = False
            #ic(fact)
            for negative_fact in negative_facts:
                neg_fact_found = True
                #ic (negative_fact)
                if len(fact) == len(negative_fact) :
                    for pos_fact_obj, negative_fact_obj in zip(fact[1:],negative_fact[1:]):
                        #ic (pos_fact_obj,negative_fact_obj)
                        if type (pos_fact_obj) == np.ndarray:
                            #ic ()
                            #ic ((pos_fact_obj == negative_fact_obj).all())
                            if not ((pos_fact_obj==negative_fact_obj).all()) :
                                neg_fact_found = False
                                break
                        else :
                            if pos_fact_obj != negative_fact_obj:
                                neg_fact_found = False
                                break
                    #ic (neg_fact_found)
                    if neg_fact_found == True :
                        fact_in_negative = True
                        break
            if fact_in_negative == True :
                continue
            else :
                init.append(fact)
            #ic ("end of checking this fact")
        #ic (facts)
        #ic (init)
    return init

def add_negative_to_init(init, negative):
    for neg in negative :
        negative_facts = []
        blocked_predicate = neg.blocked_predicate
        number_objects = len(neg.domain)
        domain_types = {}
        #ic (neg.domain)
        #ic (neg.domain[0])
        domain_types_found = {}
        domain_types_position = {}
        #ic (neg.certified)
        #ic (neg.certified[0])
        #ic (list(neg.certified[0][1:]))
        certified = list(neg.certified[0][1:])
        for i,domain in enumerate(neg.domain):
            #domain_types.append((domain[0],len(domain)-1))
            #domain_types[domain[0]] = len(domain) -1
            domain_types[domain[0]] = []
            #ic (neg.certified[0][1:])
            for j,obj in enumerate(domain[1:]):
                if obj in certified:
                    domain_types[domain[0]].append(j)
                    certified.remove(obj)
            #for obj_info in  neg.certified[0][1:]:
                #ic (obj_info)
                #ic (ic (domain[1:]))
                #useful_pos =
            #    if obj_info in domain[1:]:
            #        domain_types[domain[0]].append(domain[1:].index(obj_info))
            domain_types_found[domain[0]] = -1
            if domain[0] in domain_types_position:
                domain_types_position[domain[0]].append(i)
            else :
                domain_types_position[domain[0]] = [i]

        #ic (negative)
        #ic(domain_types)
        domain_type_objects = {}
        #ic (init)
        flag = 0
        for elem in init:
            #ic (elem)
            #if elem[0] == 'cfreeapproachpose':
            #if neg.certified[0][0] == 'cfreeapproachpose':
            #    flag = 1
            if elem[0] in domain_types:
                #ic (elem[0])
                domain_types_found[elem[0]] = 1
                #ic (domain_type_objects)
                #domain_type_objects[elem[0]] = []
                if elem[0] in domain_type_objects:
                    #domain_type_objects[elem[0]].append(elem[1:1+domain_types[elem[0]]])
                    for pos in domain_types[elem[0]]:
                        domain_type_objects[elem[0]].append(elem[1+pos])
                else :
                    domain_type_objects[elem[0]] = [elem[1]]
                    for pos in domain_types[elem[0]][1:]:
                        domain_type_objects[elem[0]].append(elem[1+pos])
                    #domain_type_objects[elem[0]] = [elem[1:1+domain_types[elem[0]]]]
            if elem[0] == blocked_predicate:
                negative_facts.append(elem)

        domain_not_satisfied = False
        for key,value in domain_types_found.items():
            if value == -1 :
                domain_not_satisfied = True
                break
        #ic (neg)
        #ic (neg.__dict__)
        #ic (neg.certified,domain_types,domain_type_objects)
        if domain_not_satisfied == True :
            ic (domain_not_satisfied)
            return init
        facts = generate_object_combinations(neg.certified,domain_types,domain_type_objects,domain_types_position,len(neg.domain))
        if flag == 1 :
            exit()
        #exit()

        for fact in facts :
            fact_in_negative = False
            #ic(fact)
            for negative_fact in negative_facts:
                neg_fact_found = True
                #ic (negative_fact)
                if len(fact) == len(negative_fact) :
                    for pos_fact_obj, negative_fact_obj in zip(fact[1:],negative_fact[1:]):
                        #ic (pos_fact_obj,negative_fact_obj)
                        if type (pos_fact_obj) == np.ndarray:
                            #ic ()
                            #ic ((pos_fact_obj == negative_fact_obj).all())
                            if not ((pos_fact_obj==negative_fact_obj).all()) :
                                neg_fact_found = False
                                break
                        else :
                            if pos_fact_obj != negative_fact_obj:
                                neg_fact_found = False
                                break
                    #ic (neg_fact_found)
                    if neg_fact_found == True :
                        fact_in_negative = True
                        break
            if fact_in_negative == True :
                continue
            else :
                init.append(fact)
            #ic ("end of checking this fact")
        #ic (facts)
        #ic (init)
    return init

def generate_object_combinations(certified,domain_types,domain_type_objects,domain_types_position,number_domains):
    lists_to_use = [[certified[0][0]]]

    #ic (lists_to_use)
    #ic (domain_types)
    #ic (domain_type_objects)
    #lists_to_use_arguments_from_types = [domain_type_objects[domain] for domain in domain_types.keys() if domain in domain_type_objects]
    #lists_to_use_arguments = [domain_type_objects[domain] for domain in domain_types.keys() if domain in domain_type_objects]
    #ic (max(domain_types_position.values()))
    lists_to_use_arguments = [None] * number_domains

    #for i,argument_lists in enumerate(lists_to_use_arguments):
    #
    #    domain_types_position
    for domain,positions in domain_types_position.items():
        for position in positions :
            #if len(domain_type_objects[domain]) > 1 :
            #    lists_to_use_arguments[position] = [domain_type_objects[domain]]
            #else :
            lists_to_use_arguments[position] = domain_type_objects[domain]

    lists_to_use += lists_to_use_arguments
    #ic (lists_to_use_arguments)
    #ic (lists_to_use)
    #ic (lists_to_use)
    facts = list(itertools.product(*lists_to_use))
    #ic (facts)
    #ic (len(facts))
    new_facts = []
    for fact in facts:
        new_fact = []
        for elem in fact:
            if type(elem) != list:
                new_fact.append(elem)
            else :
                for obj in elem:
                    new_fact.append(obj)
        new_facts.append(tuple(new_fact))
    #ic (new_facts)
    return new_facts
    #return facts

def get_objects_from_predicate(predicate,added_objects,obj_find_phase=False):
    #predicate_write_string = ""
    '''
    This function creates a string name for all the variables
    Currently only handles when the continuous values have 2 numbers and nothing else
    '''
    #ic (predicate)
    #ic (added_objects)

    kuka_primitives = examples.pybullet.utils.pybullet_tools.kuka_primitives
    kuka_primitives_all = [examples.pybullet.utils.pybullet_tools.kuka_primitives.BodyPose,
                       examples.pybullet.utils.pybullet_tools.kuka_primitives.BodyGrasp,
                       examples.pybullet.utils.pybullet_tools.kuka_primitives.BodyConf,
                       examples.pybullet.utils.pybullet_tools.kuka_primitives.BodyPath
                       ]
    '''
    kuka_primitives_all = []
    '''

    objects = []
    elem = predicate
    ##ic (predicate)
    #ic (added_objects)
    obj1_start_str = None
    obj2_start_str = None
    obj3_start_str = None
    obj_1_cont = False
    obj_2_cont = False
    obj_3_cont = False
    sep_string = "_"
    conversion = {}
    #for objs in elem[1:]:
    #    conversion[objs] = None
    #for arg in
    elem_values = [None]
    #ic (elem)
    for arg in elem[1:]:
        #ic (arg)
        #ic (type(arg))
        #ic (arg.__dict__)
        if type(arg) == pddlstream.language.object.OptimisticObject:
            #init_from_eval_curr[arg_idx + 1] = arg.pddl[1:]
            #opt_obj_conversion[arg.pddl[1:]] = arg.pddl
            #ic ("Opt obj", arg)
            elem_values.append(arg.pddl[1:])
        else:
            #ic (arg.__dict__)
            #ic (type(arg.value))
            #if type(arg.value) != np.ndarray:
            if type(arg.value) in kuka_primitives_all:
                #ic(arg.value.__dict__)
                if type(arg.value) == kuka_primitives.BodyPose :
                    elem_values.append(arg.value.pose[0])
                if type(arg.value) == kuka_primitives.BodyConf :
                    elem_values.append(arg.value.configuration)

            else :
                #ic (arg.value.__dict__)
                elem_values.append(arg.value)
                #init_from_eval_curr[arg_idx + 1] = arg.value

    #ic (elem)
    #ic (elem_values)
    #ic (elem)
    #ic (elem_values)
    #obj_1_cont = False
    #obj_2_cont = False
    obj_start_str, obj_cont = get_cont_information(elem,domain_name="kuka")
    #obj_start_str, obj_cont = get_cont_information(elem,domain_name="discrete_tamp_3d")
    #ic (elem_values)

    if obj_cont == None and obj_start_str == []:
        return [],None

    if len(elem) == 1:
        return [],None
    for i in range(1,len(elem)):
        if "cost" in str(elem[i]) \
                or "dist" in str(elem[i]):
            return objects,None
        if obj_cont[i] == True:
            #obj_str_to_add = obj1_start_str + sep_string + str(elem_values[1][0]) + sep_string + str(elem_values[1][1])
            obj_str_to_add = obj_start_str[i]
            ic (i)
            ic (obj_str_to_add )
            ic (elem)
            ic (elem_values)
            for str_component in elem_values[i]:
                obj_str_to_add  += sep_string
                obj_str_to_add += str(str_component)
        else:
            obj_str_to_add = str(elem_values[i])
        if obj_find_phase :
            if obj_str_to_add not in added_objects:
                objects.append(obj_str_to_add)
                #added_objects.append(str(elem[1]))
                added_objects.append(obj_str_to_add)
                conversion[obj_str_to_add] = elem[i]
        else:
            objects.append(obj_str_to_add)
        pass

    ic (elem,conversion)
    if obj_find_phase:
        return objects, conversion
    return objects, None

    if len(elem) == 1:
        return [],None
    elif len(elem) == 2:
        if "cost" in str(elem[1]) \
            or "dist" in str(elem[1]):
            return objects,None
        if obj_1_cont == True:
            #obj_str_to_add = obj1_start_str + sep_string + str(elem_values[1][0]) + sep_string + str(elem_values[1][1])
            obj_str_to_add = obj1_start_str
            for str_component in elem_values[1]:
                obj_str_to_add  += sep_string
                obj_str_to_add += str(str_component)
        else:
            obj_str_to_add = str(elem_values[1])
        if obj_find_phase :
            if obj_str_to_add not in added_objects:
                objects.append(obj_str_to_add)
                #added_objects.append(str(elem[1]))
                added_objects.append(obj_str_to_add)
                conversion[obj_str_to_add] = elem[1]
        else:
            objects.append(obj_str_to_add)
    elif len(elem) == 3:
        if "cost" in str(elem[1]) or "cost" in str(elem[2]) \
            or "dist" in str(elem[1]) or "dist" in str(elem[2]):
            #continue
            return objects,None
        if obj_1_cont == True:
            #obj_str_to_add = obj1_start_str + sep_string + str(elem_values[1][0]) + sep_string + str(elem_values[1][1])
            obj_str_to_add = obj1_start_str
            for str_component in elem_values[1]:
                obj_str_to_add  += sep_string
                obj_str_to_add += str(str_component)

        else:
            obj_str_to_add = str(elem_values[1])
        #if str(elem[1]) not in added_objects:
        if obj_find_phase :
            if obj_str_to_add not in added_objects:
                objects.append(obj_str_to_add)
                added_objects.append(obj_str_to_add)
                conversion[obj_str_to_add] = elem[1]
        else :
            objects.append(obj_str_to_add)
        if obj_2_cont == True:
            #obj_str_to_add = obj2_start_str + sep_string + str(elem_values[2][0]) + sep_string + str(elem_values[2][1])
            obj_str_to_add = obj2_start_str
            for str_component in elem_values[2]:
                obj_str_to_add  += sep_string
                obj_str_to_add += str(str_component)
        else:
            obj_str_to_add = str(elem_values[2])
        #if str(elem[2]) not in added_objects:
        if obj_find_phase :
            if obj_str_to_add not in added_objects:
                objects.append(obj_str_to_add)
                added_objects.append(obj_str_to_add)
                conversion[obj_str_to_add] = elem[2]
        else :
            objects.append(obj_str_to_add)

    #ic (added_objects)
    #ic (elem)
    #ic (objects)

    if obj_find_phase :
        return objects,conversion
    return objects,None
    #pass

#def write_init_to_file(init, goal):
def generate_pddl_from_init_goal(init, goal,problem='unity_1'):

    #ic ("writing to file")
    file_write_str = ""
    #file_write_str += "(define (problem unity_1)\n(:domain unity_1) \n (:objects\n"
    file_write_str += "(define (problem " +problem+")\n(:domain "+problem+") \n (:objects\n"
    obj_conversions = {}
    len (init)

    added_objects = []
    for elem in init:
        elem = list(elem)
        #ic (elem)
        #predicate_write_string = get_write_string_for_predicate(elem,added_objects)
        current_objects,init_conversion = get_objects_from_predicate(elem, added_objects,True )
        #obj_conversions = Merge(obj_conversions,init_conversion)
        if init_conversion != None:
            obj_conversions.update(init_conversion)
        #ic (elem)
        #ic (current_objects)
        for obj in current_objects:
            #file_write_str += str(obj) + "\n"
            file_write_str += str(obj) + " - obj" + "\n"

    #exit()
    #ic (added_objects)
    for goal_pred in goal:
        if goal_pred == 'and':
            continue
        goal_pred = list(goal_pred)
        #ic (goal_pred)
        current_objects,goal_conversion = get_objects_from_predicate(goal_pred, added_objects,True)
        if goal_conversion != None :
            obj_conversions.update(goal_conversion)
        for obj in current_objects:
            #file_write_str += str(obj) + "\n"
            file_write_str += str(obj) + " - obj" + "\n"
    '''
    if len(goal_pred) == 2:
        if str(goal_pred[1]) not in added_objects:
            file_write_str += str(goal_pred[1]) + "\n"
            #f.write(str(goal_pred[1]) + "\n")
            added_objects.append(str(goal_pred[1]))
    elif len(goal_pred) == 3:
        if str(goal_pred[1]) not in added_objects :
            file_write_str += str(goal_pred[1]) + "\n"
            #f.write(str(goal_pred[1])+"\n")
            added_objects.append(str(goal_pred[1]))
        if str(goal_pred[2]) not in added_objects :
            file_write_str += str(goal_pred[2]) + "\n"
            #f.write(str(goal_pred[2])+"\n")
            added_objects.append(str(goal_pred[2]))
    '''

    #ic (added_objects)
    #exit()
    file_write_str += "\n)\n(:init\n"
    #f.write("\n)")
    #f.write("\n(:init\n")

    for elem in init:
        elem = list(elem)
        '''
        new_elem = [elem[0]]
        for arg in elem[1:] :
            if type(arg) == pddlstream.language.object.OptimisticObject:
                #init_from_eval_curr[arg_idx + 1] = arg.pddl[1:]
                #opt_obj_conversion[arg.pddl[1:]] = arg.pddl
                new_elem.append(arg.pddl[1:])
            else:
                #init_from_eval_curr[arg_idx + 1] = arg.value
                new_elem.append(arg.value)
        #elem = elem.split(",")
        elem = new_elem[:]
        '''
        #ic (elem)

        cost_flag = 0
        file_write_str_temp = ""
        if len (elem) == 1 :
            if "cost" in str(elem[0]) or "dist" in str(elem[0]):  # or "cost" in str(elem[2]):
                    continue

            write_str = "\n(" + str(elem[0])  + ")"
            file_write_str += "\n(" + str(elem[0])  + ")"
        else:
            file_write_str_temp += "\n("
            write_str = "\n("

            for item in elem :
                if "Cost" in str(item) or "cost" in str(item) \
                        or "Dist" in str(item) or "dist" in str(item):
                    cost_flag = 1
                    break
            current_objects,_ = get_objects_from_predicate(elem, [])

            if "test-cfree-negative" in elem[0]:
                file_write_str_temp += "not (cfree"  # + " "+ str(elem[1]) + ")"
                write_str += "not (cfree"  # + " "+ str(elem[1]) + ")"
            else :
                file_write_str_temp += " " + str(elem[0])  # + " "+ str(elem[1]) + ")"
                write_str += " " + str(elem[0])  # + " "+ str(elem[1]) + ")"
            #ic (current_objects)
            for item in current_objects:
                file_write_str_temp += " " + item #+ " "+ str(elem[1]) + ")"
                write_str += " " + item #+ " "+ str(elem[1]) + ")"
            file_write_str_temp += ")"
            write_str += ")"
            if "test-cfree-negative" in elem[0]:
                file_write_str_temp += ")"  # + " "+ str(elem[1]) + ")"
                write_str += ")"  # + " "+ str(elem[1]) + ")"
        if cost_flag == 0 :
            file_write_str += file_write_str_temp
            #f.write(write_str)

    file_write_str += "\n)\n(:goal\n(and "
    #f.write("\n)")
    #f.write("\n(:goal\n")
    #f.write("(and ")

    #ic (goal)
    for goal_pred in goal:
        if goal_pred == 'and':
            continue
        goal_pred = list(goal_pred)
        current_objects,_ = get_objects_from_predicate(goal_pred, [])
        file_write_str += "\n" + "(" + str(goal_pred[0])
        for item in current_objects:
            file_write_str += " " + item  # + " "+ str(elem[1]) + ")"
        file_write_str += ")"

    '''
    if len(goal) == 2:
        #current_objects = get_objects_from_predicate(goal, [])
        file_write_str += "\n" + "("+ str(goal[0]) + " " + str(goal[1]) +")"
        write_str = "\n" + "("+ str(goal[0]) + " " + str(goal[1]) +")"
    if len(goal) == 3:
        file_write_str += "\n" + "(" +  str(goal[0]) + " " + str(goal[1]) + " "+ str(goal[2])+ ")"
        write_str = "\n" + "(" +  str(goal[0]) + " " + str(goal[1]) + " "+ str(goal[2])+ ")"
    #f.write(write_str)
    '''

    file_write_str += "\n)\n)\n)"
    #ic (file_write_str)
    #exit()
    return file_write_str,obj_conversions

def get_cont_information(elem,domain_name):
    if domain_name.lower() == 'discrete_tamp_3d':
        #a,b= discrete_tamp_3d_cont(elem)
        #ic (elem,a,b)
        #return a,b
        return  discrete_tamp_3d_cont(elem)

    if domain_name.lower() == 'discrete_tamp':
        return discrete_tamp_cont(elem)

    if domain_name.lower() == 'kuka':
        return kuka_cont(elem)


def kuka_cont(elem):
    obj_start_str = [None] * 6
    obj_start_str = np.array(obj_start_str)
    obj_cont = [False] * obj_start_str.shape[0]
    obj_cont = np.array(obj_cont)

    pred = elem[0].lower()
    '''
    if pred == 'stackable':
        obj_start_str[2] = 'r'
    elif pred == 'sink':
        obj_start_str[1] = 'r'
    elif pred == 'stove':
        obj_start_str[1] = 'r'
    '''
    if pred == 'pose':
        obj_start_str[2] = 'p'
    elif pred == 'grasp':
        obj_start_str[2] = 'g'
    elif pred == 'kin':
        obj_start_str[2] = 'p'
        obj_start_str[3] = 'g'
        obj_start_str[4] = 'q'
        obj_start_str[5] = 't'
    elif pred == 'freemotion':
        obj_start_str[1] = 'q'
        obj_start_str[2] = 't'
        obj_start_str[3] = 'q'
    elif pred == 'holdingmotion':
        obj_start_str[1] = 'q'
        obj_start_str[2] = 't'
        obj_start_str[3] = 'q'
        obj_start_str[5] = 'g'
    elif pred == 'supported':
        obj_start_str[2] = 'p'
        #obj_start_str[3] = 'r'
    elif pred == 'traj' or pred == 'unsafetraj':
        obj_start_str[1] = 't'
    elif pred == 'trajcollision':
        obj_start_str[1] = 't'
        obj_start_str[3] = 'p'
    elif pred == 'cfreeposepose':
        obj_start_str[2] = 'p'
        obj_start_str[4] = 'p'
    elif pred == 'cfreeapproachpose':
        obj_start_str[2] = 'p'
        obj_start_str[3] = 'g'
        obj_start_str[5] = 'p'
    elif pred == 'cfreetrajpose':
        obj_start_str[1] = 't'
        obj_start_str[3] = 'p'
    elif pred == "atpose":
        obj_start_str[2] = 'p'
    elif pred == 'atgrasp':
        obj_start_str[2] = 'g'
    elif pred == "conf" or pred == "atconf":
        obj_start_str[1] = 'q'
    #elif pred == 'on':
    #    obj_start_str[2] = 'r'
    elif pred == 'unsafepose':
        obj_start_str[2] = 'p'
    elif pred == 'unsafeapproach':
        obj_start_str[2] = 'p'
        obj_start_str[3] = 'g'
    elif "distance" in elem[0]:
        return [], None

    obj_cont[np.where(obj_start_str!=None)] = True
    #ic (obj_cont,obj_start_str)

    return obj_start_str.tolist(),obj_cont.tolist()

def discrete_tamp_cont(elem):
    return discrete_tamp_3d_cont(elem)

def discrete_tamp_3d_cont(elem):
    obj_cont = [False] * 4
    obj_start_str = [None] * 4

    #if elem[0] == "Conf" or elem[0] == "AtConf":
    if elem[0].lower() == "conf" or elem[0].lower() == "atconf":
        #obj1_start_str = 'q'
        #obj_1_cont = True
        obj_start_str[1] = 'q'
        obj_cont[1] = True
    if elem[0].lower() == 'motion':
        obj_start_str[1] = 'q'
        obj_start_str[2] = 't'
        obj_start_str[3] = 'q'
        obj_cont[1] = True
        obj_cont[2] = True
        obj_cont[3] = True
    #elif elem[0] == "Pose" or elem[0] == "Unsafe":
    elif elem[0].lower() == "pose" or elem[0].lower() == "unsafe":
        #obj1_start_str = 'p'
        #obj_1_cont = True
        obj_start_str[1] = 'p'
        obj_cont[1] = True
    #elif elem[0] == "AtPose":
    elif elem[0].lower() == "traj":
        obj_start_str[1] = 't'
        obj_cont[1] = True
    elif elem[0].lower() == "atpose":
        #obj1_start_str = 'p'
        #obj2_start_str = 'p'
        #obj_2_cont = True
        obj_start_str[2] = 'p'
        obj_cont[2] = True
    #elif elem[0] == "Kin":
    elif elem[0].lower() == "kin":
        #obj1_start_str = 'q'
        #obj2_start_str = 'p'
        #obj_1_cont = True
        #obj_2_cont = True
        obj_start_str[1] = 'q'
        obj_start_str[2] = 'p'
        obj_cont[1] = True
        obj_cont[2] = True
    #elif elem[0] == "Collision" or elem[0] == "CFree":
    elif elem[0].lower() == "collision" or elem[0].lower() == "cfree":
        #obj1_start_str = 'p'
        #obj2_start_str = 'p'
        #obj_1_cont = True
        #obj_2_cont = True
        obj_start_str[1] = 'p'
        obj_start_str[2] = 'p'
        obj_cont[1] = True
        obj_cont[2] = True
    elif "test-cfree-negative" in elem[0]:
        #obj1_start_str = 'p'
        #obj2_start_str = 'p'
        #obj_1_cont = True
        #obj_2_cont = True
        obj_start_str[1] = 'p'
        obj_start_str[2] = 'p'
        obj_cont[1] = True
        obj_cont[2] = True
    elif "distance" in elem[0]:
        return [],None

    return obj_start_str,obj_cont


def same_arg(stream_var,gym_var):
    #ic (gym_var)
    #ic (list(gym_var))
    gym_var = gym_var.split(":")[0]

    if type(stream_var) == np.ndarray:
        gym_var = gym_var.split("_")
        #ic (gym_var)
        #ic (stream_var)
        gym_var = np.array([int(elem) for elem in gym_var if type(elem) == str if elem.lstrip("-").isdigit()])
        #ic (gym_var)
        #ic (np.array_equal(gym_var, stream_var))
        #return True
        return (np.array_equal(gym_var, stream_var))

    if type(stream_var) == str:
        #ic (stream_var)
        #ic (gym_var)
        #ic (type(gym_var))
        return stream_var == gym_var

def action_in_groundings(action,groundings):
    action_to_take = None

    for grounding in groundings:
        ic(grounding.__dict__)
        found = True
        if grounding.predicate == action.name:
            for stream_var, gym_var in zip(action.args, grounding.variables):
                # ic (variable)
                ic (stream_var,gym_var)
                if not same_arg(stream_var, gym_var):
                    found = False
                    break
            if found == True:
                action_to_take = grounding
                break
        if found == False:
            break

def is_action_possible(env,pddlstream_action,gym_state):
    groundings = env.action_space.all_ground_literals(gym_state)
    # ic (state)
    groundings_list = []
    #action_to_take = None
    # ic (type(groundings))
    for grounding in groundings:
        grounding_action = grounding.predicate
        objects = grounding.variables
        groundings_list.append(pddlgym.structs.Literal(grounding_action, objects))

    # ic (groundings_list)
    action_to_take = action_in_groundings(pddlstream_action,groundings)
    return action_to_take

def pddlstream_from_pddlgym(state):

    #ic (state.literals)
    #ic (state.objects)
    init = []
    goal = []
    #numerical_predicates = {"conf":[0],"AtConf":[0], }
    '''
    assert(1 <= n_blocks <= n_poses)
    blocks = [BLOCK_TEMPLATE.format(i) for i in range(n_blocks)]
    poses = [np.array([x, 0]) for x in range(n_poses)]

    block_poses = dict(zip(blocks, poses))
    initial = DiscreteTAMPState(INITIAL_CONF, None, block_poses)
    goal_poses = {blocks[0]: poses[1]}
    '''

    for predicate in state.literals :
        #ic (predicate.__dict__)
        to_add = pddlgym_predicate_to_pddlstream_predicate(predicate)
        if to_add != None:
            init.append(to_add)

    #ic (init)

    #ic (state.goal)
    #ic (state.goal.__dict__)
    for predicate in state.goal.literals:
        to_add = pddlgym_predicate_to_pddlstream_predicate(predicate)
        if to_add != None:
            goal.append(to_add)

    if len(goal) == 1:
        pddlstream_goal = goal[0]
    else:
        pddlstream_goal = ('and',) + tuple(goal)
    #pddl_stream_goal = And(*[elem for elem in goal])
    #ic (init)
    #ic (init[2])
    #ic (init[2][1])
    #ic (type(init[2][1]))
    #exit()
    #init.append(Equal((TOTAL_COST,), 0))
    ic (init)
    ic (pddlstream_goal)
    #exit()
    return init,pddlstream_goal

def pddlgym_predicate_to_pddlstream_predicate(predicate):
    numerical_positions = {"conf":[0],"atconf":[0],"pose":[0],"atpose":[1],"block":[],"unsafe":[0],"cfree":[0,1],"kin":[0,1],"Notcfree":[0,1]}
    non_numerical_positions = {"conf":[],"atconf":[],"pose":[],"atpose":[0],"block":[0],"unsafe":[],"cfree":[],"kin":[],"Notcfree":[]}
    pddlgym_to_pddlstream_predicate = {"conf":"Conf", "atconf":"AtConf","pose":"Pose","atpose":"AtPose",
                                       "block":"Block","unsafe":"Unsafe","canmove":"CanMove",
                                       "handempty":"HandEmpty","cfree":"CFree","kin":"Kin","Notcfree": "Not Cfree"}

    #ic (predicate.__dict__)
    #ic (pddlgym_to_pddlstream_predicate['cfree'])
    #ic ()

    derived_predicates = ["unsafe"]
    added_cont_values = []
    if predicate.predicate in derived_predicates:
        return None
    if len(predicate.variables) == 0:
        # init.append((str(predicate.predicate).capitalize(),))
        return (pddlgym_to_pddlstream_predicate[str(predicate.predicate)],)
    else:
        pddlstream_predicate = [None] * (len(predicate.variables) + 1)
        pddlstream_predicate[0] = pddlgym_to_pddlstream_predicate[predicate.predicate]

        #ic (predicate.predicate)
        for position in numerical_positions[str(predicate.predicate)]:
            values = predicate.variables[position].split("_")
            #ic (values)
            int_values = [int(elem) if elem.isnumeric() else elem for elem in values[1:]]


            existing_index = [idx for idx, el in enumerate(added_cont_values) if np.array_equal(el, int_values)]
            # if int_values not in added_cont_values :
            if len(existing_index) == 0:
                added_cont_values.append(int_values)
            else:
                int_values = added_cont_values[existing_index[0]]

            #ic (int_values)
            #ic ([True if type(elem) == int else False for elem in int_values])
            if all([True if type(elem) == int else False for elem in int_values]) :
                pddlstream_predicate[position + 1] = np.array(int_values)
            else:
                pddlstream_predicate[position + 1] = str(predicate.variables[position].split(":")[0])


        for position in non_numerical_positions[str(predicate.predicate)]:
            # pddlstream_predicate.append(str(predicate.variables[position].split(":")[0]))
            pddlstream_predicate[position + 1] = str(predicate.variables[position].split(":")[0])
        return tuple(pddlstream_predicate)

def pddlgym_plan_to_pddlstream_plan(pddlgym_plan,renamed_plan,opt_evaluations,mapping=None):
    pddlstream_plan = []
    a = Action('abc',[1,2,3])
    '''
    ic (opt_evaluations)
    for eval in opt_evaluations:
        ic (eval)
        ic (eval.head)
        ic(eval.head.args)
    #ic (a)
    #ic (type(a))
    '''

    for action in renamed_plan:
        ic (action)
        ic (type(action))
        #ic (action.__dict__)

    for action in pddlgym_plan:
        ic (action)
        #ic (action.__dict__)
        args = action.variables[:]
        pddlstream_action = Action(str(action.predicate),args)
        pddlstream_plan.append(pddlstream_action)
    '''
    ic (renamed_plan)
    ic (pddlstream_plan)
    exit()
    '''

    return pddlstream_plan


##################################################

class Saver(object):
    # TODO: contextlib
    def save(self):
        raise NotImplementedError()
    def restore(self):
        raise NotImplementedError()
    def __enter__(self):
        # TODO: move the saving to enter?
        self.save()
        return self
    def __exit__(self, type, value, traceback):
        self.restore()


class Profiler(Saver):
    fields = ['tottime', 'cumtime']
    def __init__(self, field='tottime', num=10):
        assert field in self.fields
        self.field = field
        self.num = num
        self.pr = cProfile.Profile()
    def save(self):
        self.pr.enable()
        return self.pr
    def restore(self):
        self.pr.disable()
        if self.num is None:
            return None
        stream = None
        #stream = io.StringIO()
        stats = pstats.Stats(self.pr, stream=stream).sort_stats(self.field) # TODO: print multiple
        stats.print_stats(self.num)
        return stats


class Verbose(Saver): # TODO: use DisableOutput
    def __init__(self, verbose=False):
        self.verbose = verbose
    def save(self):
        if self.verbose:
            return
        self.stdout = sys.stdout
        self.devnull = open(os.devnull, 'w')
        sys.stdout = self.devnull
        #self.stderr = sys.stderr
        #self.devnull = open(os.devnull, 'w')
        #sys.stderr = self.stderr
    def restore(self):
        if self.verbose:
            return
        sys.stdout = self.stdout
        self.devnull.close()
        #sys.stderr = self.stderr
        #self.devnull.close()


class TmpCWD(Saver):
    def __init__(self, temp_cwd):
        self.tmp_cwd = temp_cwd
    def save(self):
        self.old_cwd = os.getcwd()
        os.chdir(self.tmp_cwd)
    def restore(self):
        os.chdir(self.old_cwd)

##################################################

class Comparable(object):
    def __lt__(self, other):
        raise NotImplementedError()
    def __eq__(self, other):
        return not (self < other) and not (other < self)
    def __ne__(self, other):
        return (self < other) or (other < self)
    def __gt__(self, other):
        return other < self
    def __ge__(self, other):
        return not self < other
    def __le__(self, other):
        return not other < self

class MockSet(object):
    def __init__(self, test=lambda item: True):
        self.test = test
    def __contains__(self, item):
        return self.test(item)

class Score(Comparable): # tuple
    def __init__(self, *args):
        # TODO: convert to float
        #super(Score, self).__init__(args)
        self.values = tuple(args)
    def check_other(self, other):
        return isinstance(other, Score) and (len(self.values) == len(other.values))
    def __lt__(self, other):
        assert self.check_other(other)
        return self.values < other.values
    def __iter__(self):
        return iter(self.values)
    def __neg__(self):
        return self.__class__(*(type(value).__neg__(value) for value in self.values))
    def __add__(self, other):
        return self.__class__(*(self.values + other.values))
    def __repr__(self):
        return '{}{}'.format(self.__class__.__name__, self.values)

class HeapElement(Comparable):
    def __init__(self, key, value):
        self.key = key
        self.value = value
    def __lt__(self, other):
        return self.key < other.key
    def __iter__(self):
        return iter([self.key, self.value])
    def __repr__(self):
        return '{}({}, {})'.format(self.__class__.__name__, self.key, self.value)

##################################################

def sorted_str_from_list(obj, **kwargs):
    return '[{}]'.format(', '.join(sorted(str_from_object(item, **kwargs) for item in obj)))

def str_from_object(obj, ndigits=None):  # str_object
    if type(obj) in [list]: #, np.ndarray):
        return '[{}]'.format(', '.join(str_from_object(item, ndigits) for item in obj))
    if type(obj) == tuple:
        return '({})'.format(', '.join(str_from_object(item, ndigits) for item in obj))
    #if isinstance(obj, dict):
    if type(obj) in [dict, defaultdict, Counter]:
        return '{{{}}}'.format(', '.join('{}: {}'.format(str_from_object(key, ndigits), str_from_object(obj[key], ndigits)) \
                                  for key in sorted(obj.keys(), key=lambda k: str_from_object(k, ndigits))))
    if type(obj) in [set, frozenset]:
        return '{{{}}}'.format(', '.join(sorted(str_from_object(item, ndigits) for item in obj)))
    if (ndigits is not None) and (type(obj) in [float, np.float64]):
        obj = round(obj, ndigits=ndigits)
        if obj == 0.:
            obj = 0.  # NOTE - catches -0.0 bug
        return '{0:.{1}f}'.format(obj, ndigits)
    #if isinstance(obj, types.FunctionType):
    #    return obj.__name__
    return str(obj)
    #return repr(obj)

##################################################

def incoming_from_edges(edges):
    incoming_vertices = defaultdict(set)
    for v1, v2 in edges:
        incoming_vertices[v2].add(v1)
    return incoming_vertices

def outgoing_from_edges(edges):
    outgoing_vertices = defaultdict(set)
    for v1, v2 in edges:
        outgoing_vertices[v1].add(v2)
    return outgoing_vertices

def neighbors_from_orders(orders):
    return incoming_from_edges(orders), \
           outgoing_from_edges(orders)

def adjacent_from_edges(edges):
    undirected_edges = defaultdict(set)
    for v1, v2 in edges:
        undirected_edges[v1].add(v2)
        undirected_edges[v2].add(v1)
    return undirected_edges

##################################################

def filter_orders(vertices, orders):
    # TODO: rename to filter edges?
    return [order for order in orders if all(v in vertices for v in order)]

def is_valid_topological_sort(vertices, orders, solution):
    orders = filter_orders(vertices, orders)
    if Counter(vertices) != Counter(solution):
        return False
    index_from_vertex = {v: i for i, v in enumerate(solution)}
    for v1, v2 in orders:
        if index_from_vertex[v1] >= index_from_vertex[v2]:
            return False
    return True

def dfs_topological_sort(vertices, orders, priority_fn=lambda v: 0):
    # TODO: DFS for all topological sorts
    orders = filter_orders(vertices, orders)
    incoming_edges, outgoing_edges = neighbors_from_orders(orders)

    def dfs(history, visited):
        reverse_ordering = []
        v1 = history[-1]
        if v1 in visited:
            return reverse_ordering
        visited.add(v1)
        for v2 in sorted(outgoing_edges[v1], key=priority_fn, reverse=True):
            if v2 in history:
                return None # Contains a cycle
            result = dfs(history + [v2], visited)
            if result is None:
                return None
            reverse_ordering.extend(result)
        reverse_ordering.append(v1)
        return reverse_ordering

    visited = set()
    reverse_order = []
    for v0 in sorted(vertices, key=priority_fn, reverse=True):
        if not incoming_edges[v0]:
            result = dfs([v0], visited)
            if result is None:
                return None
            reverse_order.extend(result)

    ordering = reverse_order[::-1]
    assert(is_valid_topological_sort(vertices, orders, ordering))
    return ordering

def topological_sort(vertices, orders, priority_fn=lambda v: 0):
    orders = filter_orders(vertices, orders)
    incoming_edges, outgoing_edges = neighbors_from_orders(orders)
    ordering = []
    queue = []
    for v in vertices:
        if not incoming_edges[v]:
            heappush(queue, HeapElement(priority_fn(v), v))
    while queue:
        priority, v1 = heappop(queue) # Lowest to highest
        ordering.append(v1)
        for v2 in outgoing_edges[v1]:
            incoming_edges[v2].remove(v1)
            if not incoming_edges[v2]:
                heappush(queue, HeapElement(priority_fn(v2), v2))
    if len(ordering) != len(vertices):
        return None
    assert is_valid_topological_sort(vertices, orders, ordering)
    return ordering

def layer_sort(vertices, orders): # priority_fn=lambda v: 0
    # TODO: more efficient hypergraph/layer distance (h_max)
    orders = filter_orders(vertices, orders)
    incoming_edges, outgoing_edges = neighbors_from_orders(orders)
    visited = {}
    queue = []
    for v in vertices:
        if not incoming_edges[v]:
            visited[v] = 0
            heappush(queue, HeapElement(visited[v], v))
    while queue:
        g, v1 = heappop(queue)
        for v2 in outgoing_edges[v1]:
            incoming_edges[v2].remove(v1) # TODO: non-uniform cost function for max
            if not incoming_edges[v2] and (v2 not in visited):
                visited[v2] = g + 1
                heappush(queue, HeapElement(visited[v2], v2))
    return visited

def is_acyclic(vertices, orders):
    return topological_sort(vertices, orders) is not None

def sample_topological_sort(vertices, orders):
    # https://stackoverflow.com/questions/38551057/random-topological-sorting-with-uniform-distribution-in-near-linear-time
    # https://www.geeksforgeeks.org/all-topological-sorts-of-a-directed-acyclic-graph/
    priorities = {v: random.random() for v in vertices}
    return topological_sort(vertices, orders, priority_fn=priorities.get)

def transitive_closure(vertices, orders):
    # Warshall's algorithm
    orders = filter_orders(vertices, orders)
    closure = set(orders)
    for k in vertices:
        for i in vertices:
            for j in vertices:
                if ((i, j) not in closure) and ((i, k) in closure) and ((k, j) in closure):
                    closure.add((i, j))
    return closure

##################################################

def grow_component(sources, edges, disabled=set()):
    processed = set(disabled)
    cluster = []
    queue = deque()

    def add_cluster(v):
        if v in processed:
            return
        processed.add(v)
        cluster.append(v)
        queue.append(v)

    for v0 in sources:
        add_cluster(v0)
    while queue:
        # TODO: add clusters here to ensure proper BFS
        v1 = queue.popleft()
        for v2 in edges[v1]:
            add_cluster(v2)
    return cluster

def breadth_first_search(source, edges, **kwargs):
    return grow_component([source], edges, **kwargs)

def get_ancestors(source, edges):
    return set(breadth_first_search(source, incoming_from_edges(edges))) - {source}

def get_descendants(source, edges):
    return set(breadth_first_search(source, outgoing_from_edges(edges))) - {source}

def get_connected_components(vertices, edges):
    edges = filter_orders(vertices, edges)
    undirected_edges = adjacent_from_edges(edges)
    clusters = []
    processed = set()
    for v0 in vertices:
        if v0 in processed:
            continue
        cluster = grow_component({v0}, undirected_edges, processed)
        processed.update(cluster)
        if cluster:
            clusters.append([v for v in vertices if v in cluster])
    return clusters

##################################################

SearchNode = namedtuple('Node', ['g', 'parent'])

def dijkstra(sources, edges, op=sum): # sum | max
    if not isinstance(edges, dict):
        edges = {edge: 1 for edge in edges}
    _, outgoing_edges = neighbors_from_orders(edges)
    visited = {}
    queue = []
    for v0 in sources:
        visited[v0] = SearchNode(g=0, parent=None)
        queue.append(HeapElement(visited[v0].g, v0))

    while queue:
        current_g, current_v = heappop(queue)
        if visited[current_v].g < current_g:
            continue
        for next_v in outgoing_edges[current_v]:
            next_g = op([current_g, edges[(current_v, next_v)]])
            if (next_v not in visited) or (next_g < visited[next_v].g):
                visited[next_v] = SearchNode(next_g, current_v)
                heappush(queue, HeapElement(next_g, next_v))
    return visited

##################################################

def is_hashable(value):
    #return isinstance(value, Hashable) # TODO: issue with hashable and numpy 2.7.6
    try:
        hash(value)
    except TypeError:
        return False
    return True


def hash_or_id(value):
    if is_hashable(value):
        return hash(value)
    return id(value)


def value_or_id(value):
    if is_hashable(value):
        return value
    return id(value)


def is_64bits():
    #return sys.maxsize > 2**32
    import platform
    bit, _ = platform.architecture()
    return bit == '64bit'


def inclusive_range(start, stop, step=1):
    sequence = list(np.arange(start, stop, step))
    if sequence and (sequence[-1] == stop):
        sequence.append(stop)
    return sequence


def read_pddl(this_file, pddl_filename):
    directory = os.path.dirname(os.path.abspath(this_file))
    return read(os.path.join(directory, pddl_filename))


def lowercase(*strings):
    return [string.lower() for string in strings]


def str_eq(s1, s2, ignore_case=True):
    if ignore_case:
        s1 = s1.lower()
        s2 = s2.lower()
    return s1 == s2


def clip(value, lower=-INF, upper=+INF):
    return min(max(lower, value), upper)