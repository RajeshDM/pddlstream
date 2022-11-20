#!/usr/bin/env python

from __future__ import print_function

import cProfile
import pstats
import argparse

from examples.pybullet.utils.pybullet_tools.pr2_primitives import Pose, Conf, get_ik_ir_gen, get_motion_gen, \
    get_stable_gen, get_grasp_gen, Attach, Detach, Clean, Cook, control_commands, \
    get_gripper_joints, GripperCommand, apply_commands, State
from examples.pybullet.utils.pybullet_tools.pr2_problems import cleaning_problem, cooking_problem,unity_problem, \
    thought_experiment_1
#, movement_problem
#from examples.pybullet.thought_experiments.stream_functions import get_openable_point_near_object,\
#    get_pickupable_point_near_object
from examples.pybullet.utils.pybullet_tools.pr2_utils import get_arm_joints, ARM_NAMES, get_group_joints, get_group_conf
from examples.pybullet.utils.pybullet_tools.utils import connect, get_pose, is_placement, point_from_pose, \
    disconnect, user_input, get_joint_positions, enable_gravity, save_state, restore_state, HideOutput, \
    get_distance, LockRenderer, get_min_limit, get_max_limit,dump_world,get_polar_sorted_coords
from pddlstream.algorithms.focused import solve_focused
from pddlstream.algorithms.incremental import solve_incremental
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_fn, fn_from_constant, empty_gen
from pddlstream.language.constants import Equal, AND, print_solution
from pddlstream.utils import read, INF, get_file_path, find_unique
from pddlstream.language.function import FunctionInfo
from pddlstream.language.stream import StreamInfo, PartialInputs
from pddlstream.language.object import SharedOptValue
from pddlstream.language.generator import from_gen_fn, from_test
from collections import namedtuple
from icecream import ic
import math
import random
import pickle
import matplotlib.pyplot as plt
from shapely.geometry import Point,Polygon


BASE_CONSTANT = 1
BASE_VELOCITY = 0.5

def place_movable(certified):
    for literal in certified:
        if literal[0] != 'not':
            continue
        fact = literal[1]
        if fact[0] == 'trajposecollision':
            _, b, p = fact[1:]
            p.assign()
        if fact[0] == 'trajarmcollision':
            _, a, q = fact[1:]
            q.assign()
        if fact[0] == 'trajgraspcollision':
            _, a, o, g = fact[1:]
            # TODO: finish this

# def get_base_motion_synth(problem, teleport=False):
#     # TODO: could factor the safety checks if desired (but no real point)
#     #fixed = get_fixed(robot, movable)
#     def fn(outputs, certified):
#         assert(len(outputs) == 1)
#         q0, _, q1 = find_unique(lambda f: f[0] == 'basemotion', certified)[1:]
#         place_movable(certified)
#         free_motion_fn = get_motion_gen(problem, teleport)
#         return free_motion_fn(q0, q1)
#     return fn

def move_cost_fn(c):
    [t] = c.commands
    distance = t.distance(distance_fn=lambda q1, q2: get_distance(q1[:2], q2[:2]))
    #return BASE_CONSTANT + distance / BASE_VELOCITY
    return 1

#######################################################

def extract_point2d(v):
    if isinstance(v, Conf):
        return v.values[:2]
    if isinstance(v, Pose):
        return point_from_pose(v.value)[:2]
    if isinstance(v, SharedOptValue):
        if v.stream == 'sample-pose':
            r, = v.values
            return point_from_pose(get_pose(r))[:2]
        if v.stream == 'inverse-kinematics':
            p, = v.values
            return extract_point2d(p)
        if v.stream == 'sample-point-near-object':
            ic(v)
            p, = v.values
            return extract_point2d(p)
    if isinstance(v, CustomValue):
        if v.stream == 'p-sp':
            r, = v.values
            return point_from_pose(get_pose(r))[:2]
        if v.stream == 'q-ik':
            p, = v.values
            return extract_point2d(p)
    raise ValueError(v.stream)

def opt_move_cost_fn(t):
    q1, q2 = t.values
    #distance = get_distance(extract_point2d(q1), extract_point2d(q2))
    #return BASE_CONSTANT + distance / BASE_VELOCITY
    return 1

#######################################################

CustomValue = namedtuple('OptValue', ['stream', 'values'])

def opt_pose_fn(o, r):
    p = CustomValue('p-sp', (r,))
    return p,

def opt_ik_fn(a, o, p, g):
    q = CustomValue('q-ik', (p,))
    t = CustomValue('t-ik', tuple())
    return q, t

def opt_motion_fn(q1, q2):
    t = CustomValue('t-pbm', (q1, q2))
    return t,

def opt_openable_fn(o,p):
    q = CustomValue('q-op',(p,))
    return q,

def get_2d_distance(point_1, point_2):
    if isinstance(point_1,dict):
        return math.sqrt(math.pow(abs(point_1['x'] - point_2['x'] ),2) + math.pow(abs(point_1['z']-point_2['z']),2))
    else :
        return math.sqrt(math.pow(abs(point_1[0] - point_2[0] ),2) + math.pow(abs(point_1[1]-point_2[1]),2))

def get_point_between_points(p1, p2, radius):
    distance_ratio = -0.1
    distance_new_point = 0
    while (distance_new_point < 1.6 * radius):
        x = p1[0] + distance_ratio * (p2[0] - p1[0])
        y = p1[1] + distance_ratio * (p2[1] - p1[1])
        new_pt = [x,y]
        distance_new_point= math.sqrt((new_pt[0]-p1[0])**2+(new_pt[1]-p1[1])**2)
        distance_ratio -= 0.1
    return new_pt


def get_openable_point_near_object(problem):
    robot = problem.robot

    def fn(object_data, object_pose):
        outer_points = []
        #for i, point in enumerate(object_pose[0]):
        for i, point in enumerate(object_pose.value):
            pt = (point['x'], point['z'])
            if i == 0:
                centre = pt
            else:
                outer_points.append(pt)
        # pt_1 = (pt_1['x'], )
        # polar_sorted_coords = get_polar_sorted_coords([pt_1,pt_2,pt_3,pt_4])
        polar_sorted_coords = get_polar_sorted_coords(outer_points)
        polar_sorted_coords.reverse()
        lines_angle = []

        for i in range(len(polar_sorted_coords)):
            k = (i + 1) % len(polar_sorted_coords)
            #lines_angle.append((int(get_line_angle(polar_sorted_coords[i], polar_sorted_coords[k])), (i, k)))
            lines_angle.append((int(get_line_angle_non_neg(polar_sorted_coords[i], polar_sorted_coords[k])), (i, k)))

        #obj_rotation = object_pose[1]
        obj_rotation = object_pose.support

        if obj_rotation <= 180:
            real_angle = (180 - obj_rotation - 90 ) % 360
        else:
            real_angle = (540 - obj_rotation - 90) % 360

        for i, line in enumerate(lines_angle):
            if abs(line[0] - real_angle) < 5 or abs(abs(line[0] -real_angle) - 360) < 5:
                face_location = (i + 1) % len(lines_angle)
                break

        #ic (real_angle)
        #ic (polar_sorted_coords)
        #ic (lines_angle)

        #ic(lines_angle)
        #ic (face_location)
        # ic(lines_angle[face_location][1] )
        # ic(type(polar_sorted_coords))
        pt_1 = polar_sorted_coords[lines_angle[face_location][1][0]]
        pt_2 = polar_sorted_coords[lines_angle[face_location][1][1]]
        #ic (pt_1)
        #ic (pt_2)
        #ic (centre)

        final_loc, final_real_angle = generate_point_from_centre_and_face(centre, pt_1, pt_2)
        #ic(final_real_angle)
        if final_real_angle < 0:
            final_real_angle = (final_real_angle + 360)  # %360
        #ic(final_real_angle)

        if final_real_angle >= 180:
            unity_angle = final_real_angle - 180
        else:
            unity_angle = final_real_angle + 180

        final_unity_angle = (unity_angle + 90 ) % 360

        if final_unity_angle <= 180:
            final_unity_agent_angle = 180 - final_unity_angle
        else:
            final_unity_agent_angle = 540 - final_unity_angle
        # return_conf = Conf(robot,)
        # x, z = 4.437358554501006, -2.073539754287224
        # x , z = 4.302851941683704, -1.5282154213433086
        final_pose = (final_loc[0], final_loc[1], final_unity_agent_angle)
        # final_pose = (x,z,final_unity_agent_angle)
        ic(final_pose)
        return_conf = Conf(robot, get_group_joints(robot, 'base'), final_pose)

        '''
        model_folder = 'generative_models/'
        #model_filename = 'kde_open_treasure_chest'
        model_filename = 'relative_kde_open_treasure_chest'
        with open(model_folder+model_filename, 'rb') as input:
            loaded_kde = pickle.load(input)

        final_loc = loaded_kde.resample(size=1)
        ic (final_loc)
        final_loc = (final_loc[0][0]+centre[0],final_loc[1][0]+centre[1])
        final_real_angle = get_line_angle_non_neg(centre,final_loc)
        ic (final_real_angle)
        if final_real_angle >= 180:
            unity_angle = final_real_angle - 180
        else:
            unity_angle = final_real_angle + 180

        final_unity_angle = (unity_angle + 90 ) % 360

        if final_unity_angle <= 180:
            final_unity_agent_angle = 180 - final_unity_angle
        else:
            final_unity_agent_angle = 540 - final_unity_angle

        ic (final_unity_agent_angle)
        final_pose = (final_loc[0], final_loc[1], final_unity_agent_angle)
        # final_pose = (x,z,final_unity_agent_angle)
        ic(final_pose)
        #exit()
        '''
        return (return_conf,)

    '''
    def fn (object_data, object_pose):
        new_pose = object_pose
        obj_pose = object_pose.value
        centre, pt_1, pt_2, pt_3, pt_4 = obj_pose[0]
        generated_x =  4.303715
        generated_z =  -2.438
        point_angle = obj_pose[1]
        #if obj_pose[1] >= 180 :
        if point_angle >= 180 :
            final_angle = obj_pose[1] - 180
        else :
            final_angle = obj_pose[1] + 180
        final_pose = (generated_x,generated_z, final_angle)
        return_conf =Conf(robot, get_group_joints(robot, 'base'), final_pose)
        return (return_conf,)
    '''
    return fn

def get_openable_point_near_object_learned(problem):
    robot = problem.robot

    def fn(object_data, object_pose):
        outer_points = []
        for i, point in enumerate(object_pose[0]):
            pt = (point['x'], point['z'])
            if i == 0:
                centre = pt
            else:
                outer_points.append(pt)

        model_folder = 'generative_models/'
        model_filename = 'kde_open_treasure_chest'
        with open(model_folder+model_filename, 'rb') as input:
            loaded_kde = pickle.load(input)

        final_loc = loaded_kde.resample(size=1)
        ic (final_loc)
        #ic (final_loc[0])
        #ic (final_loc[0][:2])
        final_loc = (final_loc[0][0],final_loc[1][0])
        #final_real_angle = get_line_angle_non_neg(centre,(final_loc[0][0],final_loc[1][0]))
        final_real_angle = get_line_angle_non_neg(centre,final_loc)
        ic (final_real_angle)
        if final_real_angle >= 180:
            unity_angle = final_real_angle - 180
        else:
            unity_angle = final_real_angle + 180

        final_unity_angle = (unity_angle + 90 ) % 360

        if final_unity_angle <= 180:
            final_unity_agent_angle = 180 - final_unity_angle
        else:
            final_unity_agent_angle = 540 - final_unity_angle

        final_pose = (final_loc[0], final_loc[1], final_unity_agent_angle)
        ic(final_pose)
        return_conf = Conf(robot, get_group_joints(robot, 'base'), final_pose)
        return (return_conf,)
    return fn

def get_movable_point_near_object(problem):
    robot = problem.robot

    def fn(object_data, object_pose):
        outer_points = []
        for i, point in enumerate(object_pose[0]):
            pt = (point['x'], point['z'])
            if i == 0:
                centre = pt
            else:
                outer_points.append(pt)
        # pt_1 = (pt_1['x'], )
        # polar_sorted_coords = get_polar_sorted_coords([pt_1,pt_2,pt_3,pt_4])
        polar_sorted_coords = get_polar_sorted_coords(outer_points)
        polar_sorted_coords.reverse()
        lines_angle = []

        for i in range(len(polar_sorted_coords)):
            k = (i + 1) % len(polar_sorted_coords)
            lines_angle.append((int(get_line_angle(polar_sorted_coords[i], polar_sorted_coords[k])), (i, k)))

        obj_rotation = object_pose[1]

        if obj_rotation <= 180:
            real_angle = 180 - obj_rotation
        else:
            real_angle = 540 - obj_rotation

        ic(lines_angle)
        for i, line in enumerate(lines_angle):
            if line[0] != real_angle:
                face_location = (i + 1) % len(lines_angle)
                break

                # ic(lines_angle[face_location][1] )
        # ic(type(polar_sorted_coords))
        pt_1 = polar_sorted_coords[lines_angle[face_location][1][0]]
        pt_2 = polar_sorted_coords[lines_angle[face_location][1][1]]

        final_loc, final_real_angle = generate_point_from_centre_and_face(centre, pt_1, pt_2)
        # ic(final_real_angle)
        if final_real_angle < 0:
            final_real_angle = (final_real_angle + 360)  # %360
        # ic(final_real_angle)

        if final_real_angle >= 180:
            final_unity_angle = final_real_angle - 180
        else:
            final_unity_angle = final_real_angle + 180

        if final_unity_angle <= 180:
            final_unity_agent_angle = 180 - final_unity_angle
        else:
            final_unity_agent_angle = 540 - final_unity_angle
        # return_conf = Conf(robot,)
        # x, z = 4.437358554501006, -2.073539754287224
        final_pose = (final_loc[0], final_loc[1], final_unity_agent_angle)
        # final_pose = (x,z,final_unity_agent_angle)
        ic(final_pose)
        return_conf = Conf(robot, get_group_joints(robot, 'base'), final_pose)

        return (return_conf,)

    return fn



def get_line_angle(pt_1, pt_2):
    nr = pt_2[1] - pt_1[1]
    dr = pt_2[0] - pt_1[0]
    
    if nr == 0 :
        if dr >= 0 :
            return 0
        else :
            return 180
    if dr == 0 :
        if nr >= 0 :
            return 90
        else :
            return 270

    slope = nr/dr
    #ic(nr,dr)
    #return math.degrees(math.atan(slope)),slope
    return math.degrees(math.atan2(nr,dr))

def get_line_angle_non_neg(pt_1,pt_2):
    angle = get_line_angle(pt_1,pt_2)

    if angle < 0 :
        angle += 360

    return angle

def generate_point_from_centre_and_face(centre,pt_1,pt_2):
    #ic ("centre to point 1", get_2d_distance(centre,pt_1))
    #ic ("centre to point 2", get_2d_distance(centre,pt_2))
    #ic (pt_1,pt_2)
    r_min = (get_2d_distance(centre,pt_1))
    r_max = r_min * 2
    #ic(get_2d_distance(centre,pt_2))
    #angle_1 = get_line_angle(centre,pt_1)
    #angle_2 = get_line_angle(centre,pt_2)
    #ic (angle_1,angle_2)

    #if angle_1 < 0 :
    #    angle_1 = 360 + angle_1

    #if angle_2 < 0 :
    #    angle_2 = 360 + angle_2

    angle_1 = get_line_angle_non_neg(centre,pt_1)
    angle_2 = get_line_angle_non_neg(centre,pt_2)
    if angle_1 > angle_2 :
        angle_2 += 360

    min_angle = min(angle_1,angle_2)
    max_angle = max(angle_2,angle_1)

    #ic (angle_1,angle_2)

    #angle_between_line = math.degrees(math.atan((slope_2-slope_1)/(1+slope_1*slope_2)))

    r = random.uniform(r_min*1.4, r_max)
    #ic (centre,pt_1,pt_2)
    #ic (angle_1,angle_2)
    #ic (angle_between_line)
    theta = random.uniform(angle_1,angle_2)
    unity_theta = theta % 360
    x = centre[0] + r * math.cos(math.radians(unity_theta))
    y = centre[1] + r * math.sin(math.radians(unity_theta))

    return (x,y),theta

#random_point = fn(None, chest_pose)

def generate_end_points_box(centre,width,length,height):
    pt_1 = (centre[0]-width/2,height, centre[1]-length/2)
    pt_2 = (centre[0]+width/2,height, centre[1]-length/2)
    pt_3 = (centre[0]+width/2,height, centre[1]+length/2)
    pt_4 = (centre[0]-width/2,height, centre[1]+length/2)

    return [pt_1,pt_2,pt_3,pt_4]

def object_pose_collision_check():
    def test (obj1, pose1,obj2, pose2):
    #if True :
        box_1 =  pose1.value[1:]
        box_2 = pose2.value[1:]

        box_1_pts_list = [(pt['x'],pt['y']) for pt in box_1]
        box_2_pts_list = [(pt['x'],pt['y']) for pt in box_2]

        p1 = Polygon(box_1_pts_list)
        p2 = Polygon(box_2_pts_list)

        ic (p1.intersects(p2))
        if p1.intersects(p2) > 0 :
            return False
        else :
            return True

    return test

#######################################################
def trajectory_pose_collision_check():
    def test (traj, obj, pose):
        #if True :
        box_1 =  pose.value[1:]
        #box_2 = pose2.value[1:]

        box_1_pts_list = [(pt['x'],pt['z']) for pt in box_1]
        #box_2_pts_list = [(pt['x'],pt['y']) for pt in box_2]
        polar_sorted_coords = get_polar_sorted_coords(box_1_pts_list)
        ic ("In trajectory pose collision checker for", obj)
        #ic (polar_sorted_coords)

        #p1 = Polygon(box_1_pts_list)
        obs_polygon = Polygon(polar_sorted_coords)
        #p2 = Polygon(box_2_pts_list)
        #ic (type(traj))
        #ic (traj.__dict__)
        for point in traj.commands[0].path:
            #ic (point.__dict__)
            x  = point.values[0]
            z  = point.values[1]
            #ic (x,z)

            agent_pt = Point(x,z)
            dist = agent_pt.distance(obs_polygon)
            #ic (x,z)
            if dist < 0.01 :
                ic ('cfree failing for ', obj, dist)
                #continue
                return False
        #exit()
        ic ("No collisions found for this trajectory")
        return True
        #ic (p1.intersects(p2))
        #if p1.intersects(p2) > 0 :
        #    return False
        #else :
        #    return True
    return test

def get_unity_object_pose(object_data):
    return (1,2)

def pose_gen(problem,collisions=True):

    obstacles = problem.fixed if collisions else []

    def gen(body, pose):
        # TODO: surface poses are being sampled in pr2_belief
        #p = pose.deepcopy()
        new_req_points = []
        for point in pose.value:
            new_point = point.copy()
            new_req_points.append(new_point)
            new_req_points[-1]['x'] -= 3
            new_req_points[-1]['z'] -= 3
            #if new_point['x'] == 0 and new_point['z'] == 0:
            #    ic ("Generating the -4 here")
            #exit()
        #p = (new_req_points, pose[1])
        #ic (p)
        #ic (new_req_points)
        p = Pose(body,value=new_req_points,support=pose.support)
        ic ("new generated pose" , p.value , 'for obj ', body)
        #ic ("Original pose", pose.value)
        yield (p,)
        #yield (pose,)

    # TODO: apply the acceleration technique here
    return gen

def pddlstream_from_problem(unity_agent, problem, teleport=False):
    robot = problem.robot

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    #initial_bq = Pose(robot, get_pose(robot))

    initial_bq = Conf(robot, get_group_joints(robot, 'base'), get_group_conf(robot, 'base'))

    #print (unity_agent.game_state.discovered_objects)
    #drawer = "41f05292-9206-44da-b0ba-826e6acd4eb1_drawer_top"
    open_id = "treasure_chest_large"
    open_id_te = 1
    open_id = "trophy_1"
    open_id_trophy = 2
    pickup_id_1 = 'trophy'
    pickup_id_2 = 'trophy_2'
    pickup_id_3 = 'red_cube'
    pickup_id_4 = 'brown_box'
    #open_id_2 = '6939183f-c90f-43d4-a9fa-35210461a4f5'
    open_id_2 = 'sturdy_box_small'

    init = []
    height = 0.1/2
    length = 1
    width = 1

    ic (initial_bq.__dict__)

    if unity_agent == None :
        init = [
            ('CanMove',),
            ('BConf', initial_bq),
            ('AtBConf', initial_bq),
            ('Openable', open_id_te),
            #('Pickable', pickup_id),
            #('Openable', open_id_trophy),
            Equal(('PickCost',), 1),
            Equal(('PlaceCost',), 1)
        ]
        ic ("############################")
        #ic (problem.movable)
        for body in problem.movable:
            ic (type(get_pose(body)))
            pose = Pose(body, get_pose(body))
            ic (type(pose))
            ic (pose.__dict__)
            #init += [('Graspable', body), ('Pose', body, pose),
            #         ('AtPose', body, pose)]
            centre = pose.value[0]
            #ic (centre)
            required_points = [[centre[0],centre[2], centre[1]]]
            required_points += generate_end_points_box(pose.value[0],width,length,height)
            required_dict_points = []
            for pose in required_points :
                dict_point = {}
                dict_point['x'] = pose[0]
                dict_point['y'] = pose[1]
                dict_point['z'] = pose[2]
                required_dict_points.append(dict_point)
            #ic (body,pose.to_base_conf().values,pose.value)
            #ic (pose.value[0])
            #ic (required_points)
            #ic (required_dict_points)
            rotation = 180
            pose = (required_dict_points,rotation)#, int(round(body['rotation']['y'])))
            init += [('Pose', body, pose),
                     ('AtPose', body, pose),
                     ('Occupied',pose)]

        goal  =('Open', open_id_te)

        #ic (init)
        #exit()

    elif unity_agent != None :
        ic (problem.fixed)
        #for obj in problem.fixed :
        #    ic (get_pose(obj))
        #exit()
        init = [
            ('CanMove',),
            ('BConf', initial_bq),
            ('AtBConf', initial_bq),
            ('Openable', open_id),
            ('Openable', open_id_2),
            ('Openable', pickup_id_1),
            ('Pickable', pickup_id_1),
            ('Pickable', pickup_id_3),
            ('Pickable', pickup_id_4),
            Equal(('PickCost',), 1),
            Equal(('PlaceCost',), 1)]

        for body,required_points in problem.unity_objects:
            pose = Pose(body['id'], value=required_points, support=round(body['rotation']['y']))
            init += [('Pose', body['uuid'], pose),
                     ('AtPose', body['uuid'], pose)]
            trophy_new_points = []
            if body['id'] == 'trophy':
                # ic (pose.__dict__)
                for item in pose.value:
                    new_point = item.copy()
                    trophy_new_points.append(new_point)
                    trophy_new_points[-1]['x'] -= 2
                    trophy_new_points[-1]['z'] -= 2
                    trophy_new_pose = Pose('trophy', trophy_new_points, support=pose.support)
                    # ('Pickable', body['uuid'])]
        #ic (unity_agent.game_state.discovered_objects)
        #goal  =('Open', open_id)
        #goal = ("PickedUp",pickup_id)
        #trophy_new_pose = Pose('trophy_1',second_trophy_pose.value, second_trophy_pose.support)
        #init += [('Pose', 'trophy_1', second_trophy_pose)]
        #init += [('Pose', 'trophy_2', third_trophy_pose)]
        #goal = ("AtPose",'trophy_1', second_trophy_pose)
        #init += [('Pose', 'trophy', trophy_new_pose)]
        #goal = ("AtPose",'trophy', trophy_new_pose)
        goal = ("PickedUp", pickup_id_1)

        #ic(trophy_new_pose.value)
        #exit()
        #ic (second_trophy_pose.value)

    ic (goal)
    #exit()

    #ic (init)
    #ic(problem.goal_conf)
    #if problem.goal_conf is not None:
        #goal_conf = Pose(robot, problem.goal_conf)
        #init += [('BConf', goal_conf)]
        #goal += [('AtBConf', goal_conf)]
        #goal = ('Open', open_id)
    #goal = ('Open', movable_obj)


    #goal = [AND]
    #goal += [('Open', open_id)]
    #goal += [('Open', open_id_2)]
    #goal += [('PickedUp', pickup_id)]

    #goal = ('PickedUp', pickup_id)
    '''
    goal += [('Holding', a, b) for a, b in problem.goal_holding] + \
                     [('On', b, s) for b, s in problem.goal_on] + \
                     [('Cleaned', b)  for b in problem.goal_cleaned] + \
                     [('Cooked', b)  for b in problem.goal_cooked]
    '''

    stream_map = {
        'sample-pose': from_gen_fn(pose_gen(problem)),
        #'sample-pose': from_fn(pose_gen(problem)),
        #'sample-grasp': from_list_fn(get_grasp_gen(problem)),
        'sample-openable-point-near-object': from_fn(get_openable_point_near_object(problem)),
        #'sample-openable-point-near-object': from_fn(get_openable_point_near_object_learned(problem)),
        'sample-pickable-point-near-object': from_fn(get_openable_point_near_object(problem)),
        'sample-droppable-point-near-object': from_fn(get_openable_point_near_object(problem)),
        'test-cfree-object-pose' : from_test(object_pose_collision_check()),
        'test-cfree-traj-pose' : from_test(trajectory_pose_collision_check()),
        #'test-cfree-pose' : from_test(object_pose_collision_check()),
        #'inverse-kinematics': from_gen_fn(get_ik_ir_gen(problem, teleport=teleport)),
        'plan-base-motion': from_fn(get_motion_gen(problem, teleport=teleport)),
        'MoveCost': move_cost_fn,
        'TrajPoseCollision': fn_from_constant(False),
        'TrajArmCollision': fn_from_constant(False),
        'TrajGraspCollision': fn_from_constant(False),
    }
    # get_press_gen(problem, teleport=teleport)

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal

#######################################################

# TODO: avoid copying this?

def post_process(problem, plan, teleport=False):
    if plan is None:
        return None
    commands = []
    for i, (name, args) in enumerate(plan):
        if name == 'move_base':
            ic (args)
            c = args[-1]
            new_commands = c.commands
            ic(new_commands)
        elif name == 'pick':
            a, b, p, g, _, c = args
            [t] = c.commands
            close_gripper = GripperCommand(problem.robot, a, g.grasp_width, teleport=teleport)
            attach = Attach(problem.robot, a, g, b)
            new_commands = [t, close_gripper, attach, t.reverse()]
        elif name == 'place':
            a, b, p, g, _, c = args
            [t] = c.commands
            gripper_joint = get_gripper_joints(problem.robot, a)[0]
            position = get_max_limit(problem.robot, gripper_joint)
            open_gripper = GripperCommand(problem.robot, a, position, teleport=teleport)
            detach = Detach(problem.robot, a, b)
            new_commands = [t, detach, open_gripper, t.reverse()]
        elif name == 'clean': # TODO: add text or change color?
            body, sink = args
            new_commands = [Clean(body)]
        elif name == 'cook':
            body, stove = args
            new_commands = [Cook(body)]
        elif name == 'press_clean':
            body, sink, arm, button, bq, c = args
            [t] = c.commands
            new_commands = [t, Clean(body), t.reverse()]
        elif name == 'press_cook':
            body, sink, arm, button, bq, c = args
            [t] = c.commands
            new_commands = [t, Cook(body), t.reverse()]
        else:
            raise ValueError(name)
        print(i, name, args, new_commands)
        commands += new_commands
    return commands

#######################################################

def main(unity_agent=None, display=True, teleport=False, partial=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('-simulate', action='store_true', help='Simulates the system')
    parser.add_argument('-viewer', action='store_true', help='enable the viewer while planning')
    #parser.add_argument('-display', action='store_true', help='displays the solution')
    args = parser.parse_args()

    connect(use_gui=args.viewer)
    #problem_fn = cooking_problem
    #problem_fn = unity_problem
    #problem_fn = thought_experiment_1
    problem_fn = unity_problem
    # holding_problem | stacking_problem | cleaning_problem | cooking_problem
    # cleaning_button_problem | cooking_button_problem
    ic ("In main run")
    with HideOutput():
        problem = problem_fn(unity_agent)

    #ic(problem.robot)
    #state_id = save_state()
    #saved_world = WorldSaver()
    dump_world()

    pddlstream_problem = pddlstream_from_problem(unity_agent, problem, teleport=teleport)

    stream_info = {
        'sample-pose': StreamInfo(PartialInputs('?r')),
        'inverse-kinematics': StreamInfo(PartialInputs('?p')),
        'plan-base-motion': StreamInfo(PartialInputs('?q1 ?q2'), defer=True),
        'MoveCost': FunctionInfo(opt_move_cost_fn),
        #'sample-point-near-object': StreamInfo(PartialInputs()),
    } if partial else {
        'sample-pose': StreamInfo(from_fn(opt_pose_fn)),
        'inverse-kinematics': StreamInfo(from_fn(opt_ik_fn)),
        'plan-base-motion': StreamInfo(from_fn(opt_motion_fn)),
        #'sample-point-near-object': StreamInfo(from_fn()),
        'MoveCost': FunctionInfo(opt_move_cost_fn),
        'sample-openable-point-near-object': StreamInfo(from_fn(opt_openable_fn)),
        #'sample-openable-point-near-object': from_fn(get_openable_point_near_object_learned(problem)),
        'sample-pickable-point-near-object': StreamInfo(from_fn(opt_openable_fn)),
        'sample-droppable-point-near-object': StreamInfo(from_fn(opt_openable_fn)),
    }
    _, _, _, stream_map, init, goal = pddlstream_problem
    print('Init:', init)
    print('Goal:', goal)
    print('Streams:', stream_map.keys())
    args.algorithm = 'adaptive'

    pr = cProfile.Profile()
    pr.enable()
    with LockRenderer():
        #solution = solve_incremental(pddlstream_problem, debug=True)
        solution = solve_focused(pddlstream_problem, stream_info=stream_info, success_cost=INF, debug=False)
    print (" BACK IN MAIN")
    '''
    if stream_plan is not None :
        for item in stream_plan :
            ic(type(item.__dict__['_certified'][0][1]))
            #ic(item.__dict__['_certified'][0][1].__dict__)
            break
    '''
    #print_solution(solution)
    #ic(solution)
    plan, cost, evaluations = solution
    ic(plan)
    #ic(type(plan))
    #ic(type(evaluations.all_facts[4][1].commands[0].path[0].positions))
    #ic(evaluations.all_facts[4][1].commands[0].path)
    #for action in plan :
    #    ic(action)
    pr.disable()
    #pstats.Stats(pr).sort_stats('tottime').print_stats(10)
    if plan is None:
        return
    if (not display) or (plan is None):
        disconnect()
        return
    return plan

    with LockRenderer():
        commands = post_process(problem, plan)
    
    if args.viewer:
        restore_state(state_id)
    else:
        disconnect()
        connect(use_gui=True)
        with HideOutput():
            problem_fn() # TODO: way of doing this without reloading?

    if args.simulate:
        control_commands(commands)
    else:
        apply_commands(State(), commands, time_step=0.01)
    user_input('Finish?')
    disconnect()
    # TODO: need to wrap circular joints

if __name__ == '__main__':
    main()
