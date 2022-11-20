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
from pddlstream.utils import generate_pddl_from_init_goal,update_object_score
from collections import namedtuple
from icecream import ic
import math
import random
import pickle
import matplotlib.pyplot as plt
from shapely.geometry import Point,Polygon

import os
import time
import argparse
import pddlgym
from planning import PlanningTimeout, PlanningFailure, FD, \
    validate_strips_plan, IncrementalPlanner
from PLOI.main import _create_guider,_create_planner
from guidance import NoSearchGuidance, GNNSearchGuidance

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
        ic (object_data)
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

        ic (real_angle)
        ic (polar_sorted_coords)
        ic (lines_angle)

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

def support_pose_gen(problem,collisions=True):
    obstacles = problem.fixed if collisions else []
    def gen(body, pose,body_2,pose_2):
        ic ("in support pose gen ")
        #exit()
        # TODO: surface poses are being sampled in pr2_belief
        #p = pose.deepcopy()
        new_req_points = []
        body1_centre = pose.value[0]
        body2_centre = pose_2.value[0]
        #new_obj_centre = {'x':}
        new_req_points.append(body1_centre)
        change_centre_x = body1_centre['x'] - body2_centre['x']
        change_centre_z = body1_centre['z'] - body2_centre['z']

        for point in pose_2.value[1:]:
            new_point = point.copy()
            new_point['x'] += change_centre_x
            new_point['z'] += change_centre_z
            new_req_points.append(new_point)
            #new_req_points[-1]['x'] -= 4
            #new_req_points[-1]['z'] -= 4
            #if new_point['x'] == 0 and new_point['z'] == 0:
            #    ic ("Generating the -4 here")
            #exit()
        #p = (new_req_points, pose[1])
        #ic (p)
        #ic (new_req_points)
        p = Pose(body_2,value=new_req_points,support=pose.support)
        ic ("new generated pose" , p.value , 'for obj ', body_2)
        #ic ("Original pose", pose.value)
        yield (p,)
        #yield (pose,)

    # TODO: apply the acceleration technique here
    return gen

def pose_supported_check():
    def test (body_1, pose_1, body_2, pose_2):
        #if True :
        if body_1 != 'trophy' and body_2 == 'floor':
            return True
        else:
            return False
        #ic (p1.intersects(p2))
        #if p1.intersects(p2) > 0 :
        #    return False
        #else :
        #    return True
    return test

def supported_pose_gen(problem,collisions=True):
    obstacles = problem.fixed if collisions else []
    #def gen(body, pose,body_2,pose_2):
    #def gen(body,pose,body_2):
    def gen(pose, body_2, pose_2):
        #pose_2 = pose.copy()
        ic ("in supported pose gen")
        new_req_points = []
        body1_centre = pose.value[0]
        body2_centre = pose_2.value[0]

        obj_1 = [(pt['x'],pt['z']) for pt in pose.value[1:]]
        obj_2 = [(pt['x'],pt['z']) for pt in pose_2.value[1:]]
        #box_2_pts_list = [(pt['x'],pt['y']) for pt in box_2]
        polar_sorted_coords_1 = get_polar_sorted_coords(obj_1)
        polar_sorted_coords_2 = get_polar_sorted_coords(obj_2)
        #ic ("In trajectory pose collision checker for", obj)
        #ic (polar_sorted_coords)

        #p1 = Polygon(box_1_pts_list)
        obs_polygon_1 = Polygon(polar_sorted_coords_1)
        obs_polygon_2 = Polygon(polar_sorted_coords_2)
        x_sorted = sorted(polar_sorted_coords_2,key=lambda x:x[0])

        #width = abs(polar_sorted_coords_2[0][0] - polar_sorted_coords_2[-1][0])
        width = abs(x_sorted[0][0] -x_sorted[-1][0])
        #length = abs(z_sorted[0][1] -z_sorted[-1][1])
        length = abs(x_sorted[0][1] -x_sorted[-1][1])

        obj_1_x_sorted = sorted(polar_sorted_coords_1,key=lambda x:x[0])
        obj_1_z_sorted = sorted(polar_sorted_coords_1,key=lambda x:x[1])

        min_x = obj_1_x_sorted[0][0]
        max_x = obj_1_x_sorted[-1][0]
        min_z = obj_1_z_sorted[0][1]
        max_z = obj_1_z_sorted[-1][1]

        ic (min_x)

        new_centre = {'x':random.uniform( min_x+width/2, (max_x-width/2)) ,
                        'z': random.uniform( (min_z+width/2), (max_z-width/2))}

        ic (pose.value)
        ic (new_centre)

        #change_centre_x = body1_centre['x'] - body2_centre['x']
        #change_centre_z = body1_centre['z'] - body2_centre['z']
        change_centre_x = new_centre['x'] - body2_centre['x']
        change_centre_z = new_centre['z'] - body2_centre['z']

        new_req_points.append(body1_centre)

        for point in pose_2.value[1:]:
            new_point = point.copy()
            new_point['x'] += change_centre_x
            new_point['z'] += change_centre_z
            new_req_points.append(new_point)

        p = Pose(body_2,value=new_req_points,support=pose_2.support)
        ic ("new supported generated pose" , p.value , 'for obj ', body_2)
        #ic ("Original pose", pose.value)
        yield (p,)
    return gen

def test_obj_pose_in_region():
    def test (body_1, pose_1, body_2, pose_2):
        if pose_1.value[0] == pose_2.value[0]:
            return True
        return False
    return test


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
    pickup_id_5 = 'new_box'
    pickup_id_6 = 'crayon_1'
    pickup_id_7 = 'crayon_2'
    pickup_id_8 = 'crayon_3'
    pickup_id_9 = 'crayon_4'
    pickup_id_10 = 'crayon_5'
    pickup_id_11 = 'crayon_6'
    #open_id_2 = '6939183f-c90f-43d4-a9fa-35210461a4f5'
    open_id_2 = 'sturdy_box_small'
    region_id = 'region_obj'
    side = 1
    height = side
    width = side
    centre_x = 1
    centre_y = 0
    region_pts = [{'x':centre_x,'z':centre_y},{'x':centre_x+height/2,'z':centre_y+height/2},
                  {'x':centre_x+height/2,'z':centre_y-height/2},
                  {'x':centre_x-height/2,'z':centre_y+height/2},
                  {'x':centre_x-height/2,'z':centre_y-height/2}]
    #ic (region_pts)
    region_pose = Pose(region_id, value=region_pts, support=0)
    #ic (region_pose.value)
    #ic (type(region_pose.value))
    #exit()
    '''
    "x": 2.616,
    "y": 0.15,
    "z": -2.7125
    
    "x": 1.3741116144369094,
    "y": 0.08,
    "z": -2.5752024581623734
    
    "x": 1,
    "y": 0.15,
    "z": -1.7
    '''

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
            #('Openable', open_id),
            #('Openable', open_id_2),
            ('Region', region_pose),
            #('Pickable', region_id),
            #('Pickable', pickup_id_2),
            #('Pose',drawer,drawer_pose),
            #('AtPose',drawer,drawer_pose),
            #('Openable', movable_obj),
            Equal(('PickCost',), 1),
            Equal(('PlaceCost',), 1)]
        '''
        ('Openable', pickup_id_1),
        ('Pickable', pickup_id_1),
        ('Pickable', pickup_id_3),
        ('Pickable', pickup_id_4),
        ('Pickable', pickup_id_5),
        ('Pickable', pickup_id_6),
        ('Pickable', pickup_id_7),
        ('Pickable', pickup_id_8),
        ('Pickable', pickup_id_9),
        ('Pickable', pickup_id_10),
        ('Pickable', pickup_id_11),
        '''

        for body,required_points in problem.unity_objects:
            pose = Pose(body['id'], value=required_points, support=round(body['rotation']['y']))
            init += [('Pose', body['uuid'], pose),
                     ('AtPose', body['uuid'], pose)]
            init += [('Pickable', body['uuid'])]
            #init += [('Openable', body['uuid'])]
            trophy_new_points = []
            if body['id'] == 'trophy':
                # ic (pose.__dict__)
                for item in pose.value:
                    new_point = item.copy()
                    trophy_new_points.append(new_point)
                    trophy_new_points[-1]['x'] -= 2
                    trophy_new_points[-1]['z'] -= 2
                    trophy_new_pose = Pose('trophy', trophy_new_points, support=pose.support)

            #if body['id'] == pickup_id_4:
                #goal = ("ObjInRegion", pickup_id_4,pose, "new_box")
                #goal = ('and'
                    #,('ObjInRegion',pickup_id_4,pose,"new_box")
                    #,('ObjInRegion',pickup_id_4,pose,"trophy"))
                    #,('ObjInRegion',region_id, region_pose,"trophy")
                #    )

        #init += [('Pose', pickup_id_6 , region_pose),('AtPose', pickup_id_6, region_pose)]
        #init += [('Pose', region_id , region_pose),('AtPose', region_id, region_pose)]

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
        #goal = ("PickedUp", pickup_id_1)
        goal = ("PickedUp", "trophy")
        #goal = ['and'
        #        ,('ObjInRegion',region_id,region_pose,pickup_id_5)
        #       ,('ObjInRegion',region_id, region_pose,pickup_id_4)
        #        ]
        #goal = ('and'
        #        ,('ObjInRegion',region_pose, pickup_id_5)
        #        ,('ObjInRegion',region_pose, pickup_id_4)
        #        ,('ObjInRegion',region_pose, "trophy")
                #,('ObjInRegion',region_pose, pickup_id_6)
                #,('ObjInRegion',region_pose, pickup_id_7)
                #,('ObjInRegion',region_pose, pickup_id_8)
        #        )

        #ic(trophy_new_pose.value)
        #exit()
        #ic (second_trophy_pose.value)

    #ic (init)
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
        #'sample-pose': from_gen_fn(pose_gen(problem)),
        #'sample-pose': from_fn(pose_gen(problem)),
        #'sample-grasp': from_list_fn(get_grasp_gen(problem)),
        'sample-openable-point-near-object': from_fn(get_openable_point_near_object(problem)),
        #'sample-openable-point-near-object': from_fn(get_openable_point_near_object_learned(problem)),
        'sample-pickable-point-near-object': from_fn(get_openable_point_near_object(problem)),
        'sample-droppable-point-near-object': from_fn(get_openable_point_near_object(problem)),
        #'sample-supporting-pose':from_gen_fn(support_pose_gen(problem)),
        'sample-supported-pose':from_gen_fn(supported_pose_gen(problem)),
        'test-cfree-object-pose' : from_test(object_pose_collision_check()),
        'test-cfree-traj-pose' : from_test(trajectory_pose_collision_check()),
        #'test-support-pose' : from_test(pose_supported_check()),
        #'test-pose-in-region' : from_test(test_obj_pose_in_region()),
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

def get_relevant_objects(init_goal_pddl, file_number):
    seed = 0
    #domain_name = pddlgym_env_names[domain_name]
    domain_name = 'Unity_1'
    domain_name_test = 'Unity_1Test'
    is_strips_domain = True
    num_epochs = 100
    num_train_problems = 2
    train_planner_name = "fd-opt-lmcut"
    test_planner_name = "fd-lama-first"
    planner = _create_planner(test_planner_name)
    guider_name = "gnn-bce-10"
    problem_idx = file_number
    timeout = 120
    path = "/Users/rajesh/anaconda3/envs/35_vision_2/lib/python3.7/site-packages/pddlgym/pddl/unity_1_test/"
    f = open(path+"problem1" +str(file_number) +".pddl", "w")
    f.write(init_goal_pddl)
    f.close()
    ic ("File write done")
    #ic ("In get relevant objs 1")
    guider = _create_guider(guider_name, train_planner_name,
                            num_train_problems, is_strips_domain,
                            num_epochs, seed)

    guider.seed(seed)
    guider.train(domain_name)
    planner_to_test = IncrementalPlanner(
        is_strips_domain=is_strips_domain,
        base_planner=planner, search_guider=guider, seed=seed)
    # ic ("doing incremental planning")
    # exit()
    #else:
    #    planner_to_test = planner

    #ic ("In get relevant objs 2")
    env = pddlgym.make("PDDLEnv{}-v0".format(domain_name_test))
    action_space = env.action_space._action_predicate_to_operators
    #ic ("In get relevant objs 3")
    env.fix_problem_index(problem_idx)
    #ic ("In get relevant objs 4")
    #env.fix_problem_index(problem_idx)
    state, _ = env.reset()
    start = time.time()
    obj_scores = []
    #obj_scores = planner_to_test.get_object_scores(env.domain, state, timeout=timeout,action_space=action_space)
    sorted_action_scores = planner_to_test._guidance.score_edge(state,action_space)
    ic (sorted_action_scores)
    #assert isinstance(obj_scores, object)
    #ic (obj_scores)
    #return obj_scores
    return sorted_action_scores

def get_relevant_predicates(init, obj_scores,threshold = 0.81):

    #ic (obj_scores.keys())
    relevant_objs = {}
    for key,value in obj_scores.items():
        if value > threshold :
            relevant_objs[key] = value

    relevant_init = init[:]

    for predicate in init :
        predicate_parts = list(predicate)
        #ic (predicate_parts)
        relevant = 0
        for elem in predicate_parts[1:]:
            #ic (elem)
            if any(str(elem) in obj_key for obj_key in relevant_objs.keys()):

                relevant = 1
                break
        if relevant == 0 and len(predicate_parts) > 1:
            #ic ("removing element")
            #ic (predicate)
            relevant_init.remove(predicate)

    #ic (obj_scores)
    #ic (len(init))
    #ic (len(relevant_init))
    #ic (relevant_init)

    return relevant_init


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
        #'sample-pose': StreamInfo(PartialInputs('?r')),
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
    #_, _, _, stream_map, init, goal = pddlstream_problem
    domain_pddl, constant_map, stream_pddl, stream_map, init, goal = pddlstream_problem
    print('Init:', init)
    print('Goal:', goal)
    print('Streams:', stream_map.keys())
    args.algorithm = 'adaptive'
    #ic ("Before callig the write to init function")
    init_goal_pddl = generate_pddl_from_init_goal(init,goal)
    #obj_scores = get_relevant_objects(init_goal_pddl,1)
    sorted_action_scores = get_relevant_objects(init_goal_pddl,1)
    init_subset = []
    prev_init = []
    #ic (obj_scores)

    #ic ("After write to init function")
    threshold = 0.99999
    N = 1
    with LockRenderer():
        # solution = solve_incremental(pddlstream_problem, debug=True)
        solution = solve_focused(pddlstream_problem, stream_info=stream_info, success_cost=INF, debug=False,max_complexity=2)
    ic (solution)
    action_gnn = sorted_action_scores[0][0]
    if action_gnn == "pickup_object" or action_gnn == "move_base" or action_gnn == "drop_object":
        h = 1
        correct_actions = 0
        correct_action_locations = []
        new_action_variables = []
        while action_gnn != sorted_action_scores[h][0]  or correct_actions < 2 :
            if sorted_action_scores[h][0] == action_gnn:
                correct_actions += 1
                correct_action_locations.append(h)
            h+=1
        new_action_variables.append(sorted_action_scores[0][1])
        for h in correct_action_locations:
            new_action_variables.append(sorted_action_scores[h][1])
        new_action = pddlgym.structs.Literal(action_gnn,new_action_variables)
            #if sorted_action_scores[1][0] == action_gnn:
        #action_gnn = sorted_action_scores[h][0]
    ic (new_action.__dict__)
    exit()

    while len(init_subset) != len(init):
        #ic (len(init_subset))
        init_subset = get_relevant_predicates(init,obj_scores,threshold**N )
        #ic(init_subset)
        #while len(init_subset) <= len(prev_init) and len(init_subset) != len(init):
        while len(init_subset) <= len(prev_init) and len(init_subset) != len(init):
        # len(init_subset) <= len(prev_init) and len(init_subset) != len(init):
            N += 1
            init_subset = get_relevant_predicates(init, obj_scores, threshold - N*0.1)
            ic(len(prev_init))
            ic (len(init_subset))
            #ic (N)
            #prev_init = init_subset[:]
        ic (init_subset)
        pddlstream_problem_updated =\
            domain_pddl, constant_map, stream_pddl, stream_map, init_subset, goal
        with LockRenderer():
            #solution = solve_incremental(pddlstream_problem, debug=True)
            #solution = solve_focused(pddlstream_problem, stream_info=stream_info, success_cost=INF, debug=False)
            solution = solve_focused(pddlstream_problem_updated, stream_info=stream_info, success_cost=INF, debug=False)
            ic (solution.plan)
            if solution.plan != None :
                break
        update_object_score(obj_scores,problem)
        N +=1
        prev_init = init_subset[:]
        ic (problem.relevant_objects)
        exit()
        print (" BACK IN MAIN")

    #exit()

    '''
    with LockRenderer():
        # solution = solve_incremental(pddlstream_problem, debug=True)
        solution = solve_focused(pddlstream_problem, stream_info=stream_info, success_cost=INF, debug=False)
        #solution = solve_focused(pddlstream_problem_updated, stream_info=stream_info, success_cost=INF, debug=False)
    '''
    #exit()

    pr = cProfile.Profile()
    pr.enable()
    #exit()
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
