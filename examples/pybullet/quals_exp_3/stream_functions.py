from examples.pybullet.thought_experiments.run import get_polar_sorted_coords,get_line_angle, generate_point_from_centre_and_face
import icecream as ic
from examples.pybullet.utils.pybullet_tools.pr2_utils import get_arm_joints, ARM_NAMES, get_group_joints, get_group_conf
from examples.pybullet.utils.pybullet_tools.pr2_primitives import Pose, Conf

def get_openable_point_near_object(problem):
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
            if line[0] == real_angle:
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


def get_pickupable_point_near_object(problem):
    robot = problem.robot

    def fn(object_data, object_pose):
        final_pose = (1, 1, 0)
        return_conf = Conf(robot, get_group_joints(robot, 'base'), final_pose)
        return (return_conf,)

    return fn
