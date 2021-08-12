#from examples.motion.run import *
#import examples.pybullet.pr2.run as pr2
#import examples.pybullet.thought_experiments.run as pr2
#import examples.pybullet.thought_experiments_2.run as pr2
#import examples.pybullet.quals_exp_1.run as pr2
#import examples.pybullet.quals_exp_3.run as pr2
#import examples.pybullet.quals_exp_4.run as pr2
#import examples.pybullet.exp_5.run as pr2
import examples.pybullet.exp_6_relevant_objects.run as pr2
import machine_common_sense as mcs
import yaml
import pickle
import os
import random
import argparse
from MCS_exploration import cover_floor
from MCS_exploration.gym_ai2thor.envs.mcs_env import McsEnv
from MCS_exploration.meta_controller.meta_controller import MetaController
import sys
from icecream import ic
import matplotlib.pyplot as plt
import learning_data_points as ld
from sklearn import tree

DEBUG = False

class PDDLStreamAgent:

    def __init__(self, unity_path):#, config_path, prefix,one_scene, seed=-1):
        '''
        try:
            assert "../unity_path.yaml" in os.listdir(os.getcwd())
            assert "../mcs_config.yaml" in os.listdir(os.getcwd())
        except:
            raise FileNotFoundError("You might not set up mcs config and unity path yet. Please run 'bash setup_unity.sh'.")
        '''
        level = "oracle"
        start_scene_number = 6

        self.env = McsEnv(task="interaction_scenes", scene_type='debug', start_scene_number=start_scene_number)
        self.controller = MetaController(self.env,level)
        self.env.reset()
        result = self.controller.excecute()
        return
        '''
        with open("../unity_path.yaml", 'r') as config_file:
            config = yaml.safe_load(config_file)

        self.controller = mcs.create_controller(
            os.path.join(config['unity_path'])
        )

        with open(config_path, 'r') as config_file:
            config = yaml.safe_load(config_file)
        self.level = config['metadata']
        scene_config, status = mcs.load_config_json_file(one_scene)
        goal_type = scene_config['goal']['category']

        assert self.level in ['oracle', 'level1', 'level2']

        self.step_output = self.controller.start_scene(scene_config)
        '''

    def reset(self):
        self.env.reset(repeat_current=True)
        result = self.controller.excecute()

    def execute_action_seq(self,action_seq):
        for action in action_seq :
            self.event = self.controller.step(action)

def find_actions_from_plan(plan):
    action_seq = []
    for name,args in plan :
        ic (name,args)
        if name == "move_base":
            starting_conf = args[0]
            ending_conf = args[1]
            ic (ending_conf.values)
            ending_loc = [ending_conf.values[0], ending_conf.values[1]]
            trajectory = args[2]
            action = {"action":"GotoLocation", "location":"loc|"+str(ending_loc[0]) + "|"  + "0|" + str(ending_loc[1])}
            action_seq.append(action)
            action = {"action": "RotateLook", "final_angle": ending_conf.values[2]}
            action_seq.append(action)

        if name == "open_object":
            obj_id = args[0]
            config_to_open_from = args[1]
            action = {"action":"OpenObject","objectId": str(obj_id)}
            action_seq.append(action)

        if name == "pickup_object":
            obj_id = args[0]
            config_to_open_from = args[1]
            action = {"action":"PickupObject","objectId": str(obj_id)}
            action_seq.append(action)

        if name == "drop_object":
            #obj_id = args[0]
            #config_to_open_from = args[1]
            action = {"action":"DropObject"}#,"objectId": str(obj_id)}
            action_seq.append(action)

    action = {"action": "MoveBack"}
    action_seq.append(action)
    action_seq.append(action)
    action_seq.append(action)
    #position_changes =  [args for name, args in plan]
    #actions = []
    #ic(position_changes)

    #for args in position_changes :
    #    return
        #theta = -cover_floor.get_polar_direction(args[0],args[1])  * 180/math.pi
        #n = int(abs(theta) // 10)
        #if theta > 0:
        #    action = {'action': 'RotateLeft'}
        #    for _ in range(n):
        #        actions.append(action)
        #else:
        #    action = {'action': 'RotateRight'}
        #    for _ in range(n):
        #        actions.append(action)

        #move_distance = 

    return action_seq
    

def make_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--unity-path', default='../unity_path.yaml')
    parser.add_argument('--config', default='../mcs_config.yaml')
    parser.add_argument('--prefix', default='out')
    parser.add_argument('--scenes', default='../interaction_scenes/debug/')
    return parser

def train_skill (agent , skill):
    number_data_points = 2
    count = 0
    while count < number_data_points :
        plan = pr2.main(agent.controller.sequence_generator_object.agent)
        #plan = main(None)
        ic(plan)
        if plan == None :
            continue
        action_seq = find_actions_from_plan(plan)
        agent.execute_action_seq(action_seq)
        agent.reset()
        count += 1


if __name__ == "__main__":
    #if plan == None :
    #    exit()
    #else :  
    #if True:
    args = make_parser().parse_args()
    goal_dir = args.scenes
    #all_scenes = [os.path.join(goal_dir, one_scene) for one_scene in sorted(os.listdir(goal_dir))]
    #one_scene=all_scenes[0]

    #pr2.get_relevant_objects("","")
    #exit()

    agent = PDDLStreamAgent(args.unity_path)
    count = 0
    number_data_points = 1
    data_objects = []
    #skill = "OpenObject"
    skill = "PickupObject"
    #object_id = "treasure_chest_large"
    object_id = "trophy"

    #train_skill(agent,skill)
    learnable_data = []
    state_data = []

    data_filename = "open_object_data"
    #ic ("Just before getting into loop")
    while count < number_data_points :
        ic (count)
        plan = pr2.main(agent.controller.sequence_generator_object.agent)
        #plan = main(None)
        ic(plan)
        if plan == None :
            continue
        action_seq = find_actions_from_plan(plan)
        agent.execute_action_seq(action_seq)
        #state_data += agent.controller.sequence_generator_object.agent.game_state.learnable_action_data[skill]
        state_data,return_status = agent.controller.sequence_generator_object.agent.game_state.learnable_action_data[skill][-1]
        #ic (world_state.position,return_status)
        f = open(data_filename, "a")
        ic(return_status)
        #:if return_status == "SUCCESSFUL":
        if return_status == "OUT_OF_REACH":
            f.write(str(state_data.position))
        agent.reset()
        count += 1
        f.close()

    '''
    ic(state_data)
    for world_state,return_status in state_data :
        ic (world_state.position,return_status)
        if return_status == "SUCCESSFUL":
        #if return_status == "OUT_OF_REACH":
            f.write(str(world_state.position))
    '''
    #for data_points in state_data :
    #    learnable_data.append(ld.test_network_data_point(ld.parse_initial_state(data_points[0]),data_points[1]))
    #ic (state_data)
    #ic ()
    #clf = ld.train(ld.test_network_data_for_action(object_id, action=skill, data_points=learnable_data))