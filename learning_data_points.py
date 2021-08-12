
import sklearn
from sklearn.ensemble import RandomForestClassifier
from sklearn.datasets import make_classification
from icecream import ic
from sklearn.svm import SVC
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
#from examples.pybullet.pr2.run import *
#from examples.pybullet.thought_experiments.run import *
from examples.pybullet.quals_exp_1.run import *
from sklearn import tree
from sklearn import svm
import matplotlib.pyplot as plt
import copy


from dtreeviz.trees import *


class WorldState():
    def __init__(self,obj_id,obj_list ,agent_config,success=None,action=None):
        self.main_obj_id = obj_id
        self.obj_list = obj_list[:]
        self.agent_config = agent_config
        #self.success = success
        #self.action = action
        self.action_success = {}
        if action != None :
            self.action_success[action] = success

    def update_success(self,success):
        self.success = success

class test_network_data_point ():
    def __init__(self , input_state ,success):
        self.input_state = input_state
        self.success = success

class test_network_data_for_action ():
    def __init__(self,object_id,action = None ,data_points=None):
        self.action = action
        self.object_id = object_id
        if data_points != None :
            self.data_points = data_points[:]
        else :
            self.data_points = []

    def add_data_point(self, new_data_point):
        self.data_points.append(new_data_point)

    def add_list_data_points (self,new_data_points) :
        self.data_points = self.data_points + new_data_points

def generate_data(initial_state, number_world_states, number_object_states, number_agent_config, object_id ,action):
    '''
    Function to generate random configurations along with the ground truth of that state
    '''
    new_world_data_points = []
    successful_generated_poses = []
    unsuccessful_generated_poses = []
    generated_world_states = []

    world_state = copy.deepcopy(initial_state)

    for i in range (0,number_world_states+1):
        for j in range (0, number_object_states+1) :
            successful_generated_poses = []
            unsuccessful_generated_poses = []
            if number_object_states != 0 and j == 0 :
                world_state = generate_object_config_states(initial_state, object_id, random_position=True,
                                                            random_rotation=False)
                continue
            for k in range(number_agent_config) :
                #new_world_data_points += generate_agent_config_states(world_state, object_id, ground_truth=True)
                successful_generated_poses += generate_agent_config_states(world_state, object_id, ground_truth=True)
                unsuccessful_generated_poses += generate_agent_config_states(world_state, object_id, ground_truth=False)

            for pose in successful_generated_poses:
                generated_world_states.append(
                    WorldState(world_state.main_obj_id, world_state.obj_list, pose, True, action))
            for pose in unsuccessful_generated_poses:
                generated_world_states.append(
                    WorldState(world_state.main_obj_id, world_state.obj_list, pose, False, action))
            world_state = generate_object_config_states(initial_state, object_id, random_position = True , random_rotation = False)
        #world_state = generate_world_config_states()

    return test_network_data_for_action(object_id, action=action, data_points=generated_world_states)
    #return test_network_data_for_action(object_id, action=action, data_points=new_world_data_points)

'''
def generate_data(initial_state,number_data_points,object_id,test_train, ground_truth=True):
    #Function to generate random states along with the ground truth of that state
    new_world_data_points = []
    if test_train == False :
        for i in range (number_data_points):
            #new_state = generate_agent_config_states(initial_state,object_id)
            #data_points.append(new_state)
            new_world_data_points += generate_agent_config_states(initial_state,object_id,ground_truth=True)
            new_world_data_points += generate_agent_config_states(initial_state, object_id, ground_truth=False)


    if test_train == True :
        for i in range (number_data_points):
            new_world_state = generate_object_config_states(initial_state,object_id)

            #ic(new_world_state.obj_list)
            new_world_data_points += generate_agent_config_states(new_world_state,object_id,ground_truth=True)
            new_world_data_points += generate_agent_config_states(new_world_state,object_id,ground_truth=False)
    #return data_points
    return test_network_data_for_action(object_id, action=None, data_points=new_world_data_points)
'''

def generate_agent_config_states (world_state,obj_id,ground_truth=True):
    outer_points = []

    for obj in world_state.obj_list:
        if obj_id == obj['id']:
            main_obj = obj
            break

    obj_pose = main_obj['dimensions']
    centre = (main_obj['position']['x'], main_obj['position']['z'])

    for i, point in enumerate(obj_pose):
        pt = (point['x'], point['z'])
        outer_points.append(pt)

    polar_sorted_coords = get_polar_sorted_coords(outer_points)
    polar_sorted_coords.reverse()
    #ic (polar_sorted_coords)
    lines_angle = []
    generated_poses = []

    obj_rotation = main_obj['rotation']

    if obj_rotation <= 180:
        obj_real_angle = (180 - obj_rotation - 90) % 360
    else:
        obj_real_angle = (540 - obj_rotation - 90) % 360


    face_location = -1

    for i in range(len(polar_sorted_coords)):
        k = (i + 1) % len(polar_sorted_coords)
        #ic (get_line_angle(polar_sorted_coords[i],polar_sorted_coords[k]))
        #lines_angle.append((int(get_line_angle(polar_sorted_coords[i], polar_sorted_coords[k])), (i, k)))
        lines_angle.append((int(get_line_angle_non_neg(polar_sorted_coords[i], polar_sorted_coords[k])), (i, k)))
        if int(lines_angle[-1][0]) - 3 <= int(obj_real_angle) and int(lines_angle[-1][0]) +3 >=  int(obj_real_angle):
            #ic (lines_angle[-1])
            face_location = (i + 1) % len(polar_sorted_coords)

    #ic (obj_real_angle)
    #ic (face_location)
    #ic (lines_angle)

    #for i in range(len(polar_sorted_coords)):
    for i,elem in enumerate(lines_angle):
        #k = (i + 1) % len(polar_sorted_coords)

        #pt_1 = polar_sorted_coords[i]
        #pt_2 = polar_sorted_coords[k]
        pt_1= polar_sorted_coords[lines_angle[i][1][0]]
        pt_2= polar_sorted_coords[lines_angle[i][1][1]]

        #ic (centre,pt_1,pt_2)

        if ground_truth == True :
            #if curr_face_ground_truth == True :
            if face_location == i :
                #ic (centre, pt_1,pt_2)
                #ic(pt_1, pt_2)
                generated_loc, generated_real_angle = generate_point_from_centre_and_face(centre, pt_1, pt_2)
            else :
                continue
        else :
            #if curr_face_ground_truth == False :
            if face_location != i :
                #ic(pt_1, pt_2)
                generated_loc, generated_real_angle = generate_point_from_centre_and_face(centre, pt_1, pt_2)
            else:
                continue

        #ic(generated_loc, generated_real_angle)
        final_real_angle = generated_real_angle
        if final_real_angle < 0:
            final_real_angle = (final_real_angle + 360)  # %360
        # ic(final_real_angle)

        if final_real_angle >= 180:
            unity_angle = final_real_angle - 180
        else:
            unity_angle = final_real_angle + 180

        final_unity_angle = (unity_angle + 90 ) % 360

        if final_unity_angle <= 180:
            final_unity_agent_angle = 180 - final_unity_angle
        else:
            final_unity_agent_angle = 540 - final_unity_angle
        '''
        if generated_real_angle < 0 :
            generated_unity_angle = 360 + generated_real_angle
        else :
            generated_unity_angle = generated_real_angle

        if generated_unity_angle <= 180:
            unity_agent_angle = 180 - generated_unity_angle
        else:
            unity_agent_angle = 540 - generated_unity_angle

        if unity_agent_angle>= 180:
            final_agent_unity_angle = unity_agent_angle - 180
        else:
            final_agent_unity_angle = unity_agent_angle + 180
        if generated_real_angle >= 180:
            generated_unity_angle = generated_real_angle - 180
        else:
            generated_unity_angle = generated_real_angle + 180
        #generated_unity_angle = generated_real_angle
        '''
        #if face_location == i :
            #ic (generated_real_angle,generated_unity_angle,final_unity_agent_angle)
        #    ic(generated_real_angle, generated_unity_angle,unity_agent_angle, final_agent_unity_angle)
        #'''
        #final_unity_agent_angle = generated_unity_angle
        #generated_poses.append((generated_loc[0], generated_loc[1],generated_real_angle))
        generated_poses.append((generated_loc[0], generated_loc[1], final_unity_agent_angle))

    return generated_poses


def generate_object_config_states(world_state, obj_id, random_position = False, random_rotation=False):
    new_initial_world_state = copy.deepcopy(world_state)
    #ic (new_initial_world_state.__dict__)

    for i,obj in enumerate(world_state.obj_list):
        if obj_id == obj['id']:
            if random_rotation == True:
                rotation_amount = 90 #random.randint(1,360)
                new_initial_world_state.obj_list[i]['rotation'] = (new_initial_world_state.obj_list[i][
                                                                    'rotation'] -rotation_amount) % 360
                centre = new_initial_world_state.obj_list[i]['position']
                for j,item in enumerate(new_initial_world_state.obj_list[i]['dimensions']):
                    item = copy.deepcopy(item)
                    new_initial_world_state.obj_list[i]['dimensions'][j]['x'] = ( item['x'] - centre['x'] ) * math.cos(math.radians(rotation_amount)) - \
                                                                                (item['z'] - centre['z'] )*math.sin(math.radians(rotation_amount)) + centre['x']
                    new_initial_world_state.obj_list[i]['dimensions'][j]['z'] = ( item['x'] - centre['x'] ) * math.sin(math.radians(rotation_amount)) + \
                                                                                (item['z'] - centre['z'] )*math.cos(math.radians(rotation_amount)) + centre['z']
                    #ic (item)
                #ic (new_initial_world_state.obj_list[i]['dimensions'])
            if random_position == True:
                x_movement = random.uniform(0,3)
                z_movement = random.uniform(0,3)

                new_initial_world_state.obj_list[i]['position']['x'] += x_movement
                new_initial_world_state.obj_list[i]['position']['z'] += z_movement
                for j, item in enumerate(new_initial_world_state.obj_list[i]['dimensions']):
                    item = copy.deepcopy(item)
                    new_initial_world_state.obj_list[i]['dimensions'][j]['x'] += x_movement
                    new_initial_world_state.obj_list[i]['dimensions'][j]['z'] += z_movement
            break

    #ic (new_initial_world_state.__dict__)

    return new_initial_world_state

def generate_world_config_states():
    return

def transform_x(x):
    learnable_x = []
    for obj in x.obj_list :
        if obj['id'] == x.main_obj_id :
            main_obj = obj
    agent_distance_from_object = math.sqrt(math.pow(main_obj['position']['x'] - x.agent_config[0],2) +
                                           math.pow(main_obj['position']['z'] - x.agent_config[1],2))
    #ic (x.__dict__)
    #ic (main_obj)
    learnable_x.append(agent_distance_from_object)
    if len(x.agent_config) > 2 :
        relative_rotation = abs(main_obj['rotation']-x.agent_config[2])
        learnable_x.append(relative_rotation)
    return learnable_x

def transform_relative_x_y(x):
    learnable_x = []
    for obj in x.obj_list :
        if obj['id'] == x.main_obj_id :
            main_obj = obj
    #agent_distance_from_object = math.sqrt(math.pow(main_obj['position']['x'] - x.agent_config[0],2) +
    #                                       math.pow(main_obj['position']['z'] - x.agent_config[1],2))
    #agent_x_dist = main_obj['position']['x'] - x.agent_config[0]
    #agent_z_dist = main_obj['position']['z'] - x.agent_config[1]
    agent_x_dist = -(main_obj['position']['x'] - x.agent_config[0])
    agent_z_dist = -(main_obj['position']['z'] - x.agent_config[1])
    learnable_x.append(agent_x_dist)
    learnable_x.append(agent_z_dist)
    if len(x.agent_config) > 2 :
        relative_rotation = abs(main_obj['rotation']-x.agent_config[2])
        learnable_x.append(relative_rotation)
    return learnable_x

def transform_input_states(x,transform_fn=transform_x):
    if isinstance(x,list):
        transformed_x = []
        for input_state in x:
            transformed_x.append(transform_fn(input_state))
        return transformed_x
    else :
        return transform_x(x)

def train(data_object,action):
    x = []
    y = []
    for item in data_object.data_points :
        x.append(item)
        y.append(item.action_success[action])

    X = transform_input_states(x)

    #clf = RandomForestClassifier(max_depth=10, random_state=0)
    #clf = tree.DecisionTreeClassifier(max_depth =7, random_state= 0)
    clf = svm.SVC()
    clf.fit(X, y)
    #ic (sum(y))
    pred = clf.predict(X)
    pred = np.array(pred)
    y = np.array(y)
    for i,elem in enumerate(pred) :
        X[i].append((y[i],elem))
    #ic (X)
    ic ("Training Accuracy")
    get_accuracy(y,pred)
    return clf

def test(data_object, action, clf):
    x = []
    y = []
    for item in data_object.data_points :
        x.append(item)
        y.append(item.action_success[action])
    X = transform_input_states(x)
    #ic (X)
    pred = clf.predict(X)
    pred = np.array(pred)
    y = np.array(y)
    for i,elem in enumerate(pred) :
        X[i].append((y[i],elem))
    #ic (X)
    get_accuracy(y,pred)

def get_accuracy(y, pred):

    #for i,elem in enumerate(pred) :
    #    X[i].append((y[i],elem))
    #ic (X)
    ic (sum(y))
    ic (sum(pred))
    ic (y.shape)
    position_correct = np.equal(pred, y)
    ic ("Accuracy : ", (np.sum(position_correct)/y.shape[0])*100)

def visualize_tree (clf):
    #tree.plot_tree(clf);
    #plt.show()
    tree.plot_tree(clf,
                   feature_names=["distance", "relative_rot"],
                   class_names=["False", "True"],
                   rounded=True,
                   filled=True);
    #plt.show()
    plt.savefig("decision_tree.png")

def train_test (initial_state,object_id,action="OpenObject"):
    #initial_state = []
    #train_set = generate_data(initial_state, 100,object_id,False)
    train_set = generate_data(initial_state,0,0,1000,object_id,action)
    clf = train(train_set,action)

    ic ("Training Done Testing starting")
    #exit()

    test_set = generate_data(initial_state,0,1,150,object_id,action)
    #test_set = generate_data(initial_state, 10,object_id,False)
    #test(test_set,clf)
    #test_set = generate_data(initial_state, 10,object_id,True)
    test(test_set,action,clf)

    #visualize_tree(clf)


def parse_initial_state(initial_state,agent_pos, agent_rotation,object_id):
    parsed_state = []
    required_obj_data = []

    for body in initial_state:#initial_game_state.discovered_objects:
        #pose = Pose(body,get_unity_object_pose(body))

        #ic(type(body))

        #required_points = [body['position']]
        required_points = []
        for i in range(0,8):
            point = {'x':body['dimensions'][i]['x'],'z':body['dimensions'][i]['z']}
            if point not in required_points:
                required_points.append(point)
            #if body['dimensions'][i] not in required_points:
            #    required_points.append(body['dimensions'][i])
        #ic (required_points)
        pose = (required_points, int(round(body['rotation']['y'])))
        required_obj_data.append({'id':body['id'], 'rotation': body['rotation']['y'], 'position': body['position'],'dimensions':required_points,
                              'distance':body['distance_in_world']})
    parsed_state = WorldState(object_id, required_obj_data, (agent_pos,agent_rotation))
    # (initial_game_state.position,initial_game_state.rotation))
    #ic(parsed_state.obj_list)
    return parsed_state

def main(initial_state,object_id):
    agent_pos = None
    agent_rotation = None
    parsed_state = parse_initial_state(initial_state,agent_pos,agent_rotation,object_id)
    train_test(parsed_state,object_id)


initial_state = [{'color': {'b': 65, 'g': 219, 'r': 142},
                  'dimensions': [{'x': -0.99, 'y': 7.422327e-07, 'z': 0.6192894},
                                 {'x': -0.919289351, 'y': -2.11676081e-08, 'z': 0.690000057},
                                 {'x': -0.99, 'y': -3.84604846e-07, 'z': 0.7607107},
                                 {'x': -1.06071067, 'y': 3.78795477e-07, 'z': 0.690000057},
                                 {'x': -0.9899995, 'y': 0.200000733, 'z': 0.619290948},
                                 {'x': -0.919288754, 'y': 0.199999958, 'z': 0.6900016},
                                 {'x': -0.9899994, 'y': 0.1999996, 'z': 0.7607123},
                                 {'x': -1.06071007, 'y': 0.200000376, 'z': 0.690001667}],
                  'direction': {'x': -0.183038682, 'y': -0.221171856, 'z': 0.9579039},
                  'distance': 15.9840548,
                  'distance_in_steps': 15.9840548,
                  'distance_in_world': 1.663973,
                  'explored': 0,
                  'held': False,
                  'id': '6939183f-c90f-43d4-a9fa-35210461a4f5',
                  'locationParent': None,
                  'mass': 1.33,
                  'material_list': ['WOOD'],
                  'openable': None,
                  'position': {'x': -0.9899997, 'y': 0.100000173, 'z': 0.690000832},
                  'rotation': {'x': -0.000208234225, 'y': 135.000031, 'z': 0.0004373962},
                  'shape': 'blank block cube',
                  'texture_color_list': ['red'],
                  'uuid': '6939183f-c90f-43d4-a9fa-35210461a4f5',
                  'visible': False},
                 {'color': {'b': 115, 'g': 43, 'r': 36},
                  'dimensions': [{'x': -4.29336929, 'y': -0.00174984778, 'z': -1.7299608},
                                 {'x': -4.29336929, 'y': -0.001749763, 'z': -1.92246079},
                                 {'x': -4.00286961, 'y': -0.00174989551, 'z': -1.92246091},
                                 {'x': -4.00286961, 'y': -0.00174998026, 'z': -1.72996092},
                                 {'x': -4.29336929, 'y': 0.145250142, 'z': -1.7299608},
                                 {'x': -4.29336929, 'y': 0.145250231, 'z': -1.92246079},
                                 {'x': -4.00286961, 'y': 0.145250112, 'z': -1.92246079},
                                 {'x': -4.002869, 'y': 0.145250022, 'z': -1.7299608}],
                  'direction': {'x': -0.9605302, 'y': -0.0911088, 'z': -0.262832344},
                  'distance': 35.8523417,
                  'distance_in_steps': 35.8523417,
                  'distance_in_world': 3.61494279,
                  'explored': 0,
                  'held': False,
                  'id': 'sturdy_box_small',
                  'locationParent': None,
                  'mass': 14.999999,
                  'material_list': ['PLASTIC'],
                  'openable': None,
                  'position': {'x': -4.14826, 'y': 0.134475023, 'z': -1.82629251},
                  'rotation': {'x': -2.52263417e-05, 'y': 180.0, 'z': 2.61180558e-05},
                  'shape': 'chest',
                  'texture_color_list': ['green'],
                  'uuid': 'sturdy_box_small',
                  'visible': False},
                 {'color': {'b': 196, 'g': 214, 'r': 187},
                  'dimensions': [{'x': 4.01685238, 'y': -4.00485973e-08, 'z': -1.73721552},
                                 {'x': 4.588852, 'y': -2.54441019e-08, 'z': -1.73721552},
                                 {'x': 4.588852, 'y': -1.95560474e-08, 'z': -1.31921554},
                                 {'x': 4.01685238, 'y': -3.41605428e-08, 'z': -1.31921554},
                                 {'x': 4.01685238, 'y': 0.461999953, 'z': -1.73721552},
                                 {'x': 4.588852, 'y': 0.461999953, 'z': -1.73721552},
                                 {'x': 4.588852, 'y': 0.461999953, 'z': -1.31921554},
                                 {'x': 4.01685238, 'y': 0.461999953, 'z': -1.31921554}],
                  'direction': {'x': 0.9898617, 'y': -0.06310243, 'z': -0.1272472},
                  'distance': 50.3475475,
                  'distance_in_steps': 50.3475475,
                  'distance_in_world': 5.055953,
                  'explored': 0,
                  'held': False,
                  'id': 'treasure_chest_large',
                  'locationParent': None,
                  'mass': 14.999999,
                  'material_list': ['WOOD'],
                  'openable': None,
                  'position': {'x': 4.302852, 'y': 0.144211963, 'z': -1.52183354},
                  #'rotation': {'x': 8.07083e-07, 'y': 180.0, 'z': -1.46289494e-06},
                  'rotation': {'x': 8.07083e-07, 'y': 180, 'z': -1.46289494e-06},
                  'shape': 'chest',
                  'texture_color_list': ['black'],
                  'uuid': 'treasure_chest_large',
                  'visible': False},
                 {'color': {'b': 87, 'g': 120, 'r': 129},
                  'dimensions': [{'x': 1.27911162, 'y': 2.01165676e-07, 'z': -2.58020234},
                                 {'x': 1.46911168, 'y': 2.16066837e-07, 'z': -2.58020234},
                                 {'x': 1.46911168, 'y': 0.140000224, 'z': -2.58020258},
                                 {'x': 1.27911162, 'y': 0.140000224, 'z': -2.58020258},
                                 {'x': 1.27911162, 'y': -6.70552254e-08, 'z': -2.88020253},
                                 {'x': 1.46911168, 'y': -6.70552254e-08, 'z': -2.88020253},
                                 {'x': 1.46911168, 'y': 0.139999956, 'z': -2.88020253},
                                 {'x': 1.27911162, 'y': 0.139999956, 'z': -2.88020253}],
                  'direction': {'x': 0.7418141, 'y': -0.141119242, 'z': -0.6555892},
                  'distance': 26.710052499999996,
                  'distance_in_steps': 26.710052499999996,
                  'distance_in_world': 2.69968987,
                  'explored': 0,
                  'held': False,
                  'id': 'trophy',
                  'locationParent': None,
                  'mass': 0.5,
                  'material_list': ['METAL'],
                  'openable': None,
                  'position': {'x': 1.37323332, 'y': 0.0700001344, 'z': -2.70341325},
                  'rotation': {'x': 90.0, 'y': 180.0, 'z': 0.0},
                  'shape': 'trophy',
                  'texture_color_list': ['silver'],
                  'uuid': 'trophy',
                  'visible': False}]

initial_state_2 = [
                    {'color': {'b': 240, 'g': 7, 'r': 141},
                    'dimensions': [{'x': -0.141421378, 'y': -1.59814954e-05, 'z': 1.2},
                                   {'x': 2.23517418e-08, 'y': -1.59814954e-05, 'z': 1.05857873},
                                   {'x': 0.141421378, 'y': -1.59814954e-05, 'z': 1.2},
                                   {'x': -2.23517418e-08, 'y': -1.59814954e-05, 'z': 1.34142137},
                                   {'x': -0.141421378, 'y': 0.199984014, 'z': 1.2},
                                   {'x': 2.23517418e-08, 'y': 0.199984014, 'z': 1.05857873},
                                   {'x': 0.141421378, 'y': 0.199984014, 'z': 1.2},
                                   {'x': -2.23517418e-08, 'y': 0.199984014, 'z': 1.34142137}],
                    'direction': {'x': 0.0, 'y': -0.289188683, 'z': 0.9572721},
                    'distance': 11.999999999999998,
                    'distance_in_steps': 11.999999999999998,
                    'distance_in_world': 1.25356209,
                    'held': False,
                    'mass': 1.5,
                    'material_list': ['METAL'],
                    'position': {'x': 0.0, 'y': 0.09998402, 'z': 1.2},
                    'rotation': {'x': 0.0, 'y': 225.0, 'z': 0.0},
                    'shape': 'ball',
                    'texture_color_list': ['brown'],
                    'uuid': '79bf6ea3-e27c-49a8-9a85-f353f1a6bf45',
                    'visible': True},
                    {'color': {'b': 113, 'g': 125, 'r': 173},
                    'dimensions': [{'x': -0.120208085, 'y': 0.0, 'z': 2.25079179},
                                   {'x': -0.470861316, 'y': 0.0, 'z': 2.454191},
                                   {'x': -0.8797919, 'y': 0.0, 'z': 1.74920809},
                                   {'x': -0.5291387, 'y': 0.0, 'z': 1.545809},
                                   {'x': -0.120208085, 'y': 0.44, 'z': 2.25079179},
                                   {'x': -0.470861316, 'y': 0.44, 'z': 2.454191},
                                   {'x': -0.8797919, 'y': 0.44, 'z': 1.74920809},
                                   {'x': -0.5291387, 'y': 0.44, 'z': 1.545809}],
                    'direction': {'x': -0.241075933, 'y': -0.117245331, 'z': 0.963398159},
                    'distance': 20.6155276,
                    'distance_in_steps': 20.6155276,
                    'distance_in_world': 2.11279583,
                    'held': False,
                    'mass': 4.5,
                    'material_list': ['METAL'],
                    'position': {'x': -0.500693142, 'y': 0.21899195, 'z': 2.00089169},
                    'rotation': {'x': 0.0, 'y': 30.1161957, 'z': 0.0},
                    'shape': 'table',
                    'texture_color_list': ['grey', 'white'],
                    'uuid': 'e6448e69-11e8-47e4-bd2c-61972100a3ed',
                    'visible': True},
                    {'color': {'b': 30, 'g': 247, 'r': 187},
                    'dimensions': [{'x': 0.73025, 'y': -0.003248505, 'z': 0.178750053},
                                   {'x': 0.730249941, 'y': -0.00324751972, 'z': -0.17875},
                                   {'x': 1.26975, 'y': -0.003248866, 'z': -0.178750113},
                                   {'x': 1.26975, 'y': -0.00324985129, 'z': 0.178749934},
                                   {'x': 0.7302507, 'y': 0.2697515, 'z': 0.1787508},
                                   {'x': 0.7302506, 'y': 0.2697525, 'z': -0.178749248},
                                   {'x': 1.2697506, 'y': 0.269751132, 'z': -0.178749368},
                                   {'x': 1.26975071, 'y': 0.269750118, 'z': 0.178750679}],
                    'direction': {'x': 0.9780962, 'y': -0.208153576, 'z': -0.000147520463},
                    'distance': 10.0,
                    'distance_in_steps': 10.0,
                    'distance_in_world': 1.10177362,
                    'held': False,
                    'mass': 14.999999,
                    'material_list': ['METAL'],
                    'position': {'x': 0.9997398, 'y': 0.249740332, 'z': -0.00015078485},
                    'rotation': {'x': -0.000157934788, 'y': 180.0, 'z': 0.000142974532},
                    'shape': 'chest',
                    'texture_color_list': ['grey'],
                    'uuid': 'sturdy_box_large',
                    'visible': False},
                    {'color': {'b': 93, 'g': 142, 'r': 43},
                    'dimensions': [{'x': 1.262506, 'y': 0.0006248194, 'z': 2.94374871},
                                   {'x': 0.7375059, 'y': 0.0006247894, 'z': 2.943758},
                                   {'x': 0.737490058, 'y': 0.000624806853, 'z': 2.056258},
                                   {'x': 1.26249015, 'y': 0.0006248369, 'z': 2.05624866},
                                   {'x': 1.262506, 'y': 0.238124818, 'z': 2.94374871},
                                   {'x': 0.7375059, 'y': 0.238124788, 'z': 2.943758},
                                   {'x': 0.737490058, 'y': 0.238124818, 'z': 2.056258},
                                   {'x': 1.26249015, 'y': 0.238124847, 'z': 2.05624866}],
                    'direction': {'x': 0.276442558, 'y': -0.1225909, 'z': 0.9531794},
                    'distance': 26.9258475,
                    'distance_in_steps': 26.9258475,
                    'distance_in_world': 2.73191166,
                    'held': False,
                    'mass': 5.0,
                    'material_list': ['METAL'],
                    'position': {'x': 0.73132056, 'y': 0.1381894, 'z': 2.52160788},
                    'rotation': {'x': 3.27999237e-06, 'y': 270.001038, 'z': -1.12923954e-06},
                    'shape': 'case',
                    'texture_color_list': ['white'],
                    'uuid': 'suitcase_med',
                    'visible': True},
                    {'color': {'b': 196, 'g': 214, 'r': 187},
                    'dimensions': [{'x': 1.20900035, 'y': 9.883767e-07, 'z': 0.714},
                                   {'x': 1.20899928, 'y': 1.033838e-06, 'z': 1.286},
                                   {'x': 0.7909993, 'y': 8.59367333e-07, 'z': 1.28599918},
                                   {'x': 0.7910003, 'y': 8.1390607e-07, 'z': 0.7139992},
                                   {'x': 1.20900011, 'y': 0.462000966, 'z': 0.7139999},
                                   {'x': 1.20899916, 'y': 0.462001026, 'z': 1.28599989},
                                   {'x': 0.7909991, 'y': 0.462000847, 'z': 1.28599918},
                                   {'x': 0.7910001, 'y': 0.4620008, 'z': 0.7139992}],
                    'direction': {'x': 0.687533, 'y': -0.2202385, 'z': 0.69194895},
                    'distance': 14.1421318,
                    'distance_in_steps': 14.1421318,
                    'distance_in_world': 1.48791945,
                    'held': False,
                    'mass': 14.999999,
                    'material_list': ['WOOD'],
                    'position': {'x': 0.9936177, 'y': 0.144212916, 'z': 0.9999996},
                    'rotation': {'x': -2.39149049e-05, 'y': 89.9999, 'z': -4.55373765e-06},
                    'shape': 'chest',
                    'texture_color_list': ['black'],
                    'uuid': 'treasure_chest_large',
                    'visible': False},
                    {'color': {'b': 229, 'g': 139, 'r': 47},
                    'dimensions': [{'x': 1.405, 'y': 0.000164665282, 'z': -1.355},
                                   {'x': 1.595, 'y': 0.000164665282, 'z': -1.355},
                                   {'x': 1.595, 'y': 0.140164673, 'z': -1.355},
                                   {'x': 1.405, 'y': 0.140164673, 'z': -1.355},
                                   {'x': 1.405, 'y': 0.000164501369, 'z': -1.655},
                                   {'x': 1.595, 'y': 0.000164501369, 'z': -1.65500009},
                                   {'x': 1.595, 'y': 0.140164524, 'z': -1.65500009},
                                   {'x': 1.405, 'y': 0.140164524, 'z': -1.65500009}],
                    'direction': {'x': 0.700004637, 'y': -0.183198333, 'z': -0.6902405},
                    'distance': 20.180437599999998,
                    'distance_in_steps': 20.180437599999998,
                    'distance_in_world': 2.05582762,
                    'held': False,
                    'mass': 0.5,
                    'material_list': ['METAL'],
                    'position': {'x': 1.49912167, 'y': 0.07016463, 'z': -1.47821093},
                    'rotation': {'x': 90.0, 'y': 180.0, 'z': 0.0},
                    'shape': 'trophy',
                    'texture_color_list': ['silver'],
                    'uuid': 'trophy_6',
                    'visible': False}
]


object_id = "treasure_chest_large"

def generate_and_save_data(obj_id, initial_state,action,plot=False):
    generated_poses = []
    #obj_id = 'trophy'
    #obj_id = '6939183f-c90f-43d4-a9fa-35210461a4f5'
    #obj_id = 'sturdy_box_small'
    #obj_id = 'treasure_chest_large'

    parsed_state = parse_initial_state(initial_state, None, None, obj_id)  # ,ld.object_id)

    for i in range(10000):
    #for i in range(10):
        #ic (i)
        generated_pose = generate_agent_config_states(parsed_state,
                                                         obj_id, ground_truth=True)
        generated_poses.append(np.array(list(generated_pose[0])))

    # np.save('data/2d_unity_poses.npy', generated_poses)
    #generated_poses = np.array(generated_poses)
    generated_world_states = []
    world_state = copy.deepcopy(parsed_state)
    for pose in generated_poses:
        #ic (pose)
        generated_world_states.append(
            WorldState(world_state.main_obj_id, world_state.obj_list, pose, True, action))
    #data_obj= test_network_data_for_action(object_id, action=action, data_points=generated_world_states)
    #ic (type(data_obj))
    #ic (data_obj.__dict__['data_points'][0].__dict__)
    relative_poses = transform_input_states(generated_world_states,transform_relative_x_y)
    # print (len(generated_poses))
    # print (len(generated_poses[0]))
    #df = pd.DataFrame({'x': generated_poses[:, 0],
    #                   'y': generated_poses[:, 1],
    #                   'class': np.ones(generated_poses.shape[0])})
    # df.to_csv('data/2d_unity_poses.csv')
    data_folder = './GANs_for_Credit_Card_Data/data/'
    #file_name = '2d_unity_poses_with_rotation_'
    file_name = 'relative_unity_poses_np_'
    #file_name = '2d_unity_poses'
    #ic (relative_poses)
    #np.save(data_folder + 'relative_unity_poses_np_' + str(obj_id) + '.npy', relative_poses)
    np.save(data_folder + file_name + str(obj_id), relative_poses)
    #np.save(data_folder + file_name + str(obj_id), generated_poses)

    #generated_poses = np.concatenate(generated_poses,0)
    #ic (len(generated_poses[:,1]),generated_poses[0] )
    if plot == True:
        generated_poses = np.array(generated_poses)
        #plt.scatter(generated_poses[:,0],generated_poses[:,1], alpha=0.25)
        #plt.scatter(*generated_poses.T)
        plt.plot(generated_poses[:,0],generated_poses[:,1] )
        plt.title('Real data')
        plt.ylim(-4, 2)
        plt.xlim(-5, 6)
        #plt.colorbar()
        plt.show()

    # np.savetxt('data/2d_unity_poses_np.csv',generated_poses,delimiter=',')

if __name__ == "__main__":
    #main(initial_state,object_id)
    #obj_ids = ['treasure_chest_large']
    plot = False
    obj_ids_1 = ['treasure_chest_large', 'trophy', '6939183f-c90f-43d4-a9fa-35210461a4f5', 'sturdy_box_small']
    obj_ids_2 = ['treasure_chest_large','trophy_6', 'sturdy_box_large', 'suitcase_med', '79bf6ea3-e27c-49a8-9a85-f353f1a6bf45', 'e6448e69-11e8-47e4-bd2c-61972100a3ed' ]
    for obj_id in obj_ids_1 :
        ic (obj_id)
        generate_and_save_data(obj_id, initial_state,"OpenObject",plot)