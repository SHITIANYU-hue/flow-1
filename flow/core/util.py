"""A collection of utility functions for Flow."""

import csv
import errno
import os
import numpy as np
from pathlib import Path
from lxml import etree
from xml.etree import ElementTree
from wolf.utils.enums import ExtendChangeNoopAction as A


def makexml(name, nsl):
    """Create an xml file."""
    xsi = "http://www.w3.org/2001/XMLSchema-instance"
    ns = {"xsi": xsi}
    attr = {"{%s}noNamespaceSchemaLocation" % xsi: nsl}
    t = etree.Element(name, attrib=attr, nsmap=ns)
    return t


def printxml(t, fn):
    """Print information from a dict into an xml file."""
    etree.ElementTree(t).write(
        fn, pretty_print=True, encoding='UTF-8', xml_declaration=True
    )


def ensure_dir(path):
    """Ensure that the directory specified exists, and if not, create it."""
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise
    return path


def emission_to_csv(emission_path, output_path=None):
    """Convert an emission file generated by sumo into a csv file.

    Note that the emission file contains information generated by sumo, not
    flow. This means that some data, such as absolute position, is not
    immediately available from the emission file, but can be recreated.

    Parameters
    ----------
    emission_path : str
        path to the emission file that should be converted
    output_path : str
        path to the csv file that will be generated, default is the same
        directory as the emission file, with the same name
    """
    parser = etree.XMLParser(recover=True)
    tree = ElementTree.parse(emission_path, parser=parser)
    root = tree.getroot()

    # parse the xml data into a dict
    out_data = []
    for time in root.findall('timestep'):
        t = float(time.attrib['time'])

        for car in time:
            out_data.append(dict())
            try:
                out_data[-1]['time'] = t
                out_data[-1]['CO'] = float(car.attrib['CO'])
                out_data[-1]['y'] = float(car.attrib['y'])
                out_data[-1]['CO2'] = float(car.attrib['CO2'])
                out_data[-1]['electricity'] = float(car.attrib['electricity'])
                out_data[-1]['type'] = car.attrib['type']
                out_data[-1]['id'] = car.attrib['id']
                out_data[-1]['eclass'] = car.attrib['eclass']
                out_data[-1]['waiting'] = float(car.attrib['waiting'])
                out_data[-1]['NOx'] = float(car.attrib['NOx'])
                out_data[-1]['fuel'] = float(car.attrib['fuel'])
                out_data[-1]['HC'] = float(car.attrib['HC'])
                out_data[-1]['x'] = float(car.attrib['x'])
                out_data[-1]['route'] = car.attrib['route']
                out_data[-1]['relative_position'] = float(car.attrib['pos'])
                out_data[-1]['noise'] = float(car.attrib['noise'])
                out_data[-1]['angle'] = float(car.attrib['angle'])
                out_data[-1]['PMx'] = float(car.attrib['PMx'])
                out_data[-1]['speed'] = float(car.attrib['speed'])
                out_data[-1]['edge_id'] = car.attrib['lane'].rpartition('_')[0]
                out_data[-1]['lane_number'] = car.attrib['lane'].rpartition('_')[-1]
            except KeyError:
                del out_data[-1]

    # sort the elements of the dictionary by the vehicle id
    out_data = sorted(out_data, key=lambda k: k['id'])

    # default output path
    if output_path is None:
        output_path = emission_path[:-3] + 'csv'

    # output the dict data into a csv file
    keys = out_data[0].keys()
    with open(output_path, 'w') as output_file:
        dict_writer = csv.DictWriter(output_file, keys)
        dict_writer.writeheader()
        dict_writer.writerows(out_data)


def convert_lanes_to_edges(lanes):
    """
    Convert lanes (iterable) to edges (iterable).
    Remove lane index from the end and then remove duplicates while retaining order.
    Also works with single lane str.
    
    >>> lanes
    >>> ['1175109_0', '1175109_1', '1175109_2', '1183934_0', '1183934_1', '1183934_2']

    >>> convert_lanes_to_edges(lanes)
    >>> ['1175109', '1183934']
    """
    if isinstance(lanes, str):
        return lanes.rsplit('_', 1)[0]
    return list(dict.fromkeys(map(lambda x: x.rsplit('_', 1)[0], lanes)))


def update_dict_using_dict(dict1, dict2, operator):
    """
    Updates the elements of dict1 with the elements of dict2 using the given operator.
    Returns a 

    Args:
        dict1 (dict): Dictionary.
        dict2 (dict): Dictionary.
        operator (function): Can use the operator module to get basic operators.

    Returns:
        dict: Returns a new dictionary.
    """
    dict1 = dict1.copy()
    for k, v in dict2.items():
        if k in dict1:
            dict1[k] = operator(dict1[k], dict2[k])
        else:
            dict1[k] = dict2[k]
    return dict1


def update_all_dict_values(my_dict, value):
    """
    All the values of the given dictionary are updated to a single given value.

    Args:
        my_dict (dict): Given dictionary.
        value: Can be any data structure. list, dict, tuple, set, int or string.

    Returns:
        dict: Updated dict with the performed changes.
    """
    my_dict = my_dict.copy()
    for k in my_dict.keys():
        my_dict[k] = value
    return my_dict


def first(iterable):
    """
    Returns first element of iterable.
    """
    return next(iter(iterable))


def compactify_episode(transitions, intersection_id):
    """
    Generates compactified episode arrays using transitions collected over the episode.
    Note that state leads action by 1 timestep.
    This is because the first state generated using env.reset() is not captured in the transitions array passed as argument here.
    Easy fix is to ignore the first element of the action array.

    Args:
        transitions (list): List of transitions collected.
        intersection_id (str): Namely.

    Returns:
        dict: Compactified episode.
    """
    episode = {k: [t[k] for t in transitions] for k in first(transitions)}
    obs_list = list(map(lambda x: x[intersection_id]['detector_obs'], episode['observation']))
    if isinstance(obs_list[0], dict):
        obs_list = list(map(lambda x: (x[intersection_id]['detector_obs']['enter'], x[intersection_id]['detector_obs']['on'], x[intersection_id]['detector_obs']['speed']), episode['observation']))


    episode['obs'] = np.array(obs_list)
    episode['enter'] = np.array(list(map(lambda x: x[intersection_id]['detector_obs']['enter'], episode['observation']))[:-1])
    episode['on'] = np.array(list(map(lambda x: x[intersection_id]['detector_obs']['on'], episode['observation']))[:-1])
    episode['speed'] = np.array(list(map(lambda x: x[intersection_id]['detector_obs']['speed'], episode['observation']))[:-1])
    episode['all_veh_pos'] = np.array(list(map(lambda x: x[intersection_id]['detector_obs']['all_veh_pos'], episode['observation']))[:-1])
    episode['all_veh_speed'] = np.array(list(map(lambda x: x[intersection_id]['detector_obs']['all_veh_speed'], episode['observation']))[:-1])

    episode['phase'] = np.array(list(map(lambda x: x[intersection_id]['detector_obs']['phase'], episode['observation']))[:-1])
    # episode['enter_history'] = np.array(list(map(lambda x: x[intersection_id]['detector_obs']['enter_history'], episode['observation']))[:-1])
    # episode['on_history'] = np.array(list(map(lambda x: x[intersection_id]['detector_obs']['on_history'], episode['observation']))[:-1])
    # episode['speed_history'] = np.array(list(map(lambda x: x[intersection_id]['detector_obs']['speed_history'], episode['observation']))[:-1])
    episode['phase_action'] = np.array(list(map(lambda x: x[intersection_id]['detector_obs']['phase_action'], episode['observation']))[:-1])
    episode['reward'] = np.array(list(map(lambda x: x[intersection_id], episode['reward']))[1:])
    episode['action'] = np.array(list(map(lambda x: x[intersection_id], episode['action']))[1:])
    # corrected_actions, corrected_phase_actions = get_corrected_actions(episode['phase'][:, 0, 3:, :, 0], episode['action'])
    # episode['corrected_action'] = corrected_actions
    # episode['corrected_p_action'] = corrected_phase_actions
    
    del episode['observation']
    return episode


def get_wolf_root_directory():
    """
    Returns the wolf root directory.

    Returns:
        pathlib.PosixPath: wolf root directory object.
    """
    path = Path(os.getcwd())
    while (path.name != 'wolf'):
        path = path.parent
    return path


def get_flow_root_directory():
    """
    Returns the flow root directory.

    Returns:
        pathlib.PosixPath: wolf root directory object.
    """
    path = Path(__file__)
    while (path.name != 'flow'):
        path = path.parent
    return path


def save_episode_using_numpy(episode, base_fname, subfolder=None):
    """
    Saves episode using numpy.save method.

    Args:
        episode (dict): Dictionary of episode.
        base_fname (str): Base filename.
    """
    base_path = get_flow_root_directory().parent/'episodes'
    if subfolder:
        base_path = base_path/subfolder
    base_path.mkdir(exist_ok=True)
    ep_count = 0
    dest_fname = Path(base_path/f'{base_fname}{ep_count}.npy')
    while (dest_fname.exists()):
        ep_count += 1
        dest_fname = Path(base_path/f'{base_fname}{ep_count}.npy')
    np.save(dest_fname, episode)


def get_current_phase_time(phase_history):
    """
    Given phase history array of shape (1, history_length), return the phase time of current phase.

    Args:
        phase_history (numpy.ndarray): numpy array of phase history.
            Should be of the shape: (1, history_length). Eg: (1, 60).

    Returns:
        int: Phase time of current phase.
    """
    current_phase = phase_history[:, -1]
    negative_phase = 1 - current_phase
    
    reversed_phase = phase_history[:, ::-1]
    phase_time = np.argmax(reversed_phase == negative_phase)
    return current_phase, phase_time

    
def get_corrected_actions(phase_history, action):
    """
    Given multiple timesteps of phase history and actions, generate syncronized corrected actions.
    Generated current actions are syncronized with observation timesteps.
    Since observations start from t=1 while actions start from t=0, current_action is action[t + 1].
    Illegal CHANGE actions are changed to EXTEND, and vice versa for illegal EXTEND actions.
    This gives corrected_action from current_action.

    Args:
        phase_history (numpy.ndarray): numpy array of phase history over multiple timesteps.
            Should be of the shape: (num_timesteps, 1, history_length). Eg: (500, 1, 60).
        action (numpy.ndarray): numpy array of actions over multiple timesteps.
            Should of the shape: (num_timesteps,). Eg: (500,).

    Returns:
        numpy.ndarray: numpy array of corrected actions.
            Should be of the shape: (num_timesteps - 1,). Eg: (499,).
    """
    MIN_PHASE_TIME = 10
    MAX_PHASE_TIME = 60
    corrected_actions = []
    corrected_phase_actions = []

    # no corresponding action for final timestep, therefore loop stops before phase_history[-1].
    for t, ph in enumerate(phase_history[:-1]):
        current_phase, current_phase_time = get_current_phase_time(ph)
        current_action = action[t + 1]
        corrected_action = A.CHANGE if (current_phase_time >= MIN_PHASE_TIME) and (current_action == A.CHANGE) else A.EXTEND
        if current_phase_time == MAX_PHASE_TIME:
            corrected_action = A.CHANGE
        corrected_actions.append(corrected_action)

        phase_action = get_phase_action(first(current_phase), corrected_action)
        corrected_phase_actions.append(phase_action)

    corrected_actions = np.array(corrected_actions)
    corrected_phase_actions = np.array(corrected_phase_actions)
    return corrected_actions, corrected_phase_actions


def get_phase_action(current_phase, corrected_action):
    """
    Get onehot phase action from EXTEND/CHANGE type action and current phase.
    Currently, only compatible with 2 phase intersections, similar to those seen in GridEnvs.

    Args:
        current_phase (int): Current phase integer. Can be RED (0) or GREEN (1).
        corrected_action (int): Current action integer. Can be EXTEND (0) or CHANGE (1).

    Returns:
        list: Onehot phase action.
    """
    phase_to_onehot_phase = {0: [1, 0], 1: [0, 1]}

    if corrected_action == A.EXTEND:
        return phase_to_onehot_phase[current_phase]
    
    if corrected_action == A.CHANGE:
        next_phase = 1 - current_phase
        return phase_to_onehot_phase[next_phase]