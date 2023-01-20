"""Objects that define the various meta-parameters of an experiment."""

import logging
import collections

from flow.utils.flow_warnings import deprecated_attribute
from flow.controllers.car_following_models import SimCarFollowingController
from flow.controllers.rlcontroller import RLController
from flow.controllers.lane_change_controllers import SimLaneChangeController
from flow.config import WOLF_SEED


SPEED_MODES = {
    "aggressive": 0,
    "obey_safe_speed": 1,
    "no_collide": 7,
    "right_of_way": 25,
    "all_checks": 31
}

LC_MODES = {
    "no_lc_safe": 512,
    "no_lc_aggressive": 0,
    "sumo_default": 1621,
    "no_strategic_aggressive": 1108,
    "no_strategic_safe": 1620,
    "only_strategic_aggressive": 1,
    "only_strategic_safe": 513,
    "no_cooperative_aggressive": 1105,
    "no_cooperative_safe": 1617,
    "only_cooperative_aggressive": 4,
    "only_cooperative_safe": 516,
    "no_speed_gain_aggressive": 1093,
    "no_speed_gain_safe": 1605,
    "only_speed_gain_aggressive": 16,
    "only_speed_gain_safe": 528,
    "no_right_drive_aggressive": 1045,
    "no_right_drive_safe": 1557,
    "only_right_drive_aggressive": 64,
    "only_right_drive_safe": 576
}

# Traffic light defaults
PROGRAM_ID = 1
MAX_GAP = 3.0
DETECTOR_GAP = 0.6
SHOW_DETECTORS = True


class TrafficLightParams:
    """Base traffic light class

    This object (and its derived classes) hold properties necessary
    for adding traffic signals to a network.
    """
    def __init__(self, baseline=False):
        """Instantiate traffic light.

        Attributes
        ----------
        baseline: bool
        """
        self._tls_properties = dict()

        # all traffic light parameters are set to default baseline values
        self.baseline = baseline

    def get_properties(self):
        """
        Return traffic light properties.

        This is meant to be used by the generator to import traffic light data
        to the .net.xml file. It is also useful for JSON transfers.
        """
        return self._tls_properties

    def set_properties(self, tls_properties):
        """
        Set the traffic light properties dictionary.

        Meant to be used in JSON transfers
        """
        self._tls_properties = tls_properties


class SumoTrafficLightParams(TrafficLightParams):
    """SUMO traffic light.

    This class is used to place traffic lights in the network and describe
    the state of these traffic lights. In addition, this class supports
    modifying the states of certain lights via TraCI.
    """
    def add(self,
            node_id,
            tls_type="static",
            programID=10,
            offset=None,
            phases=None,
            maxGap=None,
            detectorGap=None,
            showDetectors=None,
            file=None,
            freq=None):
        """Add a traffic light component to the network.

        When generating networks using xml files, using this method to add a
        traffic light will explicitly place the traffic light in the requested
        node of the generated network.

        If traffic lights are not added here but are already present in the
        network (e.g. through a prebuilt net.xml file), then the traffic light
        class will identify and add them separately.

        Parameters
        ----------
        node_id : str
            name of the node with traffic lights
        tls_type : str, optional
            type of the traffic light (see Note)
        programID : str, optional
            id of the traffic light program (see Note)
        offset : int, optional
            initial time offset of the program
        phases : list of dict, optional
            list of phases to be followed by the traffic light, defaults
            to default sumo traffic light behavior. Each element in the list
            must consist of a dict with two keys:

            * "duration": length of the current phase cycle (in sec)
            * "state": string consist the sequence of states in the phase
            * "minDur": optional
                The minimum duration of the phase when using type actuated
            * "maxDur": optional
                The maximum duration of the phase when using type actuated

        maxGap : int, optional
            describes the maximum time gap between successive vehicle that will
            cause the current phase to be prolonged, **used for actuated
            traffic lights**
        detectorGap : int, optional
            used for actuated traffic lights
            determines the time distance between the (automatically generated)
            detector and the stop line in seconds (at each lanes maximum
            speed), **used for actuated traffic lights**
        showDetectors : bool, optional
            toggles whether or not detectors are shown in sumo-gui, **used for
            actuated traffic lights**
        file : str, optional
            which file the detector shall write results into
        freq : int, optional
            the period over which collected values shall be aggregated

        Note
        ----
        For information on defining traffic light properties, see:
        http://sumo.dlr.de/wiki/Simulation/Traffic_Lights#Defining_New_TLS-Programs
        """
        # prepare the data needed to generate xml files
        self._tls_properties[node_id] = {"id": node_id, "type": tls_type}

        if programID:
            self._tls_properties[node_id]["programID"] = programID

        if offset:
            self._tls_properties[node_id]["offset"] = offset

        if phases:
            self._tls_properties[node_id]["phases"] = phases

        if tls_type == "actuated":
            # Required parameters
            self._tls_properties[node_id]["max-gap"] = \
                maxGap if maxGap else MAX_GAP
            self._tls_properties[node_id]["detector-gap"] = \
                detectorGap if detectorGap else DETECTOR_GAP
            self._tls_properties[node_id]["show-detectors"] = \
                showDetectors if showDetectors else SHOW_DETECTORS

            # Optional parameters
            if file:
                self._tls_properties[node_id]["file"] = file

            if freq:
                self._tls_properties[node_id]["freq"] = freq

    def actuated_default(self):
        """Return the default values for an actuated network.

        An actuated network is a network for a system where
        all junctions are actuated traffic lights.

        Returns
        -------
        tl_logic : dict
            traffic light logic
        """
        tl_type = "actuated"
        program_id = 1
        max_gap = 3.0
        detector_gap = 0.8
        show_detectors = True
        phases = [{
            "duration": "31",
            "minDur": "8",
            "maxDur": "45",
            "state": "GrGr"
        }, {
            "duration": "6",
            "minDur": "3",
            "maxDur": "6",
            "state": "yryr"
        }, {
            "duration": "31",
            "minDur": "8",
            "maxDur": "45",
            "state": "rGrG"
        }, {
            "duration": "6",
            "minDur": "3",
            "maxDur": "6",
            "state": "ryry"
        }]

        return {
            "tl_type": str(tl_type),
            "program_id": str(program_id),
            "max_gap": str(max_gap),
            "detector_gap": str(detector_gap),
            "show_detectors": show_detectors,
            "phases": phases
        }


class AimsunTrafficLightParams(TrafficLightParams):
    """ Aimsun traffic light

    This class is used to place traffic lights on nodes and
    meterings on sections (edges) of an Aimsun network.

#TODO: Add documentation for additional arguments
    """
    def __init__(self,
                 master_control_plan=None,
                 control_plan=None,
                 actuated_ids=set(),
                 conv_to_external_ids=set(),
                 baseline=False):
        """
        Instantiate the Aimsun traffic light parameters object.
        """
        super().__init__(baseline)
        self.master_control_plan = master_control_plan
        self.control_plan = control_plan
        self.actuated_ids = actuated_ids
        self.conv_to_external_ids = conv_to_external_ids # Import program logic from Aimsun and make external (Wolf-controlled)

    def add(self, program_logic_like, actuated=False):
        """
        Adds a traffic light to a node.

        The traffic light phases (and other relevant information for creating the TL)
        are collected in an AimsunProgramLogic object.

        Parameters
        ----------
            program_logic_like : AimsunProgramLogic or AimsunProgramLogicTemplate object
                The information necessary for configuring and controlling
                the traffic light
            actuated : Boolean, optional
                Whether or not to generate detectors for each turn of the
                program logic at Aimsun load-time. The detectors will be
                generated iff the 'actuated' argument is True.
        """
        self._tls_properties[program_logic_like.node_id] = program_logic_like

        if actuated:
            self.actuated_ids.add(program_logic_like.node_id)



class VehicleParams:
    """Base vehicle class.

    This is used to describe the state of all vehicles in the network.
    State information on the vehicles for a given time step can be set or
    retrieved from this class.
    """

    def __init__(self):
        """Instantiate the base vehicle class."""
        self.ids = []  # ids of all vehicles

        # vehicles: Key = Vehicle ID, Value = Dictionary describing the vehicle
        # Ordered dictionary used to keep neural net inputs in order
        self.__vehicles = collections.OrderedDict()

        #: total number of vehicles in the network
        self.num_vehicles = 0
        #: int : number of rl vehicles in the network
        self.num_rl_vehicles = 0
        #: int : number of unique types of vehicles in the network
        self.num_types = 0
        #: list of str : types of vehicles in the network
        self.types = []

        #: dict (str, str) : contains the parameters associated with each type
        #: of vehicle
        self.type_parameters = dict()

        #: dict (str, int) : contains the minGap attribute of each type of
        #: vehicle
        self.minGap = dict()

        #: list : initial state of the vehicles class, used for serialization
        #: purposes
        self.initial = []

    def add(self,
            veh_id,
            acceleration_controller=(SimCarFollowingController, {}),
            lane_change_controller=(SimLaneChangeController, {}),
            routing_controller=None,
            initial_speed=0,
            num_vehicles=0,
            car_following_params=None,
            lane_change_params=None,
            length=None,
            color=None,
            simulator='traci'):
        """Add a sequence of vehicles to the list of vehicles in the network.

        Parameters
        ----------
        veh_id : str
            base vehicle ID for the vehicles (will be appended by a number)
        acceleration_controller : tup, optional
            1st element: flow-specified acceleration controller
            2nd element: controller parameters (may be set to None to maintain
            default parameters)
        lane_change_controller : tup, optional
            1st element: flow-specified lane-changer controller
            2nd element: controller parameters (may be set to None to maintain
            default parameters)
        routing_controller : tup, optional
            1st element: flow-specified routing controller
            2nd element: controller parameters (may be set to None to maintain
            default parameters)
        initial_speed : float, optional
            initial speed of the vehicles being added (in m/s)
        num_vehicles : int, optional
            number of vehicles of this type to be added to the network
        car_following_params : flow.core.params.SumoCarFollowingParams
            Params object specifying attributes for Sumo car following model.
        lane_change_params : flow.core.params.SumoLaneChangeParams
            Params object specifying attributes for Sumo lane changing model.
        length : float, optional
            Vehicle length
        simulator : string, optional
            The simulator used (determines which parameter classes are used
            by default).
        """
        if car_following_params is None:
            car_following_params = SumoCarFollowingParams() if simulator == 'traci' \
                                   else AimsunCarFollowingParams()

        if lane_change_params is None:
            lane_change_params = SumoLaneChangeParams() if simulator == 'traci' \
                                 else AimsunLaneChangeParams()

        type_params = {}
        type_params.update(car_following_params.controller_params)
        type_params.update(lane_change_params.controller_params)

        # This dict will be used when trying to introduce new vehicles into
        # the network via a Flow. It is passed to the vehicle kernel object
        # during environment instantiation.
        self.type_parameters[veh_id] = \
            {"acceleration_controller": acceleration_controller,
             "lane_change_controller": lane_change_controller,
             "routing_controller": routing_controller,
             "initial_speed": initial_speed,
             "car_following_params": car_following_params,
             "lane_change_params": lane_change_params}

        if length:
            type_params['length'] = length
            self.type_parameters[veh_id]['length'] = length

        if color:
            type_params['color'] = color
            self.type_parameters[veh_id]['color'] = color

        # TODO: delete?
        self.initial.append({
            "veh_id":
                veh_id,
            "acceleration_controller":
                acceleration_controller,
            "lane_change_controller":
                lane_change_controller,
            "routing_controller":
                routing_controller,
            "initial_speed":
                initial_speed,
            "num_vehicles":
                num_vehicles,
            "car_following_params":
                car_following_params,
            "lane_change_params":
                lane_change_params
        })

        # This is used to return the actual headways from the vehicles class.
        # It is passed to the vehicle kernel class during environment
        # instantiation.
        self.minGap[veh_id] = type_params.get("minGap", None)

        for i in range(num_vehicles):
            v_id = veh_id + '_%d' % i

            # add the vehicle to the list of vehicle ids
            self.ids.append(v_id)

            self.__vehicles[v_id] = dict()

            # specify the type
            self.__vehicles[v_id]["type"] = veh_id

            # update the number of vehicles
            self.num_vehicles += 1
            if acceleration_controller[0] == RLController:
                self.num_rl_vehicles += 1

        # increase the number of unique types of vehicles in the network, and
        # add the type to the list of types
        self.num_types += 1
        self.types.append({"veh_id": veh_id, "type_params": type_params})

    def get_type(self, veh_id):
        """Return the type of a specified vehicle.

        Parameters
        ----------
        veh_id : str
            vehicle ID whose type the user is querying
        """
        return self.__vehicles[veh_id]["type"]


class SimParams(object):
    """Simulation-specific parameters.

    All subsequent parameters of the same type must extend this.

    Attributes
    ----------
    sim_step : float optional
        seconds per simulation step; 0.1 by default
    render : str or bool, optional
        specifies whether to visualize the rollout(s)

        * False: no rendering
        * True: render in the simulator
    save_render: bool, optional
        Whether to save rendering to video directory.

    restart_instance : bool, optional
        specifies whether to restart a simulation upon reset. Restarting
        the instance helps avoid slowdowns cause by excessive inflows over
        large experiment runtimes, but also require the GUI to be started
        after every reset if "render" is set to True.
    emission_path : str, optional
        Path to the folder in which to create the emissions output.
        Emissions output is not generated if this value is not specified

    horizon_time : int or float, optional
        The simulation horizon past warm-up time. Defaults to infinity
    horizon_simsteps : int, optional
        Horizon measured in simulator steps

    warmup_time : int or float, optional
        Simulation time before the initialization of training
        during a rollout. These warmup steps are not added as steps
        into training, and the actions of rl agents during these steps
        are dictated by Aimsun. Defaults to zero
    warmup_simsteps : int, optional
        Similar to warmup_time, but measured in simulator steps

    Preconditions
    -------------
        If both warmup_time and warmup_simsteps are passed,
        the values must be consistent with respect to sim_step.
        Similarly for horizon_time and horizon_simsteps.
    """

    def __init__(self,
                 sim_step=0.1,
                 render=False,
                 save_render=False,
                 restart_instance=False,
                 emission_path=None,
                 start_time=0,
                 warmup_time=0,
                 warmup_simsteps=0,
                 horizon_time=float('inf'),
                 horizon_simsteps=float('inf')):
        """Instantiate SimParams."""
        self.sim_step = sim_step
        self.render = render
        self.save_render = save_render
        self.restart_instance = restart_instance
        self.emission_path = emission_path

        self.start_time = start_time

        def assert_timeunit_consistency(time, simsteps, default):
            if time != default and simsteps != default:
                assert isclose(time, simsteps * sim_step, rel_tol=1e-4)
            elif time != default:
                simsteps = round(time / sim_step)
            elif simsteps != default:
                time = round(simsteps * sim_step, 2)
            return time, simsteps

        self.horizon_time, self.horizon_simsteps = \
            assert_timeunit_consistency(
                horizon_time, horizon_simsteps, float('inf'))

        self.warmup_time, self.warmup_simsteps = \
            assert_timeunit_consistency(
                warmup_time, warmup_simsteps, 0)


class AimsunParams(SimParams):
    """ Aimsun-specific simulation parameters. Extends SimParams.

    Attributes
    ----------
    In addition to the SimParams attributes:

    replication : int
        Aimsun ID of the replication to be simulated.
        The scenario and experiment will be deduced from the replication.


                  start_time
    |-----------------|--------------------------------------------------|
     <- warmup_time -> <-                horizon_time                  ->

    start_time : Int or Float or None, optional
        The start time of the simulation, in seconds from 12:00:00 AM.
        If None, the default start time of the scenario will be used.
        Defaults to None.


    traffic_demand: str or None, optional
        Name of the traffic demand to be used in the experiment.
        If None, Wolf will attempt to generate a new traffic demand using
        VehicleParams and InFlows.
    centroid_config_name : str, optional
        name of the centroid configuration to load in Aimsun. This
        parameter is only used when loading an Aimsun template,
        not when generating one.
    subnetwork_id : int, optional
        The id of the subnetwork to load in Aimsun. This parameter is not
        used when generating a network; it can be used when loading an
        Aimsun template containing a subnetwork in order to only load
        the objects contained in this subnetwork. If set to None or if the
        specified subnetwork does not exist, the whole network will be loaded.

    stats_collection_interval : Int or Float
                                or (Int, Int, Int) or [Int, Int, Int], optional
        The duration of the interval used for collection of statistics
        (measured either in seconds or in (hour, minute, sec)).
        If equal to 0, then statistics are not collected.
    detection_interval: Int or Float
                        or (Int, Int, Int) or [Int, Int, Int], optional
        The duration of the interval used for collection of detector readings.
        If equal to 0, then detectors are not active.

    initial_state_id: Int, optional
        The Aimsun id of the initial state. If None, the warmup time will be used.
        If not None, then a warmup time will not be used. Defaults to None.

    record_routes : Bool, optional
        Whether to keep buffered lists of vehicles that enter a new
        section. Defaults to False.
    evaluate : Dict or False, optional
        If a nonempty dictionary is passed, configures the information that
        will be stored for evaluation.

        The following (optional) keys can be passed:
            'save_to': String or os.path
                The directory where the data will be saved.
            'net': Bool
                If key is passed, the network graph will be stored in JSON format
                (saved with extension .net.json)
            'obsrew': Bool
                If the key is passed, the observations and rewards will be
                dumped in JSON format (saved with extension .obsrew.json)
            'sqlite': Dict or False
                If a nonempty dictionary is passed, configures the SQLITE DB export
                    'save_to': String
                        The filename
                    'vehs', 'detectors', 'network', 'sections', 'lanes'
                        These keys configure the statistics that will be stored
                        in the database
            'xmlanim': Bool
                If key is passed, saves the vehicle trajectories XML file.
    verbose : Bool, optional
        If False, filters out some of the messages posted to the console
        when Aimsun is being loaded, or is running. Defaults to True.
    seed : 'random' or Int, optional
        The seed to use. If the string 'random' is passed, a random integer seed
        will be generated at Aimsun API initialization.
    simulator_restart_period: Int, optional
        More flexible than "restart_instance". If equal to N != -1, then
        the simulator will be restarted every N environment resets. The
        intended purpose is a brute-force way of clearing simulator memory
        leaks. Defaults to -1.
    staggered_init_address : (String, Int), optional
        TCP address to use if using an Aimsun license
        semaphore (used in cases when launching several
        Aimsun instances simultaneously in parallel freezes
        the license server)
    nthreads: Dictionary, optional
        'sim': Int
            Number of threads to use for simulation
        'routing': Int
            Number of threads to use for routing
    """

    def __init__(self,
                 replication,
                 sim_step=0.1,
                 render=False,
                 save_render=False,
                 restart_instance=False,
                 simulator_restart_period=-1,
                 emission_path=None,
                 start_time=None,
                 warmup_time=0,
                 warmup_simsteps=0,
                 initial_state_id=None,
                 horizon_time=float('inf'),
                 horizon_simsteps=float('inf'),
                 traffic_demand=None,
                 centroid_config_name=None,
                 subnetwork_id=None,
                 stats_collection_interval=0,
                 detection_interval=0,
                 record_routes=False,
                 evaluate=False,
                 verbose=True,
                 seed=WOLF_SEED,
                 staggered_init_address=None,
                 nthreads={}):
        """Instantiate AimsunParams."""
        super().__init__(
            sim_step, render, save_render, restart_instance, emission_path,
            start_time, warmup_time, warmup_simsteps,
            horizon_time, horizon_simsteps)
        self.replication = replication
        self.traffic_demand = traffic_demand
        self.centroid_config_name = centroid_config_name
        self.subnetwork_id = subnetwork_id

        assert((warmup_time == 0) or (initial_state_id is None))
        self.initial_state_id = initial_state_id

        self.stats_collection_interval = stats_collection_interval
        self.detection_interval = detection_interval

        self.record_routes = record_routes
        self.evaluate = evaluate
        self.verbose = verbose
        self.seed = seed
        self.staggered_init_address = staggered_init_address
        self.nthreads = nthreads

        assert not (restart_instance and (simulator_restart_period > 1))
        self.simulator_restart_period = simulator_restart_period

class SumoParams(SimParams):
    """
    Sumo-specific simulation parameters. Extends SimParams.

    These parameters are used to customize a sumo simulation instance upon
    initialization. This includes passing the simulation step length,
    specifying whether to use sumo's gui during a run, and other features
    described in the Attributes below.

    Attributes
    ----------
    sim_step : float optional
        seconds per simulation step; 0.1 by default
    emission_path : str, optional
        Path to the folder in which to create the emissions output.
        Emissions output is not generated if this value is not specified
    render : str or bool, optional
        specifies whether to visualize the rollout(s)

        * False: no rendering
        * True: delegate rendering to sumo-gui
    save_render: bool, optional
        Whether to save rendering to video directory.
    restart_instance : bool, optional
        specifies whether to restart a sumo instance upon reset. Restarting
        the instance helps avoid slowdowns cause by excessive inflows over
        large experiment runtimes, but also require the gui to be started
        after every reset if "render" is set to True.

    port : int, optional
        Port for Traci to connect to; finds an empty port by default
    start_at_load : bool, optional
        If set to False, the user needs to manually start the simulation
        in the GUI
    lateral_resolution : float, optional
        width of the divided sublanes within a lane, defaults to None (i.e.
        no sublanes). If this value is specified, the vehicle in the
        network cannot use the "LC2013" lane change model.
    no_step_log : bool, optional
        specifies whether to add sumo's step logs to the log file, and
        print them into the terminal during runtime, defaults to True
    overtake_right : bool, optional
        whether vehicles are allowed to overtake on the right as well as
        the left
    seed : int, optional
        seed for sumo instance
    save_rng_state : bool, optional
        Include the state of the random number generators when saving
        state
    print_warnings : bool, optional
        If set to false, this will silence sumo warnings on the stdout
    delay_between_simsteps : float, optional
        Wall-clock time delay between sim-steps
    teleport_time : int, optional
        If negative, vehicles don't teleport in gridlock. If positive,
        they teleport after teleport_time seconds
    num_clients : int, optional
        Number of clients that will connect to Traci
    color_by_speed : bool
        whether to color the vehicles by the speed they are moving at the
        current time step
    use_ballistic: bool, optional
        If true, use a ballistic integration step instead of an euler step
    """

    def __init__(self,
                 sim_step=0.1,
                 render=False,
                 save_render=False,
                 emission_path=None,
                 restart_instance=False,
                 start_time=0.,
                 warmup_time=0,
                 warmup_simsteps=0,
                 horizon_time=float('inf'),
                 horizon_simsteps=float('inf'),
                 port=None,
                 start_at_load=True,
                 lateral_resolution=None,
                 no_step_log=True,
                 overtake_right=False,
                 seed=None,
                 save_rng_state=False,
                 print_warnings=True,
                 delay_between_simsteps=0.,
                 teleport_time=-1,
                 num_clients=1,
                 color_by_speed=False,
                 use_ballistic=False):
        """Instantiate SumoParams."""
        super().__init__(
            sim_step, render, save_render, restart_instance, emission_path,
            start_time, warmup_time, warmup_simsteps,
            horizon_time, horizon_simsteps)

        self.port = port
        self.start_at_load = start_at_load
        self.lateral_resolution = lateral_resolution
        self.no_step_log = no_step_log
        self.seed = seed
        self.save_rng_state = save_rng_state
        self.overtake_right = overtake_right
        self.print_warnings = print_warnings
        self.delay_between_simsteps = delay_between_simsteps
        self.teleport_time = teleport_time
        self.num_clients = num_clients
        self.color_by_speed = color_by_speed
        self.use_ballistic = use_ballistic


class EnvParams:
    """Environment and experiment-specific parameters.

    This includes specifying the bounds of the action space and relevant
    coefficients to the reward function, as well as specifying how the
    positions of vehicles are modified in between rollouts.

    Attributes
    ----------
    additional_params : dict, optional
        Specify additional environment params for a specific
        environment configuration
    sims_per_step : int, optional
        number of sumo simulation steps performed in any given rollout
        step. RL agents perform the same action for the duration of
        these simulation steps.
    evaluate : bool, optional
        flag indicating that the evaluation reward should be used
        so the evaluation reward should be used rather than the
        normal reward
    clip_actions : bool, optional
        specifies whether to clip actions from the policy by their range when
        they are inputted to the reward function. Note that the actions are
        still clipped before they are provided to `apply_rl_actions`.
    save_agent_data : bool, optional
        Specifies whether to save the actions, rewards and obs for
        each agent (saved to wolf/other/tmp).
    """

    def __init__(self,
                 additional_params=None,
                 horizon=100, #TODO: Remove
                 sims_per_step=1,
                 evaluate=False,
                 clip_actions=True,
                 save_agent_data=False):
        """Instantiate EnvParams."""
        self.additional_params = \
            additional_params if additional_params is not None else {}
        self.sims_per_step = sims_per_step
        self.horizon=horizon
        self.evaluate = evaluate
        self.clip_actions = clip_actions
        self.save_agent_data = save_agent_data

    def get_additional_param(self, key):
        """Return a variable from additional_params."""
        return self.additional_params[key]


class NetParams:
    """Network configuration parameters.

    Unlike most other parameters, NetParams may vary drastically dependent
    on the specific network configuration. For example, for the ring road
    the network parameters will include a characteristic length, number of
    lanes, and speed limit.

    In order to determine which additional_params variable may be needed
    for a specific network, refer to the ADDITIONAL_NET_PARAMS variable
    located in the network file.

    Attributes
    ----------
    inflows : InFlows type, optional
        Specifies the inflows of specific edges and the types of vehicles
        entering the network from these edges
    od_config : OriginDestinationConfig type, optional
        Specifies the demand using an origin-destination matrix.
        Contains inflow edge ids, outflow edge ids, and a matrix giving
        the number of trips between every (inflow, outflow) pair
    osm_path : str, optional
        path to the .osm file that should be used to generate the network
        configuration files
    template : str, optional
        path to the network template file that can be used to instantiate a
        netowrk in the simulator of choice
    additional_params : dict, optional
        network specific parameters; see each subclass for a description of
        what is needed
    """

    def __init__(self,
                 inflows=None,
                 od_config=None,
                 osm_path=None,
                 template=None,
                 additional_params=None):
        """Instantiate NetParams."""
        assert inflows is None or od_config is None,\
            'Demand must be defined either by turning proportions or by '\
            'an origin-destination matrix for a centroid configuration, '\
            'not both.'
        self.inflows = inflows or InFlows()
        self.od_config = od_config
        self.osm_path = osm_path
        self.template = template
        self.additional_params = additional_params or {}


class InitialConfig:
    """Initial configuration parameters.

    These parameters that affect the positioning of vehicle in the
    network at the start of a rollout. By default, vehicles are uniformly
    distributed in the network.

    Attributes
    ----------
    shuffle : bool, optional  # TODO: remove
        specifies whether the ordering of vehicles in the Vehicles class
        should be shuffled upon initialization.
    spacing : str, optional
        specifies the positioning of vehicles in the network relative to
        one another. May be one of: "uniform", "random", or "custom".
        Default is "uniform".
    min_gap : float, optional  # TODO: remove
        minimum gap between two vehicles upon initialization, in meters.
        Default is 0 m.
    x0 : float, optional  # TODO: remove
        position of the first vehicle to be placed in the network
    perturbation : float, optional
        standard deviation used to perturb vehicles from their uniform
        position, in meters. Default is 0 m.
    bunching : float, optional
        reduces the portion of the network that should be filled with
        vehicles by this amount.
    lanes_distribution : int, optional
        number of lanes vehicles should be dispersed into. If the value is
        greater than the total number of lanes on an edge, vehicles are
        spread across all lanes.
    edges_distribution : str or list of str or dict, optional
        edges vehicles may be placed on during initialization, may be one
        of:

        * "all": vehicles are distributed over all edges
        * list of edges: list of edges vehicles can be distributed over
        * dict of edges: where the key is the name of the edge to be
          utilized, and the elements are the number of cars to place on
          each edge
    additional_params : dict, optional
        some other network-specific params
    """

    def __init__(self,
                 shuffle=False,
                 spacing="uniform",
                 min_gap=0,
                 perturbation=0.0,
                 x0=0,
                 bunching=0,
                 lanes_distribution=float("inf"),
                 edges_distribution="all",
                 additional_params=None):
        """Instantiate InitialConfig.

        These parameters that affect the positioning of vehicle in the
        network at the start of a rollout. By default, vehicles are uniformly
        distributed in the network.
        """
        self.shuffle = shuffle
        self.spacing = spacing
        self.min_gap = min_gap
        self.perturbation = perturbation
        self.x0 = x0
        self.bunching = bunching
        self.lanes_distribution = lanes_distribution
        self.edges_distribution = edges_distribution
        self.additional_params = additional_params or dict()


class SumoCarFollowingParams:
    """Parameters for sumo-controlled acceleration behavior.

    Attributes
    ----------
    speed_mode : str or int, optional
        may be one of the following:

         * "right_of_way" (default): respect safe speed, right of way and
           brake hard at red lights if needed. DOES NOT respect
           max accel and decel which enables emergency stopping.
           Necessary to prevent custom models from crashing
         * "obey_safe_speed": prevents vehicles from colliding
           longitudinally, but can fail in cases where vehicles are allowed
           to lane change
         * "no_collide": Human and RL cars are preventing from reaching
           speeds that may cause crashes (also serves as a failsafe). Note:
           this may lead to collisions in complex networks
         * "aggressive": Human and RL cars are not limited by sumo with
           regard to their accelerations, and can crash longitudinally
         * "all_checks": all sumo safety checks are activated
         * int values may be used to define custom speed mode for the given
           vehicles, specified at:
           http://sumo.dlr.de/wiki/TraCI/Change_Vehicle_State#speed_mode_.280xb3.29

    accel : float
        see Note
    decel : float
        see Note
    sigma : float
        see Note
    tau : float
        see Note
    min_gap : float
        see minGap Note
    max_speed : float
        see maxSpeed Note
    speed_factor : float
        see speedFactor Note
    speed_dev : float
        see speedDev in Note
    impatience : float
        see Note
    car_follow_model : str
        see carFollowModel in Note
    kwargs : dict
        used to handle deprecations

    Note
    ----
    For a description of all params, see:
    http://sumo.dlr.de/wiki/Definition_of_Vehicles,_Vehicle_Types,_and_Routes
    """

    def __init__(
            self,
            speed_mode='right_of_way',
            accel=2.6,
            decel=4.5,
            sigma=0.5,
            tau=1.0,  # past 1 at sim_step=0.1 you no longer see waves
            min_gap=2.5,
            max_speed=30,
            speed_factor=1.0,
            speed_dev=0.1,
            impatience=0.5,
            car_follow_model="IDM",
            **kwargs):
        """Instantiate SumoCarFollowingParams."""
        """
        # check for deprecations (minGap)
        if "minGap" in kwargs:
            deprecated_attribute(self, "minGap", "min_gap")
            min_gap = kwargs["minGap"]

        # check for deprecations (maxSpeed)
        if "maxSpeed" in kwargs:
            deprecated_attribute(self, "maxSpeed", "max_speed")
            max_speed = kwargs["maxSpeed"]

        # check for deprecations (speedFactor)
        if "speedFactor" in kwargs:
            deprecated_attribute(self, "speedFactor", "speed_factor")
            speed_factor = kwargs["speedFactor"]

        # check for deprecations (speedDev)
        if "speedDev" in kwargs:
            deprecated_attribute(self, "speedDev", "speed_dev")
            speed_dev = kwargs["speedDev"]

        # check for deprecations (carFollowModel)
        if "carFollowModel" in kwargs:
            deprecated_attribute(self, "carFollowModel", "car_follow_model")
            car_follow_model = kwargs["carFollowModel"]
        """

        # create a controller_params dict with all the specified parameters
        self.controller_params = {
            "accel": accel,
            "decel": decel,
            "sigma": sigma,
            "tau": tau,
            "minGap": min_gap,
            "maxSpeed": max_speed,
            "speedFactor": speed_factor,
            "speedDev": speed_dev,
            "impatience": impatience,
            "carFollowModel": car_follow_model,
        }

        # adjust the speed mode value
        if isinstance(speed_mode, str) and speed_mode in SPEED_MODES:
            speed_mode = SPEED_MODES[speed_mode]
        elif not (isinstance(speed_mode, int)
                  or isinstance(speed_mode, float)):
            logging.error("Setting speed mode of to default.")
            speed_mode = SPEED_MODES["obey_safe_speed"]

        self.speed_mode = speed_mode


class SumoLaneChangeParams:
    """Parameters for sumo-controlled lane change behavior.

    Attributes
    ----------
    lane_change_mode : str or int, optional
        may be one of the following:
        * "no_lc_safe" (default): Disable all SUMO lane changing but still
          handle safety checks (collision avoidance and safety-gap enforcement)
          in the simulation. Binary is [001000000000]
        * "no_lc_aggressive": SUMO lane changes are not executed, collision
          avoidance and safety-gap enforcement are off.
          Binary is [000000000000]

        * "sumo_default": Execute all changes requested by a custom controller
          unless in conflict with TraCI. Binary is [011001010101].

        * "no_strategic_aggressive": Execute all changes except strategic
          (routing) lane changes unless in conflict with TraCI. Collision
          avoidance and safety-gap enforcement are off. Binary is [010001010100]
        * "no_strategic_safe": Execute all changes except strategic
          (routing) lane changes unless in conflict with TraCI. Collision
          avoidance and safety-gap enforcement are on. Binary is [011001010100]
        * "only_strategic_aggressive": Execute only strategic (routing) lane
          changes unless in conflict with TraCI. Collision avoidance and
          safety-gap enforcement are off. Binary is [000000000001]
        * "only_strategic_safe": Execute only strategic (routing) lane
          changes unless in conflict with TraCI. Collision avoidance and
          safety-gap enforcement are on. Binary is [001000000001]

        * "no_cooperative_aggressive": Execute all changes except cooperative
          (change in order to allow others to change) lane changes unless in
          conflict with TraCI. Collision avoidance and safety-gap enforcement
          are off. Binary is [010001010001]
        * "no_cooperative_safe": Execute all changes except cooperative
          lane changes unless in conflict with TraCI. Collision avoidance and
          safety-gap enforcement are on. Binary is [011001010001]
        * "only_cooperative_aggressive": Execute only cooperative lane changes
          unless in conflict with TraCI. Collision avoidance and safety-gap
          enforcement are off. Binary is [000000000100]
        * "only_cooperative_safe": Execute only cooperative lane changes
          unless in conflict with TraCI. Collision avoidance and safety-gap
          enforcement are on. Binary is [001000000100]

        * "no_speed_gain_aggressive": Execute all changes except speed gain (the
           other lane allows for faster driving) lane changes unless in conflict
           with TraCI. Collision avoidance and safety-gap enforcement are off.
           Binary is [010001000101]
        * "no_speed_gain_safe": Execute all changes except speed gain
          lane changes unless in conflict with TraCI. Collision avoidance and
          safety-gap enforcement are on. Binary is [011001000101]
        * "only_speed_gain_aggressive": Execute only speed gain lane changes
          unless in conflict with TraCI. Collision avoidance and safety-gap
          enforcement are off. Binary is [000000010000]
        * "only_speed_gain_safe": Execute only speed gain lane changes
          unless in conflict with TraCI. Collision avoidance and safety-gap
          enforcement are on. Binary is [001000010000]

        * "no_right_drive_aggressive": Execute all changes except right drive
          (obligation to drive on the right) lane changes unless in conflict
          with TraCI. Collision avoidance and safety-gap enforcement are off.
          Binary is [010000010101]
        * "no_right_drive_safe": Execute all changes except right drive
          lane changes unless in conflict with TraCI. Collision avoidance and
          safety-gap enforcement are on. Binary is [011000010101]
        * "only_right_drive_aggressive": Execute only right drive lane changes
          unless in conflict with TraCI. Collision avoidance and safety-gap
          enforcement are off. Binary is [000001000000]
        * "only_right_drive_safe": Execute only right drive lane changes
          unless in conflict with TraCI. Collision avoidance and safety-gap
          enforcement are on. Binary is [001001000000]

        * int values may be used to define custom lane change modes for the
          given vehicles, specified at:
          http://sumo.dlr.de/wiki/TraCI/Change_Vehicle_State#lane_change_mode_.280xb6.29

    model : str, optional
        see laneChangeModel in Note
    lc_strategic : float, optional
        see lcStrategic in Note
    lc_cooperative : float, optional
        see lcCooperative in Note
    lc_speed_gain : float, optional
        see lcSpeedGain in Note
    lc_keep_right : float, optional
        see lcKeepRight in Note
    lc_look_ahead_left : float, optional
        see lcLookaheadLeft in Note
    lc_speed_gain_right : float, optional
        see lcSpeedGainRight in Note
    lc_sublane : float, optional
        see lcSublane in Note
    lc_pushy : float, optional
        see lcPushy in Note
    lc_pushy_gap : float, optional
        see lcPushyGap in Note
    lc_assertive : float, optional
        see lcAssertive in Note
    lc_accel_lat : float, optional
        see lcAccelLate in Note
    kwargs : dict
        used to handle deprecations

    Note
    ----
    For a description of all params, see:
    http://sumo.dlr.de/wiki/Definition_of_Vehicles,_Vehicle_Types,_and_Routes
    """

    def __init__(self,
                 lane_change_mode="no_lc_safe",
                 model="LC2013",
                 lc_strategic=1.0,
                 lc_cooperative=1.0,
                 lc_speed_gain=1.0,
                 lc_keep_right=1.0,
                 lc_look_ahead_left=2.0,
                 lc_speed_gain_right=1.0,
                 lc_sublane=1.0,
                 lc_pushy=0,
                 lc_pushy_gap=0.6,
                 lc_assertive=1,
                 lc_accel_lat=1.0,
                 **kwargs):
        """Instantiate SumoLaneChangeParams."""
        """
        # check for deprecations (lcStrategic)
        if "lcStrategic" in kwargs:
            deprecated_attribute(self, "lcStrategic", "lc_strategic")
            lc_strategic = kwargs["lcStrategic"]

        # check for deprecations (lcCooperative)
        if "lcCooperative" in kwargs:
            deprecated_attribute(self, "lcCooperative", "lc_cooperative")
            lc_cooperative = kwargs["lcCooperative"]

        # check for deprecations (lcSpeedGain)
        if "lcSpeedGain" in kwargs:
            deprecated_attribute(self, "lcSpeedGain", "lc_speed_gain")
            lc_speed_gain = kwargs["lcSpeedGain"]

        # check for deprecations (lcKeepRight)
        if "lcKeepRight" in kwargs:
            deprecated_attribute(self, "lcKeepRight", "lc_keep_right")
            lc_keep_right = kwargs["lcKeepRight"]

        # check for deprecations (lcLookaheadLeft)
        if "lcLookaheadLeft" in kwargs:
            deprecated_attribute(self, "lcLookaheadLeft", "lc_look_ahead_left")
            lc_look_ahead_left = kwargs["lcLookaheadLeft"]

        # check for deprecations (lcSpeedGainRight)
        if "lcSpeedGainRight" in kwargs:
            deprecated_attribute(self, "lcSpeedGainRight",
                                 "lc_speed_gain_right")
            lc_speed_gain_right = kwargs["lcSpeedGainRight"]

        # check for deprecations (lcSublane)
        if "lcSublane" in kwargs:
            deprecated_attribute(self, "lcSublane", "lc_sublane")
            lc_sublane = kwargs["lcSublane"]

        # check for deprecations (lcPushy)
        if "lcPushy" in kwargs:
            deprecated_attribute(self, "lcPushy", "lc_pushy")
            lc_pushy = kwargs["lcPushy"]

        # check for deprecations (lcPushyGap)
        if "lcPushyGap" in kwargs:
            deprecated_attribute(self, "lcPushyGap", "lc_pushy_gap")
            lc_pushy_gap = kwargs["lcPushyGap"]

        # check for deprecations (lcAssertive)
        if "lcAssertive" in kwargs:
            deprecated_attribute(self, "lcAssertive", "lc_assertive")
            lc_assertive = kwargs["lcAssertive"]

        # check for deprecations (lcAccelLat)
        if "lcAccelLat" in kwargs:
            deprecated_attribute(self, "lcAccelLat", "lc_accel_lat")
            lc_accel_lat = kwargs["lcAccelLat"]
        """

        # check for valid model
        if model not in ["LC2013", "SL2015"]:
            logging.error("Invalid lane change model! Defaulting to LC2013")
            model = "LC2013"

        if model == "LC2013":
            self.controller_params = {
                "laneChangeModel": model,
                "lcStrategic": str(lc_strategic),
                "lcCooperative": str(lc_cooperative),
                "lcSpeedGain": str(lc_speed_gain),
                "lcKeepRight": str(lc_keep_right),
                # "lcLookaheadLeft": str(lc_look_ahead_left),
                # "lcSpeedGainRight": str(lcSpeedGainRight)
            }
        elif model == "SL2015":
            self.controller_params = {
                "laneChangeModel": model,
                "lcStrategic": str(lc_strategic),
                "lcCooperative": str(lc_cooperative),
                "lcSpeedGain": str(lc_speed_gain),
                "lcKeepRight": str(lc_keep_right),
                "lcLookaheadLeft": str(lc_look_ahead_left),
                "lcSpeedGainRight": str(lc_speed_gain_right),
                "lcSublane": str(lc_sublane),
                "lcPushy": str(lc_pushy),
                "lcPushyGap": str(lc_pushy_gap),
                "lcAssertive": str(lc_assertive),
                "lcAccelLat": str(lc_accel_lat)
            }

        # adjust the lane change mode value
        if isinstance(lane_change_mode, str) and lane_change_mode in LC_MODES:
            lane_change_mode = LC_MODES[lane_change_mode]
        elif not (isinstance(lane_change_mode, int)
                  or isinstance(lane_change_mode, float)):
            logging.error("Setting lane change mode to default.")
            lane_change_mode = LC_MODES["no_lc_safe"]

        self.lane_change_mode = lane_change_mode


class AimsunCarFollowingParams:
    """
    Parameters for Aimsun-controlled acceleration behavior.

    These are a subset of the Vehicle Type attributes in Aimsun. The default vehicle-following
    model used in Aimsun is Gipps, with the option to enable Adaptive Cruise Control (ACC) and
    Collaborative Adaptive Cruise Control (CACC).

    Most of the attributes are random variables with a truncated normal distribution, which is
    characterized by the mean, standard deviation, min and max.

    Attributes
    ----------
        max_acc: dict
            {'mean': float, 'dev': float, 'min': float, 'max': float}
            Maximum Acceleration: This is the maximum acceleration, in m/s2, that the vehicle
                can achieve under any circumstances. As used in Gipps car-following model

        normal_dec: dict
            {'mean': float, 'dev': float, 'min': float, 'max': float}
            Normal Deceleration: This is the maximum deceleration, in m/s2, that the vehicle
                can achieve under normal circumstances. As used in Gipps car-following model

        max_dec: dict
            {'mean': float, 'dev': float, 'min': float, 'max': float}
            Maximum Deceleration: This is the most severe braking, in m/s2, that the vehicle
                can achieve under special circumstances. As used in Gipps car-following model

        safety_margin_factor: dict
            {'mean': float, 'dev': float, 'min': float, 'max': float}
            Safety Margin Factor: In the gap acceptance calculations to determine whether a
                vehicle can move at a priority junction, the safety margin is set in the Road
                Type parameters. This vehicle type parameter provides a multiplier, with a
                truncated normal range, to apply to the turn safety margin values.

        lateral_clearance: dict
            {'mean': float, 'dev': float, 'min': float, 'max': float}
            Lateral Clearance: The lateral clearance vehicle to vehicle variations come from
                distributions defined by vehicle type. The minimum lateral spacing between
                two vehicles is the sum of the lateral clearances of both vehicles.

        max_lateral_speed: dict
            {'mean': float, 'dev': float, 'min': float, 'max': float}
            Max Lateral Speed: When moving laterally, the vehicles use their maximum lateral speed.

        sensitivity_factor: dict
            {'mean': float, 'dev': float, 'min': float, 'max': float}
            Sensitivity Factor: In the deceleration component of the car-following model,
                the follower makes an estimation of the deceleration of the leader using
                the sensitivity factor.

        gap: dict
            {'mean': float, 'dev': float, 'min': float, 'max': float}
            Gap: This parameter is used to override the headway between vehicles and force
                a larger distance between vehicles than the value calculated by the car-following
                model. The default Gap parameter value of constant 0.0 implies the normal headway
                will be used, any other value forces a larger headway.

        headway_aggressiveness: dict
            {'mean': float, 'dev': float, 'min': float, 'max': float}
            Headway Aggressiveness: Modifies the relationship of the inter-vehicle distance as a
                function of speed. This distance is simply linear in the Gipps model and does not
                correspond to the observed behavior under congested highway conditions.

        stop_and_go: boolean
            Favors Stop and Go: Allows a vehicle to adjust how it uses its aggressiveness value.
            If True, +a is used during deceleration and -a is used during acceleration. Hence,
            when a > 0, the gap between vehicles will be larger during acceleration than deceleration
            for the same speed.

        ==== Adaptive Cruise Control (ACC) ====
        acc_params: dict
            'percent_acc': float (percentage)
                Percentage of vehicles of this type equipped with ACC

            'speed_gain_free_flow': float
                Speed Gain Free Flow: The gain on the speed difference between the free flow speed
                    and the vehicle's current speed (s-1)

            'speed_gain_following': float
                Speed Gain Following: The gain on the speed difference between the preceding vehicle
                    and the subject vehicle (s-1)

            'distance_gain': float
                Distance Gain: The gain on the position difference between the preceding vehicle and the
                    subject vehicle (s-2)

            'lower_clearance_threshold': float
            'upper_clearance_threshold': float
                  Lower Distance Threshold, Upper Distance Threshold: The thresholds for the space
                      between the rear bumper of a vehicle and the front bumper of the following (metres)

            'desired_time_gap': dict
                {'mean': float, 'dev': float, 'min': float, 'max': float}
                  Desired Time Gap: The desired time gap of the ACC controller (s)
                      (this is a random variable with a truncated normal distribution)

        ==== Collaborative Adaptive Cruise Control (CACC) ====
        cacc_params: dict
            'percent_cacc': float (percentage)
                Percentage of vehicles of this type equipped with CACC

            'speed_gain': float
                Speed Gain: The gain on the speed difference between the preceding connected vehicle
                    and the subject CACC vehicle

            'distance_gain': float
                Distance Gain: The gain on the position difference between the preceding connected
                    vehicle and the subject CACC vehicle (s-1)

            'time_gap_leader': float
            'time_gap_follower': float
                Time Gap Leader, Time Gap Follower: The constant time gap between the the connected leader
                    (follower) of the subject CACC vehicle (s)

            'lower_gap_threshold': float
            'upper_gap_threshold': float
                Lower Gap Threshold, Upper Gap Threshold: The upper and lower thresholds for the time gap (s)
    """
    def __init__(self,
                 max_acc = {},
                 normal_dec = {},
                 max_dec = {},
                 safety_margin_factor = {},
                 lateral_clearance = {},
                 sensitivity_factor = {},
                 gap = {},
                 headway_aggressiveness = {},
                 stop_and_go=False,
                 acc_params = {},
                 cacc_params = {}):

        # Default Aimsun values for Vehicle Type == Car
        self.controller_params = {
                           'max_acc': {'mean': 3., 'dev': 0.2, 'min': 2.6, 'max': 3.4},
                        'normal_dec': {'mean': 4., 'dev': 0.25,'min': 3.5, 'max': 4.5},
                           'max_dec': {'mean': 6., 'dev': 0.5, 'min': 5.,  'max': 7.},
              'safety_margin_factor': {'mean': 1., 'dev': 0.,  'min': 1.,  'max': 1.},
                 'lateral_clearance': {'mean': 0.3,'dev': 0.1, 'min': 0.2, 'max': 0.45},
                 'max_lateral_speed': {'mean': 3., 'dev': 0.,  'min': 3.,  'max': 3.},
                'sensitivity_factor': {'mean': 1., 'dev': 0.,  'min': 1.,  'max': 1.},
                               'gap': {'mean': 0., 'dev': 0.,  'min': 0.,  'max': 0.},
            'headway_aggressiveness': {'mean': 0., 'dev': 0.,  'min': -1., 'max': 1.},
            'stop_and_go': stop_and_go,
            'acc_params': {'percent_acc': 0.,
                           'speed_gain_free_flow': 0.4,
                           'speed_gain_following': 0.07,
                           'distance_gain': 0.23,
                           'lower_distance_threshold': 100.,
                           'upper_distance_threshold': 120.,
                           'desired_time_gap': {'mean': 1.2,
                                                'dev': 0.4,
                                                'min': 1.1,
                                                'max': 2.2},},
            'cacc_params': {'percent_cacc': 0.,
                            'speed_gain': 0.0125,
                            'distance_gain': 0.45,
                            'time_gap_leader': 1.5,
                            'time_gap_follower': 0.6,
                            'lower_gap_threshold': 1.5,
                            'upper_gap_threshold': 2.,},
        }

        # Update the default values with the passed values
        self.controller_params['max_acc'].update(max_acc)
        self.controller_params['normal_dec'].update(normal_dec)
        self.controller_params['max_dec'].update(max_dec)
        self.controller_params['safety_margin_factor'].update(safety_margin_factor)
        self.controller_params['lateral_clearance'].update(lateral_clearance)
        self.controller_params['sensitivity_factor'].update(sensitivity_factor)
        self.controller_params['gap'].update(gap)
        self.controller_params['headway_aggressiveness'].update(headway_aggressiveness)
        self.controller_params['acc_params'].update(acc_params)
        self.controller_params['cacc_params'].update(cacc_params)

        if not (0 <= (self.controller_params['acc_params']['percent_acc'] +
                      self.controller_params['cacc_params']['percent_cacc']) <= 100):
            raise ValueError('The sum of the percentages of vehicles equipped with ACC'
                             ' and with CACC should be between 0 and 100.')

        # Needed for base controller, recorded here for the moment
        self.controller_params['accel'] = self.controller_params['max_acc']['mean']
        self.controller_params['decel'] = self.controller_params['max_dec']['mean']


class AimsunLaneChangeParams:
    """
    Parameters for Aimsun-controlled lane-change behavior.

    These are a subset of the Vehicle Type attributes in Aimsun.

    Attributes
    ----------
        overtake_speed_threshold: float
        lane_recovery_speed_threshold: float
            Overtake Speed Threshold and Lane Recovery Speed Threshold: These two parameters
                control overtaking when a vehicle changes lane to pass another. If the vehicle
                is constrained to travel at less than the Overtaking Speed Threshold of its
                desired speed, it will consider an overtaking manouver. Lane Recovery Speed
                Threshold is the percentage of the desired speed of a vehicle above which a
                vehicle may decide to get back to the original lane.

        percent_staying_in_overtaking_lane: float
            Percentage Staying in Overtaking Lane: Probability that a vehicle will stay in the
                faster lane instead of recovering to a slower lane following an overtake
                manouver.

        imprudent_lane_changing: boolean
            Imprudent Lane Changing: Whether or not a vehicle of this type will still change
                lane after assesing an unsafe gap.

        cooperate_in_creating_gap: boolean
            Cooperate in Creating a Gap: Whether or not vehicles of this type can cooperate
                in creating a gap for a lane changing vehicle to accept.

        aggressiveness_level: dict
            {'mean': float, 'dev': float, 'min': float, 'max': float}
            Aggressiveness Level: The higher the level, the smaller the gap the vehicle will
                accept in lane-changing

        look_ahead_factor: dict
            {'min': float, 'max': float}
            Look-Ahead Distance Factor: Modify the look-aheads used in the lane changing model
                to determine where vehicles consider their lane choice for a forthcoming turn.

        allow_non_lane_based: boolean
            Allow Vehicles Non-Lane Based Behavior: Whether the vehicle of this type can consider
                non-lane based movements or not.

        margin_for_overtaking: dict
            {'mean': float, 'dev': float, 'min': float, 'max': float}
            Margin for Overtaking Manouver: Safety margin for two-way overtaking
                (truncated normal) (s)
    """
    def __init__(self,
                 overtake_speed_threshold=90.,
                 lane_recovery_speed_threshold=95.,
                 percent_staying_in_overtaking_lane=0.,
                 imprudent_lane_changing=True,
                 cooperate_in_creating_gap=True,
                 aggressiveness_level={},
                 look_ahead_factor={},
                 allow_non_lane_based=False,
                 margin_for_overtaking={}):

        # Default Aimsun values for Vehicle Type == Car
        self.controller_params = {
            'overtake_speed_threshold': overtake_speed_threshold,
            'lane_recovery_speed_threshold': lane_recovery_speed_threshold,
            'percent_staying_in_overtaking_lane': percent_staying_in_overtaking_lane,
            'imprudent_lane_changing': imprudent_lane_changing,
            'cooperate_in_creating_gap': cooperate_in_creating_gap,
            'aggressiveness_level': {'mean': 0.2, 'dev': 0., 'min': 0., 'max': 1.},
            'look_ahead_factor': {'min': 0.8, 'max': 1.2},
            'allow_non_lane_based': allow_non_lane_based,
            'margin_for_overtaking': {'mean': 5.,
                                      'dev': 3.,
                                      'min': 1.,
                                      'max': 10.}
        }

        # Update the default values with the passed values
        self.controller_params['aggressiveness_level'].update(aggressiveness_level)
        self.controller_params['look_ahead_factor'].update(look_ahead_factor)
        self.controller_params['margin_for_overtaking'].update(margin_for_overtaking)


class InFlows:
    """Used to add inflows to a network.

    Inflows can be specified for any edge that has a specified route or routes.
    """

    def __init__(self):
        """Instantiate Inflows."""
        self.__flows = []

    def add(self,
            edge,
            veh_type,
            vehs_per_hour=None,
            probability=None,
            period=None,
            depart_lane="first",
            depart_speed=0,
            name="flow",
            begin=1,
            end=86400,
            number=None,
            **kwargs):
        """Specify a new inflow for a given type of vehicles and edge.

        Parameters
        ----------
        edge : str
            starting edge for the vehicles in this inflow
        veh_type : str
            type of the vehicles entering the edge. Must match one of the types
            set in the Vehicles class
        vehs_per_hour : float, optional
            number of vehicles per hour, equally spaced (in vehicles/hour).
            Cannot be specified together with probability or period
        probability : float, optional
            probability for emitting a vehicle each second (between 0 and 1).
            Cannot be specified together with vehs_per_hour or period
        period : float, optional
            insert equally spaced vehicles at that period (in seconds). Cannot
            be specified together with vehs_per_hour or probability
        depart_lane : int or str
            the lane on which the vehicle shall be inserted. Can be either one
            of:

            * int >= 0: index of the lane (starting with rightmost = 0)
            * "random": a random lane is chosen, but the vehicle insertion is
              not retried if it could not be inserted
            * "free": the most free (least occupied) lane is chosen
            * "best": the "free" lane (see above) among those who allow the
              vehicle the longest ride without the need to change lane
            * "first": the rightmost lane the vehicle may use

            Defaults to "first".
        depart_speed : float or str
            the speed with which the vehicle shall enter the network (in m/s)
            can be either one of:

            - float >= 0: the vehicle is tried to be inserted using the given
              speed; if that speed is unsafe, departure is delayed
            - "random": vehicles enter the edge with a random speed between 0
              and the speed limit on the edge; the entering speed may be
              adapted to ensure a safe distance to the leading vehicle is kept
            - "speedLimit": vehicles enter the edge with the maximum speed that
              is allowed on this edge; if that speed is unsafe, departure is
              delayed

            Defaults to 0.
        name : str, optional
            prefix for the id of the vehicles entering via this inflow.
            Defaults to "flow"
        begin : float, optional
            first vehicle departure time (in seconds, minimum 1 second).
            Defaults to 1 second
        end : float, optional
            end of departure interval (in seconds). This parameter is not taken
            into account if 'number' is specified. Defaults to 24 hours
        number : int, optional
            total number of vehicles the inflow should create (due to rounding
            up, this parameter may not be exactly enforced and shouldn't be set
            too small). Default: infinite (c.f. 'end' parameter)
        kwargs : dict, optional
            see Note

        Note
        ----
        For information on the parameters start, end, vehs_per_hour,
        probability, period, number, as well as other vehicle type and routing
        parameters that may be added via \*\*kwargs, refer to:
        http://sumo.dlr.de/wiki/Definition_of_Vehicles,_Vehicle_Types,_and_Routes
        """
        new_inflow = {
            "name": "%s_%d" % (name, len(self.__flows)),
            "vtype": veh_type,
            "edge": edge,
            "departLane": depart_lane,
            "departSpeed": depart_speed,
            "begin": begin,
            "end": end
        }
        new_inflow.update(kwargs)

        inflow_params = [vehs_per_hour, probability, period]
        n_inflow_params = len(inflow_params) - inflow_params.count(None)
        if n_inflow_params != 1:
            raise ValueError(
                "Exactly one among the three parameters 'vehs_per_hour', "
                "'probability' and 'period' must be specified in InFlows.add. "
                "{} were specified.".format(n_inflow_params))
        if probability is not None and (probability < 0 or probability > 1):
            raise ValueError(
                "Inflow.add called with parameter 'probability' set to {}, but"
                " probability should be between 0 and 1.".format(probability))
        if begin is not None and begin < 1:
            raise ValueError(
                "Inflow.add called with parameter 'begin' set to {}, but begin"
                " should be greater or equal than 1 second.".format(begin))

        if number is not None:
            del new_inflow["end"]
            new_inflow["number"] = number

        if vehs_per_hour is not None:
            new_inflow["vehsPerHour"] = vehs_per_hour
        if probability is not None:
            new_inflow["probability"] = probability
        if period is not None:
            new_inflow["period"] = period

        self.__flows.append(new_inflow)

    def sort(self, key):
        self.__flows.sort(key=key)

    def get(self):
        """Return the inflows of each edge."""
        return self.__flows

    def is_empty(self):
        """Returns the number of registered inflows"""
        return not self.__flows


class DetectorParams:
    def __init__(self):
        """Instantiate Detectors."""
        self.__detectors = []
        self.__pending_detectors = []

    def add_induction_loop_detector(
        self,
        name,
        lane_id,
        position,
        frequency,
        storage_file='out.xml',
        friendly_position=False):
        """
        Adds a single induction loop detector.

        Args:
            name (str): Detector ID.
            lane_id (str): Lane ID.
            position (int): Position on lane. Negative values denotes position from the opposite end of the lane.
            frequency (int): Frequence of data collection.
            storage_file (str, optional): Storage of detector data. Defaults to 'out.xml'.
            friendly_position (bool, optional): Defaults to False.
        """
        detector = {
            'id': name,
            'lane': lane_id,
            'pos': str(position),
            'freq': str(frequency),
            'file': storage_file,
            'friendlyPos': str(friendly_position).lower(),
            'type': 'inductionLoop',
        }
        self.__detectors.append(detector)

    def add_induction_loop_detectors_to_intersection(self, *args, **kwargs):
        """
        These are added to pending detectors list as when this method is called only node_id, position are available.
        But to create detectors, lane_id is also required which can be acquired only when the network object is available.
        """
        detector_params = {'args': args, **kwargs}
        self.__pending_detectors.append(detector_params)

    def _add_induction_loop_detectors_to_intersection(
        self,
        name,
        node_id,
        positions,
        frequency,
        network=None,
        storage_file='out.xml',
        friendly_position=False,
    ):
        assert network is not None, 'Network cannot be None.'

        connections = network.connections[node_id]
        edge_id_idx_mapping = [edge['id'] for edge in network.edges]
        incoming_lanes = list(set([f"{c['from']}_{c['fromLane']}" for c in connections]))
        incoming_lanes.sort()
        counter = 0

        for lane_id in incoming_lanes:
            lane_length = network.edges[edge_id_idx_mapping.index(lane_id.rsplit('_', 1)[0])]['length']
            for position in positions:
                if abs(position) < int(lane_length):
                    detector = {
                        'id': f'{name}_{counter}',
                        'lane': lane_id,
                        'pos': str(position),
                        'freq': str(frequency),
                        'file': storage_file,
                        'friendlyPos': str(friendly_position).lower(),
                        'type': 'inductionLoop',
                    }
                    counter += 1
                    self.__detectors.append(detector)

    def add_lane_area_detector(self, det_params):
        raise NotImplementedError

    def get(self, network=None):
        """Return all the detectors."""
        # create pending detectors
        for detector_params in self.__pending_detectors:
            args = detector_params['args']
            del detector_params['args']
            self._add_induction_loop_detectors_to_intersection(
                *args, **detector_params, network=network
            )
        self.__pending_detectors = []
        return self.__detectors

#TODO: Add Aimsun detector parameters