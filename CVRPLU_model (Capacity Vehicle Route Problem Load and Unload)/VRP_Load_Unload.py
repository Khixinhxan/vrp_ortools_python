
import math
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import numpy as np


###########################fde
# Problem Data Definition #
###########################
def create_data_model():
    """Stores the data for the problem"""
    data = {}

    data['time_windows'] = [
        (0, 0), (0, 8000), (0, 8000), (0, 8000),  # 0,1,2,3
        (0, 8000),
        (0, 8000),
        (0, 8000),
        (0, 8000)]
    data['demands'] = [0, -50, -50, -50, 40, 10, 20, 30]
    data['demands_w'] = [0, -10, -10, -10, 3, 4, 6, 5]
    data['demands_w'] = [0, -50, -50, -50, 40, 10, 20, 30]
    data['demands_h'] = [0, -50, -50, -50, 40, 10, 20, 30]
    data['demands_l'] = [0, -50, -50, -50, 40, 10, 20, 30]
    # data['demands'] = [0, -50, -50, -50, 50, 50, 50, 50]
    # # data['demands_w'] = [0, -10, -10, -10, 3, 4, 6, 5]
    # data['demands_w'] = [0, -10, -10, -10, 8, 2, 4, 6]
    # # data['demands_w'] = [0, -50, -50, -50, 40, 10, 20, 30]
    # data['demands_h'] = [0, -10, -10, -10, 8, 2, 4, 6]
    # # data['demands_h'] = [0, -50, -50, -50, 40, 10, 20, 30]
    # data['demands_l'] = [0, -10, -10, -10, 8, 2, 4, 6]
    # # data['demands_l'] = [0, -50, -50, -50, 40, 10, 20, 30]

    data['locations'] = [
        (51.14, 71.44), (51.16, 71.46), (51.17, 71.47), (51.14, 71.44),  # 0,1,2,3
        (51.14, 71.45),  # 4
        (51.1467, 71.4583),  # 5
        (51.1053, 71.4404),  # 6
        (51.14, 71.42)]  # 7
    data['vehicle_capacity'] = 50
    data['vehicle_capacity_w'] = 10
    # data['vehicle_capacity_w'] = 50
    data['vehicle_capacity_h'] = 10
    # data['vehicle_capacity_h'] = 50
    data['vehicle_capacity_l'] = 10
    # data['vehicle_capacity_l'] = 50
    data['num_locations'] = 8
    data['time_per_demand_unit'] = 5
    data['num_vehicles'] = 1
    data['depot'] = 0
    data['number_of_depots'] = 4
    data['vehicle_speed'] = 83.33333333333333

    return data


#######################
# Problem Constraints #
#######################

def gps_distance(position_1, position_2):
    lat1 = position_1[0]
    lat2 = position_2[0]
    lon1 = position_1[1]
    lon2 = position_2[1]

    R = 6378.137;  # Radius of earth in KM
    dLat = lat2 * math.pi / 180 - lat1 * math.pi / 180;
    dLon = lon2 * math.pi / 180 - lon1 * math.pi / 180;
    a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) * math.sin(dLon / 2) * math.sin(dLon / 2);
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a));
    d = R * c * 1000;  # in meters
    return d


def create_distance_evaluator(data):
    """Creates callback to return distance between points."""
    _distances = {}
    # precompute distance between location to have distance callback in O(1)
    for from_node in xrange(data["num_locations"]):
        _distances[from_node] = {}
        for to_node in xrange(data["num_locations"]):
            if from_node == to_node:
                _distances[from_node][to_node] = 0
            else:
                _distances[from_node][to_node] = (
                    gps_distance(data["locations"][from_node],
                                 data["locations"][to_node]))

    def distance_evaluator(from_node, to_node):
        """Returns the manhattan distance between the two nodes"""
        return _distances[from_node][to_node]

    return distance_evaluator


def create_demand_evaluator_dwhl(data, ind):
    """Creates callback to get demands at each location."""
    if ind == 0:
        _demands = data["demands"]
    elif ind == 1:
        _demands = data["demands_w"]
    elif ind == 2:
        _demands = data["demands_h"]
    elif ind == 3:
        _demands = data["demands_l"]

    def demand_evaluator(from_node, to_node):
        """Returns the demand of the current node"""
        del to_node
        return _demands[from_node]

    print('Demand_{}: {}'.format(ind, _demands))
    return demand_evaluator


def add_capacity_constraints(routing, data, demand_evaluator, ind):
    """Adds capacity constraint"""
    if ind == 0:
        _vehicle_capacity = data["vehicle_capacity"]
    elif ind == 1:
        _vehicle_capacity = data["vehicle_capacity_w"]
    elif ind == 2:
        _vehicle_capacity = data["vehicle_capacity_h"]
    elif ind == 3:
        _vehicle_capacity = data["vehicle_capacity_l"]
    capacity = 'Capacity_{}'.format(ind)
    print('Vehicle Capacity_{}: {}'.format(ind, _vehicle_capacity))
    routing.AddDimension(
        demand_evaluator,
        _vehicle_capacity,  # Null slack
        _vehicle_capacity,
        True,  # start cumul to zero
        capacity)

    # Add Slack for reseting to zero unload depot nodes.
    # e.g. vehicle with load 10/15 arrives at node 1 (depot unload)
    # so we have CumulVar = 10(current load) + -15(unload) + 5(slack) = 0.
    capacity_dimension = routing.GetDimensionOrDie(capacity)
    # print('CapacityDimension: {}'.format(capacity_dimension))
    for node_index in [1, 2, 3]:
        index = routing.NodeToIndex(node_index)
        capacity_dimension.SlackVar(index).SetRange(0, _vehicle_capacity)
    for node_index in [4, 5, 6, 7]:
        index = routing.NodeToIndex(node_index)
        capacity_dimension.SlackVar(index).SetRange(0, 0)


def add_disjunction(routing, data):
    dodisjoint = 0
    if dodisjoint:
        for node_index in [1, 2, 3]:
            print('Depot NodeIndex: {}'.format(node_index))
            routing.AddDisjunction([node_index], 0)
        penalty = 1000000
        for node_index in xrange(4, data["num_locations"]):
            print('Location NodeIndex: {}'.format(node_index))
            routing.AddDisjunction([node_index], penalty)


def create_time_evaluator(data):
    """Creates callback to get total times between locations."""

    def service_time(data, node):
        """Gets the service time for the specified location."""
        if data["demands"][node] < 0:
            return 0
        return data["demands"][node] * data["time_per_demand_unit"]

    def travel_time(data, from_node, to_node):
        """Gets the travel times between two locations."""
        if from_node == to_node:
            travel_time = 0
        else:
            travel_time = gps_distance(
                data["locations"][from_node],
                data["locations"][to_node]) / data["vehicle_speed"]
        return travel_time

    _total_time = {}
    # precompute total time to have time callback in O(1)
    for from_node in xrange(data["num_locations"]):
        _total_time[from_node] = {}
        for to_node in xrange(data["num_locations"]):
            if from_node == to_node:
                _total_time[from_node][to_node] = 0
            else:
                _total_time[from_node][to_node] = int(
                    service_time(data, from_node) +
                    travel_time(data, from_node, to_node))

    def time_evaluator(from_node, to_node):
        """Returns the total time between the two nodes"""
        return _total_time[from_node][to_node]

    return time_evaluator


def add_time_window_constraints(routing, data, time_evaluator):
    """Add Time windows constraint"""
    time = 'Time'
    horizon = 8000  # total
    routing.AddDimension(
        time_evaluator,
        horizon,  # allow waiting time
        horizon,  # maximum time per vehicle
        False,  # don't force start cumul to zero since we are giving TW to start nodes
        time)
    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location except depot
    # and "copy" the slack var in the solution object (aka Assignment) to print it
    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx == 0:
            continue
        index = routing.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        routing.AddToAssignment(time_dimension.SlackVar(index))
    # Add time window constraints for each vehicle start node
    # and "copy" the slack var in the solution object (aka Assignment) to print it
    for vehicle_id in xrange(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data["time_windows"][0][0],
                                                data["time_windows"][0][1])
        routing.AddToAssignment(time_dimension.SlackVar(index))
        # Warning: Slack var is not defined for vehicle's end node
        # routing.AddToAssignment(time_dimension.SlackVar(self.routing.End(vehicle_id)))


###########
# Printer #
###########
def print_solution(data, routing, assignment):
    """Prints assignment on console"""
    print('---------------------------')
    print('Objective: {}'.format(assignment.ObjectiveValue()))
    total_distance = 0
    total_load = 0
    total_time = 0
    capacity_dimension = routing.GetDimensionOrDie('Capacity_0')
    time_dimension = routing.GetDimensionOrDie('Time')
    dropped = []
    for order in xrange(0, routing.nodes()):
        index = routing.NodeToIndex(order)
        if assignment.Value(routing.NextVar(index)) == index:
            dropped.append(order)
    print('dropped orders: {}'.format(dropped))

    for vehicle_id in xrange(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        distance = 0
        while not routing.IsEnd(index):
            load_var = capacity_dimension.CumulVar(index)
            time_var = time_dimension.CumulVar(index)
            plan_output += ' {0} Load({1}) Time({2},{3}) ->'.format(
                routing.IndexToNode(index),
                assignment.Value(load_var),
                assignment.Min(time_var),
                assignment.Max(time_var))
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            distance += routing.GetArcCostForVehicle(previous_index, index,
                                                     vehicle_id)
        load_var = capacity_dimension.CumulVar(index)
        time_var = time_dimension.CumulVar(index)
        plan_output += ' {0} Load({1}) Time({2},{3})\n'.format(
            routing.IndexToNode(index),
            assignment.Value(load_var),
            assignment.Min(time_var),
            assignment.Max(time_var))
        plan_output += 'Distance of the route: {}m\n'.format(distance)
        plan_output += 'Load of the route: {}\n'.format(assignment.Value(load_var))
        plan_output += 'Time of the route: {}\n'.format(assignment.Value(time_var))
        print(plan_output)
        total_distance += distance
        total_load += assignment.Value(load_var)
        total_time += assignment.Value(time_var)
    print('Total Distance of all routes: {}m'.format(total_distance))
    print('Total Load of all routes: {}'.format(total_load))
    print('Total Time of all routes: {}min'.format(total_time))


########
# Main #
########
def main():
    """Entry point of the program"""
    # Instantiate the data problem.
    data = create_data_model()

    # Create Routing Model
    # routing = pywrapcp.RoutingModel(
    #     data["num_locations"],
    #     data["num_vehicles"])

    # Create Routing Model
#    routing = pywrapcp.RoutingModel(
#        data["num_locations"],
#        data["num_vehicles"],
#        data["depot"])
    
    manager = pywrapcp.RoutingIndexManager(len(data['time_windows']),
                                           data['num_vehicles'], 0)
    # [END index_manager]

    # Create Routing Model.
    # [START routing_model]
    routing = pywrapcp.RoutingModel(manager)
    
    
    # Define weight of each edge
    distance_evaluator = create_distance_evaluator(data)
    routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
    # Add Capacity constraint
    demand_evaluator = create_demand_evaluator_dwhl(data, 0)
    add_capacity_constraints(routing, data, demand_evaluator, 0)

    demand_evaluator_w = create_demand_evaluator_dwhl(data, 1)
    add_capacity_constraints(routing, data, demand_evaluator_w, 1)
    demand_evaluator_h = create_demand_evaluator_dwhl(data, 2)
    add_capacity_constraints(routing, data, demand_evaluator_h, 2)
    demand_evaluator_l = create_demand_evaluator_dwhl(data, 3)
    add_capacity_constraints(routing, data, demand_evaluator_l, 3)

    add_disjunction(routing, data)
    # Add Time Window constraint
    time_evaluator = create_time_evaluator(data)
    add_time_window_constraints(routing, data, time_evaluator)

    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)  # pylint: disable=no-member
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    print_solution(data, routing, assignment)


if __name__ == '__main__':
    main()


