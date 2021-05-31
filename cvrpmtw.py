#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May 27 11:41:27 2020

@author: ubuntu
"""

from ortools.constraint_solver.pywrapcp import RoutingIndexManager, \
    RoutingModel, DefaultRoutingModelParameters, \
    DefaultRoutingSearchParameters, BOOL_FALSE
from ortools.constraint_solver.routing_enums_pb2 import FirstSolutionStrategy

is_debug = False


class CreateDistanceCallback(object):
    """Create callback to calculate distances and travel times between points."""

    def __init__(self, distance_matrix):
        """Initialize distance array."""
        self.matrix = distance_matrix

    def Distance(self, from_node, to_node):
        return self.matrix[from_node][to_node]


class CreateDemandCallback(object):
    """Create callback to get demands at location node."""

    def __init__(self, demands):
        self.matrix = demands

    def Demand(self, from_node, to_node):
        return self.matrix[from_node]


# Service time (proportional to demand) callback.
class CreateServiceTimeCallback(object):
    """Create callback to get time windows at each location."""

    def __init__(self, time_in_locations):
        self.time_in_locations = time_in_locations

    def ServiceTime(self, from_node, to_node):
        return self.time_in_locations[from_node]


# Create total_time callback (equals service time plus travel time).
class CreateTotalTimeCallback(object):
    def __init__(self, service_time_callback, dist_callback, speed):
        self.service_time_callback = service_time_callback
        self.dist_callback = dist_callback
        self.speed = speed

    def TotalTime(self, from_node, to_node):
        service_time = self.service_time_callback(from_node, to_node)
        travel_time = self.dist_callback(from_node, to_node) / self.speed
        return service_time + travel_time


UNKNOWN_EXCEPTION = 1
ZERO_LOCATIONS_CODE = 1000
NO_SOLUTION = 1001
OVERLAPPING_TIME_WINDOWS = 1002


def main():
#    debug('Starting script')

    # TODO: Validate json structure
#    global is_debug
#    is_debug = True if args.debug else False
#    debug(args.data)
#    input_data = json.loads(args.data)
    input_data = {"locations":[{
            "time_windows":[],
            "service_time":0},
            {"time_windows":[],
             "service_time":0},
             {"time_windows":[[0,10800]],
              "service_time":900},
              {"time_windows":[[0,10800]],
               "service_time":900},
               {"time_windows":[[21000,46800]],
                "service_time":900},
                {"time_windows":[[18000,46800]],
                 "service_time":900}],
            "distance_matrix":[[0,0,2553,672,3561,2372],
                               [0,0,2553,672,3561,2372],
                               [2583,2583,0,2288,2063,741],
                               [812,812,2410,0,3418,2229],
                               [3585,3585,1906,3290,0,1756],
                               [2452,2452,586,2157,1732,0]],
            "start_location_index":0,
             "end_location_index":1}
    locations = input_data["locations"]
    distance_matrix = input_data["distance_matrix"]
    num_locations = len(input_data["locations"])

    start_location = input_data["start_location_index"] if "start_location_index" in input_data else 0
    end_location = input_data["end_location_index"] if "end_location_index" in input_data else 1
    num_vehicles = 1
    speed = 1
    fix_start_cumul_to_zero = True
    search_time_limit = 8 * 1000
    max_time_window = 24 * 3600


    # The number of nodes of the VRP is num_locations.
    # Nodes are indexed from 0 to num_locations - 1. By default the start of
    # a route is node 0.
    print(num_locations)
    print(num_vehicles)
    print([start_location])
    print([end_location])
#    routing = RoutingIndexManager(num_locations, num_vehicles, 0, 0)
#    search_parameters = RoutingModel.DefaultSearchParameters()
#
#    search_parameters.time_limit_ms = search_time_limit
#
#    # Setting first solution heuristic: the method for finding a first solution to the problem.
#    search_parameters.first_solution_strategy = FirstSolutionStrategy.PATH_CHEAPEST_ARC
#    # The 'PATH_CHEAPEST_ARC' method does the following:
#    # Starting from a route "start" node, connect it to the node which produces the
#    # cheapest route segment, then extend the route by iterating on the last node added to the route.
#
#    dist_between_locations = CreateDistanceCallback(distance_matrix)
#    dist_callback = dist_between_locations.Distance
#    routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)

    # Adding time dimension constraints.
    time_in_locations = [l["service_time"] for l in locations]
#    service_times = CreateServiceTimeCallback(time_in_locations)
#    service_time_callback = service_times.ServiceTime
#
#    total_times = CreateTotalTimeCallback(service_time_callback, dist_callback, speed)
#    total_time_callback = total_times.TotalTime
#
#    # Add a dimension for time-window constraints and limits on the start times and end times.
#    time = "Time"
#    routing.AddDimension(total_time_callback,  # total time function callback
#                         max_time_window,
#                         max_time_window,
#                         fix_start_cumul_to_zero,
#                         time)
#
#    # Add limit on size of the time windows.
#    time_dimension = routing.GetDimensionOrDie(time)

    # Transform time windows to "not allowed time windows"
    for order in range(num_locations):
        time_windows = locations[order]["time_windows"]
        if len(time_windows) == 0:
            continue  # Skip empty time windows since it means they are available all the time
        time_windows = sorted(time_windows, key=lambda tw: tw[0])
        pairwise_tw = pairwise(time_windows)
        # Check for overlapping time windows
        for cur, next in pairwise_tw:
            if next is not None:
                cur_end, next_start = cur[1], next[0]
                if cur_end >= next_start:
                    print('Overlapping time windows are not allowed',
                                {"time_windows": [cur, next]})

        closed_time_windows = []
        cur_time = 0
        # Find available "not allowed time windows" between time windows in the interval [0, max_time_window]
        for start, end in time_windows:
            if start - cur_time > 1:
                closed_time_windows.append([cur_time, start - 1])
            cur_time = end + 1
        if max_time_window - cur_time > 1:
            closed_time_windows.append([cur_time, max_time_window])
            
        
    print(closed_time_windows)
    print([tw[0] for tw in closed_time_windows])
    print([tw[1] for tw in closed_time_windows])
#        # Add the not allowed time window constraints
#        solver = routing.solver()
#        solver.AddConstraint(
#            solver.NotMemberCt(
#                time_dimension.CumulVar(order),
#                [tw[0] for tw in closed_time_windows], [tw[1] for tw in closed_time_windows]
#            )
#        )

    # Solve displays a solution if any.
    assignment = routing.SolveWithParameters(search_parameters)


    # Solution cost.
    print("Total distance of all routes: %s" % assignment.ObjectiveValue())
    # Inspect solution.
    time_dimension = routing.GetDimensionOrDie(time)

    vehicle_nbr = 0
    index = routing.Start(vehicle_nbr)
    route = []

    while not routing.IsEnd(index):
        node_index = routing.IndexToNode(index)

        time_var = time_dimension.CumulVar(index)
        route.append({
            "index": node_index,
            "min_arrive_time": assignment.Min(time_var),
            "max_arrive_time": assignment.Max(time_var),
        })
        index = assignment.Value(routing.NextVar(index))

    node_index = routing.IndexToNode(index)
    time_var = time_dimension.CumulVar(index)
    route.append({
        "index": node_index,
        "min_arrive_time": assignment.Min(time_var),
        "max_arrive_time": assignment.Max(time_var),
    })

    stats = {
        "cost": assignment.ObjectiveValue()
    }
    result = {"is_error": False, "route": route, "stats": stats}
    print(result)


def pairwise(iterable):
    it = iter(iterable)
    a = next(it, None)

    for b in it:
        yield (a, b)
        a = b


if __name__ == '__main__':
    # try:
    main()
#    args = parse_args()
#    main(args)
    # except BaseException as err:
    #     error_and_exit(error(UNKNOWN_EXCEPTION, 'An unknown exception occurred', {"err": str(err)}))