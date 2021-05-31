#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 14 13:51:53 2020

@author: ubuntu
"""

import sys
from ortools.constraint_solver import pywrapcp, routing_enums_pb2


def add_durations(manager, routing, durations_matrix, num_locations, num_vehicles, horizon, time_windows):
    def duration_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return durations_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(duration_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    routing.AddDimension(
        transit_callback_index,
        horizon,  # allow waiting time
        horizon,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        "Duration"
    )
    duration_dimension = routing.GetDimensionOrDie('Duration')

    # Add time window constraints for each location except start/end locations
    for location_idx in range(num_locations):
        index = manager.NodeToIndex(location_idx)
        duration_dimension.CumulVar(index).SetRange(time_windows[location_idx][0], time_windows[location_idx][1])
        routing.AddToAssignment(duration_dimension.SlackVar(index))

    # Add time window constraints for each vehicle start and end locations
    for vehicle_id in range(num_vehicles):
        st_index = routing.Start(vehicle_id)
        end_index = routing.End(vehicle_id)
        duration_dimension.CumulVar(st_index).SetRange(time_windows[num_locations + vehicle_id][0],
                                                       time_windows[num_locations + vehicle_id][1])
        duration_dimension.CumulVar(end_index).SetRange(time_windows[num_locations + num_vehicles + vehicle_id][0],
                                                        time_windows[num_locations + num_vehicles + vehicle_id][1])

def print_solution(num_vehicles, manager, routing, assignment):
    """Prints assignment on console."""
    time_dimension = routing.GetDimensionOrDie('Duration')
    total_time = 0
    for vehicle_id in range(num_vehicles):
        index = routing.Start(vehicle_id)
        locations = -1
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        while not routing.IsEnd(index):
            locations += 1
            time_var = time_dimension.CumulVar(index)
            plan_output += '{0} Time({1},{2}) -> '.format(
                manager.IndexToNode(index), assignment.Min(time_var),
                assignment.Max(time_var))
            index = assignment.Value(routing.NextVar(index))
        time_var = time_dimension.CumulVar(index)
        plan_output += '{0} Time({1},{2})\n'.format(manager.IndexToNode(index),
                                                    assignment.Min(time_var),
                                                    assignment.Max(time_var))
        plan_output += 'Locations visited: {}. Time of the route: {}s\n'.format(locations, assignment.Min(time_var))
        print(plan_output)
        total_time += assignment.Min(time_var)
    print('Total time of all routes: {}s'.format(total_time))


def run_vrp(num_vehicles, num_locations, vehicles_fixed_cost):
    m = sys.maxsize

    durations_matrix = [
        [0, m, 6410, 5314, 5100, 5100, m, m, m, m, m, m, 3600, 3600, 3600, 3600, 3600, 3600],
        [m, 0, m, m, m, 5100, m, m, m, m, m, m, 3600, 3600, 3600, 3600, 3600, 3600],
        [m, m, 0, m, m, 10728, m, m, m, m, m, m, 5425, 5425, 5425, 5425, 5425, 5425],
        [m, m, m, 0, m, 3990, m, m, m, m, m, m, 1658, 1658, 1658, 1658, 1658, 1658],
        [m, m, m, m, 0, 5100, m, m, m, m, m, m, 3600, 3600, 3600, 3600, 3600, 3600],
        [m, m, m, m, m, m, m, m, m, m, m, m, 3600, 3600, 3600, 3600, 3600, 3600],
        [0, 0, 0, 0, 0, 0, m, m, m, m, m, m, 0, m, m, m, m, m],
        [0, 0, 0, 0, 0, 0, m, m, m, m, m, m, m, 0, m, m, m, m],
        [0, 0, 0, 0, 0, 0, m, m, m, m, m, m, m, m, 0, m, m, m],
        [0, 0, 0, 0, 0, 0, m, m, m, m, m, m, m, m, m, 0, m, m],
        [0, 0, 0, 0, 0, 0, m, m, m, m, m, m, m, m, m, m, 0, m],
        [0, 0, 0, 0, 0, 0, m, m, m, m, m, m, m, m, m, m, m, 0],
        [m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m],
        [m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m],
        [m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m],
        [m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m],
        [m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m],
        [m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m, m]
    ]

    time_windows = [
        [18028, 18448], [22558, 22978], [23400, 23820], [27000, 27420], [27010, 27430], [31536, 31956], 
        [120, 36119], [120, 36119], [3720, 39719], [14520, 50519], [21720, 57719], [25320, 61319], 
        [121, 36120], [121, 36120], [3721, 39720], [14521, 50520], [21721, 57720], [25321, 61320]
    ]
    
    horizon = 62000
    
    """Solve the VRP with time windows."""
    manager = pywrapcp.RoutingIndexManager(
        len(durations_matrix),
        num_vehicles, 
        list(range(num_locations, num_locations + num_vehicles)), 
        list(range(num_locations + num_vehicles, num_locations + num_vehicles + num_vehicles))
    )

    routing = pywrapcp.RoutingModel(manager)

    add_durations(manager, routing, durations_matrix, num_locations, num_vehicles, horizon, time_windows)

    # seting a fixed cost per each vehicle
    routing.SetFixedCostOfAllVehicles(vehicles_fixed_cost)

    penalty = horizon * (len(durations_matrix) ** 2)
    for location in range(num_locations):
        routing.AddDisjunction([manager.NodeToIndex(location)], penalty)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION

    assignment = routing.SolveWithParameters(search_parameters)

    if assignment:
        print_solution(num_vehicles, manager, routing, assignment)
    else:
        print("Assignment is None")

    return assignment


if __name__ == '__main__':
    # 0 fixed cost per vehicle - only 5 locations are visited
    run_vrp(num_locations=6, num_vehicles=6, vehicles_fixed_cost=0)

    # now, each vehicle has a fixed cost of 1500 - all 6 locations are visited with leass vehicles...
    run_vrp(num_locations=6, num_vehicles=6, vehicles_fixed_cost=1500)