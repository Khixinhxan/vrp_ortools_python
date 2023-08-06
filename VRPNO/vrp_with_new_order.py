#!/usr/bin/env python3
"""Or-tool discussion on Github
https://github.com/google/or-tools/discussions/2425
"""

"""Capacited Vehicles Routing Problem (CVRP).
Here we simulate a vehicle already loaded with 4 orders and on its way to deliver them
When a new order (node 5 -> 6) arrives.
"""
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def print_solution(manager, routing, solution):
    """Prints solution on console."""
    #objective
    print(f'Objective: {solution.ObjectiveValue()}')
    # Display dropped nodes.
    dropped_nodes = 'Dropped nodes:'
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(node)) == node:
            dropped_nodes += f'{manager.IndexToNode(node)} '
    print(dropped_nodes)
    # Display routes
    capacity_dimension = routing.GetDimensionOrDie('Capacity')
    for vehicle_id in range(manager.GetNumberOfVehicles()):
        index = routing.Start(vehicle_id)
        plan_output = f'Route for vehicle {vehicle_id}:\n'
        route_distance = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            load_var = capacity_dimension.CumulVar(index)
            route_load = solution.Value(load_var)
            plan_output += f' {node_index} Load({route_load}) -> '
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        node_index = manager.IndexToNode(index)
        load_var = capacity_dimension.CumulVar(index)
        route_load = solution.Value(load_var)
        plan_output += f' {node_index} Load({route_load})\n'
        plan_output += f'Distance of the route: {route_distance}m\n'
        print(plan_output)


def main():
    """Solve the CVRP problem."""
    distance_matrix = [
            [ 1, 1, 1, 1, 1, 1, 1 ],
            [ 1, 1, 1, 1, 1, 1, 1 ],
            [ 1, 1, 1, 1, 1, 1, 1 ],
            [ 1, 1, 1, 1, 1, 1, 1 ],
            [ 1, 1, 1, 1, 1, 1, 1 ],
            [ 1, 1, 1, 1, 1, 1, 1 ],
            [ 1, 1, 1, 1, 1, 1, 1 ],
            ]

    demands = [
            0, # Start position
            -1, # order 1
            -1, # order 2
            -1, # order 3
            -1, # order 4
            1, # new order pickup location
            -1 # new order delivery location
            ]

    assert len(distance_matrix) == len(demands)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
            len(demands), # number of locations (start, order, depots)
            1, # number of vehicle
            0) # start location idx

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    distance = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        100,  # vehicle maximum travel distance
        True,  # start cumul to zero
        distance)
    distance_dimension = routing.GetDimensionOrDie(distance)
    #distance_dimension.SetGlobalSpanCostCoefficient(100)

    #------------------- CAPACITY ----------

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)

    capacity = 'Capacity'
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        [8],  # vehicle maximum capacities
        False,  # start cumul to zero
        capacity)
    capacity_dimension = routing.GetDimensionOrDie(capacity)

    # 4 Orders already loaded
    index = routing.Start(0)
    capacity_dimension.CumulVar(index).SetValue(4)

    # Add new pickup and delivery order (5 -> 6)
    pickup_index = manager.NodeToIndex(5)
    delivery_index = manager.NodeToIndex(6)
    routing.AddPickupAndDelivery(pickup_index, delivery_index)
    routing.solver().Add(
        routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index))
    routing.solver().Add(
        distance_dimension.CumulVar(pickup_index) <=
        distance_dimension.CumulVar(delivery_index))
    routing.AddDisjunction(
            [5,6],
            100_000, # high penalty so solver has strong incentive to add it
            2) # max cardinality

    # Initial route (Without start and end)
    # notice new pickup and delivery are dropped
    initial_routes = [[1, 2, 3, 4]]
    
    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)
    #search_parameters.first_solution_strategy = (
    #    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(2)
    # search_parameters.log_search = True
    
    routing.CloseModelWithParameters(search_parameters)
    # ReadAssignmentFromRoutes will close the model...
    initial_solution = routing.ReadAssignmentFromRoutes(initial_routes, True)
    print('Initial solution:')
    print_solution(manager, routing, initial_solution)

    # Solve the problem.
    solution = routing.SolveFromAssignmentWithParameters(initial_solution, search_parameters)
    #solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(manager, routing, solution)
    else:
      print(solution)


if __name__ == '__main__':
    main()