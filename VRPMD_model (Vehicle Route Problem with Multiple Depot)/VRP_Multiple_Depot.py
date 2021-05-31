
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['time_matrix'] = [
        [0, 6, 7, 9, 7, 3, 6, 2, 3, 2, 6, 6, 4, 4, 5, 9, 7, 0],
        [6, 0, 8, 3, 2, 6, 8, 4, 8, 8, 13, 7, 5, 8, 12, 12, 14, 6],
        [7, 8, 0, 11, 10, 6, 3, 9, 5, 8, 4, 15, 14, 13, 9, 18, 9, 15],
        [9, 3, 11, 0, 3, 7, 10, 6, 10, 10, 14, 6, 7, 9, 14, 6, 16, 14],
        [7, 2, 10, 3, 0, 6, 9, 4, 8, 9, 13, 4, 6, 8, 12, 8, 14, 9],
        [3, 6, 6, 7, 6, 0, 2, 8, 2, 2, 7, 9, 7, 7, 6, 12, 8, 3],
        [6, 8, 3, 10, 9, 2, 0, 6, 2, 5, 4, 12, 10, 10, 6, 11, 5, 10],
        [2, 4, 9, 6, 4, 8, 6, 0, 4, 4, 8, 5, 4, 13, 7, 8, 10, 12],
        [3, 8, 5, 10, 8, 2, 2, 4, 0, 3, 4, 9, 8, 7, 3, 13, 6, 5],
        [2, 8, 8, 10, 9, 2, 5, 4, 3, 0, 4, 6, 5, 4, 3, 9, 5, 8],
        [6, 13, 4, 14, 13, 7, 4, 8, 4, 4, 0, 10, 9, 8, 4, 13, 4, 9],
        [6, 7, 15, 6, 4, 9, 12, 5, 9, 6, 10, 0, 1, 3, 7, 13, 10, 11],
        [4, 5, 14, 7, 6, 7, 10, 4, 8, 5, 9, 1, 0, 2, 16, 4, 8, 1],
        [4, 8, 13, 9, 8, 7, 10, 13, 7, 4, 8, 3, 2, 0, 4, 5, 6, 2],
        [5, 12, 9, 14, 12, 6, 6, 7, 3, 3, 4, 7, 16, 4, 0, 9, 12, 4],
        [9, 12, 18, 6, 8, 12, 11, 8, 13, 9, 13, 13, 4, 5, 9, 0, 9, 10],
        [7, 14, 9, 16, 14, 8, 5, 10, 6, 5, 4, 10, 8, 6, 12, 9, 0, 13],
        [0, 6, 15, 14, 9, 3, 10, 12, 5, 8, 9, 11, 1, 2, 4, 10, 13, 0]
    ]
    data['time_windows'] = [
        (0, 22),  # depot
        (7, 12),  # 1
        (10, 15),  # 2
        (6, 8),  # 3
        (10, 13),  # 4
        (0, 5),  # 5
        (5, 10),  # 6
        (0, 4),  # 7
        (5, 7),  # 8
        (0, 3),  # 9
        (10, 16),  # 10
        (10, 15),  # 11
        (0, 9),  # 12
        (5, 10),  # 13
        (7, 10),  # 14
        (10, 15),  # 15
        (11, 15),  # 16
        (18, 25)  # 17
    ]
    data['num_days'] = 4
    data['start'] = [0, 0, 0, 0]  # ,17,0,17]
    data['end'] = [17, 16, 17, 17]
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    time_dimension = routing.GetDimensionOrDie('time')
    max_route_distance = 0
    for vehicle_id in range(data['num_days']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            plan_output += ' {0} ({1}-{2})'.format(manager.IndexToNode(index),
                        solution.Min(time_var),
                        solution.Max(time_var))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{0} ({1}-{2}) \n'.format(manager.IndexToNode(index),
                        solution.Min(time_var),
                        solution.Max(time_var))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print('Maximum of the route distances: {}m'.format(max_route_distance))


def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_days'], data['start'], data['end'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add time constraint.
    dimension_name = 'time'
    routing.AddDimension(
        transit_callback_index,
        20,  # no slack
        300,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    time_dimension = routing.GetDimensionOrDie(dimension_name)
    # add time window constraints

    time_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)


if __name__ == '__main__':
    main()