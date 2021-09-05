from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math
import numpy as np
import matplotlib.pyplot as plt


def create_data_model():
    data = {}
    fuel_capacity = 30000   # fuel_capacity in -ft
    _locations = [
        (2.5, 2.5),  # start
        (2.3425, 2.025),  
        (2.185, 1.55),
        (2.0275, 1.075),
        (1.87, 0.6),
        (1.6175, 1.265),
        (1.365, 1.93),
        (1.1125, 2.595),
        (0.86, 3.26),
        (1.685, 3.2975),
        (2.51, 3.335),
        (3.335, 3.3725),
        (4.16, 3.41),   # end
        (1.3, 1.1),  # locations to visit
        (0.6, 4.2), (4.6, 4.0),
        (1.3, 4.2), (4.3, 0.5), (4.8, 1.3),
        (0.6, 0.2), (5, 2.1), (3.1, 1.3), (2.0, 2.5),
        (3.2, 3.9), (4.1, 4.7), (3.3, 0.6), (3.3, 4.5), (0.7, 2.8),
        (4.1, 2.6), (1.2, 1.0), (0.7, 3.2), (0.3, 0.3), (4.3, 4.7), (2, 0),
        (0.2, 1.7), (0.5, 4.2), (0.7, 0.4), (4, 2.9)]
    data["coordinates"] = _locations

    data["locations"] = [(l[0] * 5280, l[1] * 5280) for l in _locations]   # 26.4k x 26.4k sq. ft. (5 x 5 miles sq.)
    data["num_locations"] = len(data["locations"])
    data["time_windows"] = [(0, 60),  # veh_start_node
                            (1860, 1960),  # recharge_station_1
                            (3660, 3760),
                            (5460, 5560),
                            (7260, 7360),
                            (9060, 9160),
                            (10860, 10960),
                            (12660, 12760),
                            (14460, 14560),
                            (16260, 16360),
                            (18060, 18160),
                            (19860, 19960),     # recharge station 11
                            (21660, 21760),
                            (0, 100000),
                            (7, 100000),
                            (10, 100000),
                            (14, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000),
                            (0, 100000)]
    data["num_vehicles"] = 2
    data["fuel_capacity"] = fuel_capacity
    data["vehicle_speed"] = 33  # ft/s
    data["starts"] = [0, 0]
    data["ends"] = [12, 12]
    distance_matrix = np.zeros((data["num_locations"], data["num_locations"]), dtype=int)
    for i in range(data["num_locations"]):
        for j in range(data["num_locations"]):
            if i == j:
                distance_matrix[i][j] = 0
            else:
                distance_matrix[i][j] = euclidean_distance(data["locations"][i], data["locations"][j])
    dist_matrix = distance_matrix.tolist()
    # print(dist_matrix)
    data["distance_matrix"] = dist_matrix
    assert len(data['distance_matrix']) == len(data['locations'])
    assert len(data['distance_matrix']) == len(data['time_windows'])
    assert len(data['starts']) == len(data['ends'])
    assert data['num_vehicles'] == len(data['starts'])
    return data


def euclidean_distance(position_1, position_2):
    return round(math.hypot((position_1[0] - position_2[0]), (position_1[1] - position_2[1])))


def print_solution(data, manager, routing, solution):
    print("Objective: {}".format(solution.ObjectiveValue()))
    total_distance = 0
    total_load = 0
    total_time = 0
    distance_dimension = routing.GetDimensionOrDie("Distance")
    fuel_dimension = routing.GetDimensionOrDie("Fuel")
    time_dimension = routing.GetDimensionOrDie("Time")
    dropped_nodes = "Dropped nodes:"
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(node)) == node:
            dropped_nodes += " {}".format(manager.IndexToNode(node))
    print(dropped_nodes)
    dum_list = [(data["locations"][0][0], data["locations"][0][1])]
    dum_list2 = [(data["locations"][0][0], data["locations"][0][1])]
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = "Route for vehicle {}:\n".format(vehicle_id)
        distance = 0
        while not routing.IsEnd(index):
            fuel_var = fuel_dimension.CumulVar(index)
            time_var = time_dimension.CumulVar(index)
            slack_var = time_dimension.SlackVar(index)
            # dist_var = distance_dimension.CumulVar(index)
            plan_output += "{0} Fuel({1}) Time({2},{3}) Slack({4},{5}) -> ".format(
                manager.IndexToNode(index),
                solution.Value(fuel_var),
                solution.Min(time_var),
                solution.Max(time_var),
                solution.Min(slack_var),
                solution.Max(slack_var))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            if vehicle_id == 0:
                if index <= 36:
                    if index == 1 or index == 2 or index == 3 or index == 4 or index == 5 or index == 6 or index == 7 \
                            or index == 8 or index == 9 or index == 10 or index == 11:
                        dum_list.append(data["locations"][index])
                    else:
                        dum_list.append(data["locations"][index + 1])
            else:
                if index <= 36:
                    if index == 1 or index == 2 or index == 3 or index == 4 or index == 5 or index == 6 or index == 7 \
                            or index == 8 or index == 9 or index == 10 or index == 11:
                        dum_list2.append(data["locations"][index])
                    else:
                        dum_list2.append(data["locations"][index + 1])
            print(f"plop {routing.GetArcCostForVehicle(previous_index, index, vehicle_id)}\n")
            distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        # dist_var = distance_dimension.CumulVar(index)
        fuel_var = fuel_dimension.CumulVar(index)
        time_var = time_dimension.CumulVar(index)
        plan_output += "{0} Fuel({1}) Time({2},{3}) \n".format(
            manager.IndexToNode(index),
            solution.Value(fuel_var),
            solution.Min(time_var),
            solution.Max(time_var))
        plan_output += "Distance of the route: {} ft\n".format(distance)
        plan_output += "Remaining Fuel of the route: {}\n".format(solution.Value(fuel_var))
        plan_output += "Total Time of the route: {} seconds\n".format(solution.Value(time_var))
        print(plan_output)
        total_distance += distance
        total_load += solution.Value(fuel_var)
        total_time += solution.Value(time_var)
    print('Total Distance of all routes: {} ft'.format(total_distance))
    print('Total Fuel remaining of all routes: {}'.format(total_load))
    print('Total Time of all routes: {} seconds'.format(total_time))

    dum_list.append((data["locations"][12][0], data["locations"][12][1]))
    dum_list2.append((data["locations"][12][0], data["locations"][12][1]))
    x1 = []
    y1 = []
    x2 = []
    y2 = []
    for i in dum_list:
        x1.append(i[0])
        y1.append(i[1])
    for j in dum_list2:
        x2.append(j[0])
        y2.append(j[1])
    fig = plt.figure()
    ax = fig.add_subplot(111)
    for i in range(len(data["locations"])):
        if i <= 12:
            ax.plot(data["locations"][i][0], data["locations"][i][1], 'k.', markersize=10)
        else:
            ax.plot(data["locations"][i][0], data["locations"][i][1], 'kx', markersize=10)

    for i in range(len(x1)):
        if i <= len(x1) - 2:
            ax.arrow(x1[i], y1[i], (x1[i+1] - x1[i]), (y1[i+1] - y1[i]), width=100, color='red')
            # patches.FancyArrowPatch((x1[i], y1[i]), ((x1[i+1] - x1[i]), (y1[i+1] - y1[i])), arrowstyle='<->', mutation_scale=20)
    for j in range(len(x2)):
        if j <= len(x2) - 2:
            ax.arrow(x2[j], y2[j], (x2[j+1] - x2[j]), (y2[j+1] - y2[j]), width=100, color='green')
            # patches.FancyArrowPatch((x1[j], y1[j]), ((x1[j+1] - x1[j]), (y1[j+1] - y1[j])), arrowstyle='<->', mutation_scale=20)
    # else:
    #     ax.arrow(x1[6], y1[6], (x1[0] - x1[6]), (y1[0] - y1[6]), width=2)
    plt.savefig('UAV=2_k3.pdf')
    plt.show()


def main():
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]),
        data["num_vehicles"],
        data["starts"],
        data["ends"])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Distance
    stations = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]  # depot + refill stations

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        if from_node in stations and to_node in stations:
            return data["fuel_capacity"]*5
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        data["fuel_capacity"]*5,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Fuel constraints
    def fuel_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return -data["distance_matrix"][from_node][to_node]

    fuel_callback_index = routing.RegisterTransitCallback(fuel_callback)
    routing.AddDimension(
        fuel_callback_index,
        data["fuel_capacity"],
        data["fuel_capacity"],
        False,
        'Fuel')

    penalty = 0
    fuel_dimension = routing.GetDimensionOrDie('Fuel')
    for vehicle_id in range(data["num_vehicles"]):
        fuel_dimension.SlackVar(routing.Start(vehicle_id)).SetValue(0)
        for node in range(len(data["distance_matrix"])):
            if node == 0 or node == 12:
                continue
            if node > 12:
                index = manager.NodeToIndex(node)
                fuel_dimension.SlackVar(index).SetValue(0)
                routing.AddVariableMinimizedByFinalizer(fuel_dimension.CumulVar(node))
            else:
                index = manager.NodeToIndex(node)
                routing.AddDisjunction([index], penalty)

    # Time
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        if from_node in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27,
                         28, 29, 30, 31, 32, 33, 34, 35, 36, 37]:
            return 600 + int(data["distance_matrix"][from_node][to_node] / data["vehicle_speed"])
        else:
            return 600 + int(data["distance_matrix"][from_node][to_node] / data["vehicle_speed"])

    time_callback_index = routing.RegisterTransitCallback(time_callback)
    # routing.SetArcCostEvaluatorOfAllVehicles(time_callback_index)
    routing.AddDimension(
        time_callback_index,
        300,
        50000,
        False,
        'Time')

    time_dimension = routing.GetDimensionOrDie('Time')
    time_dimension.SetGlobalSpanCostCoefficient(100)
    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx == 0 or location_idx == 12:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        routing.AddToAssignment(time_dimension.SlackVar(index))
    # Add time window constraints for each vehicle start node
    # and "copy" the slack var in the solution object (aka Assignment) to print it
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data["time_windows"][0][0], data["time_windows"][0][1])
        routing.AddToAssignment(time_dimension.SlackVar(index))

    # for i in range(len(data["distance_matrix"])):
    #     if i > 5:
    #         routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(manager.NodeToIndex(i)))
    #         routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(manager.NodeToIndex(i)))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(60)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print("No solution found!")

    print("Solver status:", routing.status())


if __name__ == '__main__':
    main()
