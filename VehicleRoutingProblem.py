# goal is to minimize the longest single route
# distances between locations are calculated using Manhattan distance -  |x1 - x2| + |y1 - y2|

# start imports

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# create sample data model


def createDataModel():
    data = {}
    # distance_matrix: An array of distances between locations on meters
    # num_locations: The number of locations
    # num_vehicles: The number of vehicles in the fleet
    # depot: The index of the depot, the location where all vehicles start and end their routes
    data["distanceMatrix"] = [
        [0, 548, 776, 696, 582, 274, 502, 194, 308,
            194, 536, 502, 388, 354, 468, 776, 662],
        [548, 0, 684, 308, 194, 502, 730, 354, 696,
            742, 1084, 594, 480, 674, 1016, 868, 1210],
        [776, 684, 0, 992, 878, 502, 274, 810, 468,
            742, 400, 1278, 1164, 1130, 788, 1552, 754],
        [696, 308, 992, 0, 114, 650, 878, 502, 844,
            890, 1232, 514, 628, 822, 1164, 560, 1358],
        [582, 194, 878, 114, 0, 536, 764, 388, 730,
            776, 1118, 400, 514, 708, 1050, 674, 1244],
        [274, 502, 502, 650, 536, 0, 228, 308, 194,
            240, 582, 776, 662, 628, 514, 1050, 708],
        [502, 730, 274, 878, 764, 228, 0, 536, 194,
            468, 354, 1004, 890, 856, 514, 1278, 480],
        [194, 354, 810, 502, 388, 308, 536, 0, 342,
            388, 730, 468, 354, 320, 662, 742, 856],
        [308, 696, 468, 844, 730, 194, 194, 342, 0,
            274, 388, 810, 696, 662, 320, 1084, 514],
        [194, 742, 742, 890, 776, 240, 468, 388, 274,
            0, 342, 536, 422, 388, 274, 810, 468],
        [536, 1084, 400, 1232, 1118, 582, 354, 730,
            388, 342, 0, 878, 764, 730, 388, 1152, 354],
        [502, 594, 1278, 514, 400, 776, 1004, 468,
            810, 536, 878, 0, 114, 308, 650, 274, 844],
        [388, 480, 1164, 628, 514, 662, 890, 354, 696,
            422, 764, 114, 0, 194, 536, 388, 730],
        [354, 674, 1130, 822, 708, 628, 856, 320, 662,
            388, 730, 308, 194, 0, 342, 422, 536],
        [468, 1016, 788, 1164, 1050, 514, 514, 662,
            320, 274, 388, 650, 536, 342, 0, 764, 194],
        [776, 868, 1552, 560, 674, 1050, 1278, 742,
            1084, 810, 1152, 274, 388, 422, 764, 0, 798],
        [662, 1210, 754, 1358, 1244, 708, 480, 856,
            514, 468, 354, 844, 730, 536, 194, 798, 0],
    ]
    data["numVehicles"] = 4
    data["depot"] = 0
    return data

# function displays the routes for the vehicles and the total distances of the routes


def printSolution(data, manager, routing, solution):
    print(f"Objective: {solution.ObjectiveValue()}")
    totalDistance = 0
    for Vid in range(data["numVehicles"]):
        index = routing.Start(Vid)
        planOutput = "Route for vehicle {}: \n".format(Vid)
        routeDistance = 0
        while not routing.IsEnd(index):
            planOutput += "{}->".format(manager.IndexToNode(index))
            previousIndex = index
            index = solution.Value(routing.NextVar(index))
            routeDistance += routing.GetArcCostForVehicle(
                previousIndex, index, Vid)
        planOutput += "{}\n".format(manager.IndexToNode(index))
        planOutput += "Distance of the route: {}m\n".format(routeDistance)
        print(planOutput)
        totalDistance += routeDistance
    print("Total distance of all routes: {}m".format(totalDistance))


def main():
    # Instantiating data
    data = createDataModel()
    # create routing index manager
    manager = pywrapcp.RoutingIndexManager(
        len(data["distanceMatrix"]), data["numVehicles"], data["depot"])
    # create routing model
    routing = pywrapcp.RoutingModel(manager)
    # create and register a transit callback
    # distance_callback: returns the distances between locations and passes it to the solver sets the arc costs to be the distances of the arcs

    def distanceCallback(fromIndex, toIndex):
        # convert from routing variable index to distance matrix NodeIndex
        fromNode = manager.IndexToNode(fromIndex)
        toNode = manager.IndexToNode(toIndex)
        return data["distanceMatrix"][fromNode][toNode]
    transitCallbackIndex = routing.RegisterTransitCallback(distanceCallback)
    # define cost of each arc
    routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex)
    # Add Distance constraint.
    dimension_name = 'Distance'
    # create a distance dimension, which computes the cumulative distance traveled by each vehicle along its route
    routing.AddDimension(
        transitCallbackIndex,
        0,  # no slack
        3000,  # vehicle maximum travel distance
        True,
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)
    # setting first solution heuristic
    searchParameters = pywrapcp.DefaultRoutingSearchParameters()
    searchParameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # solving problem
    solution = routing.SolveWithParameters(searchParameters)
    # print solution
    if solution:
        printSolution(data, manager, routing, solution)
    else:
        print("No solution found.")


if __name__ == "__main__":
    main()
