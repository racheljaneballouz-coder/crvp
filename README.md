# crvp
crvp using python
!pip install ortools
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# Distance matrix (10 cities)
distance_matrix = [
[0, 3, 2, 33, 6, 1, 2, 19, 29, 6],
[3, 0, 1, 30, 3, 2, 2, 21, 33, 3],
[2, 1, 0, 31, 4, 1, 2, 21, 31, 4],
[33, 30, 31, 0, 26, 31, 30, 33, 61, 26],
[6, 3, 4, 26, 0, 5, 4, 20, 36, 0],
[1, 2, 1, 31, 5, 0, 0, 20, 31, 4],
[2, 2, 2, 30, 4, 0, 0, 19, 31, 4],
[19, 21, 21, 33, 20, 20, 19, 0, 35, 20],
[29, 33, 31, 61, 36, 31, 31, 35, 0, 35],
[6, 3, 4, 26, 0, 4, 4, 20, 35, 0]
]

# Demands for each city
demands = [0, 213, 213, 98, 124, 100, 178, 128, 144, 92]

# Truck capacities (7 trucks with different capacities)
truck_capacities = [224, 224, 168, 200, 224, 168, 224]

# Depot index
depot = 0
# Create the routing index manager
manager = pywrapcp.RoutingIndexManager(len(distance_matrix), len(truck_capacities), depot)

# Create the routing model
routing = pywrapcp.RoutingModel(manager)

# Define the distance callback
def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return distance_matrix[from_node][to_node]

transit_callback_index = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Define the demand callback
def demand_callback(from_index):
    from_node = manager.IndexToNode(from_index)
    return demands[from_node]

demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
routing.AddDimensionWithVehicleCapacity(
    demand_callback_index,
    0, # No slack
    truck_capacities,
    True, # Start and end at the depot
    "Capacity"
)
# Set search parameters
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

# Solve the problem
solution = routing.SolveWithParameters(search_parameters)

# Print solution
if solution:
    for vehicle_id in range(len(truck_capacities)):
        index = routing.Start(vehicle_id)
        route = []
        load = 0
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            route.append(node)
            load = demands[node]
            index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))
        print(f"Route for truck {vehicle_id}: {route}")
        print(f"Load: {load}")
else:
    print("No solution found")
