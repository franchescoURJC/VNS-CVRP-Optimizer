"""
Algoritmo de optimización de rutas para vehículos con capacidades de 100 usando metaheurísticas 
Se deben visitar con exactamente el número de vehículos indicados recorriendo la menor distancia posible.
(El depósito tiene demanda 0)
"""

from math import sqrt
import time
import random 
import matplotlib.pyplot as plt

def read_data(filename):
    with open(filename, "r") as file:
        lines = file.readlines()

        # Ambos son listas ya que utilizo los índices como id de cada nodo, (el orden de los elementos se debe conservar)
        coordinates = []
        demands = []
        section = None

        for line in lines:
            stripped_line = line.strip()
            if stripped_line.upper().startswith("NODO_COORDX_COORDY"): #Evitar errores por minúsculas/mayúsculas
                section = "coordinates"
                continue
            elif stripped_line.upper().startswith("DEMANDA"):
                section = "demands"
                continue
            
            if section == "coordinates":
                if stripped_line:
                    parts = stripped_line.split()
                    if len(parts) == 3:
                        # Asumo que vienen los nodos en el mismo orden en coordenadas y en demandas y en orden creciente. Además por simpleza del código el id del nodo es el nodo del fichero menos uno.
                        node = int(parts[0])
                        x = int(parts[1])
                        y = int(parts[2])
                        coordinates.append((node-1, x, y))
                    else:
                        Exception("Error: bad file formatting")
            
            elif section == "demands":
                if stripped_line:
                    parts = stripped_line.split()
                    if len(parts) == 2:
                        demand = int(parts[1])
                        demands.append(demand)
                    else:
                        Exception("Error: bad file formatting")

    return coordinates, demands
    

def calculate_euclidean_distance(coord_x1, coord_y1, coord_x2, coord_y2):
    return sqrt((coord_x2 - coord_x1)**2 + (coord_y2 - coord_y1)**2)


def process_data(coordinates):
    distance_matrix = [[] for _ in coordinates]
    savings = []

    # Calculate distances
    for start_node, x1, y1 in coordinates:
        for _, x2, y2 in coordinates:
            distance = calculate_euclidean_distance(x1, y1, x2, y2)
            distance_matrix[start_node].append(distance)
    
    """
    Clarke & Wright savings heuristic
    Savings formula: s(i, j) = d(D, i) + d(D, j) - d(i, j) 
    """ 
    # Calculate savings
    for i in range(len(distance_matrix)):
        for j in range(1, i): # Going to the deposit equals 0 in savings 
            saving = distance_matrix[0][i] + distance_matrix[0][j] - distance_matrix[i][j]
            savings.append((i, j, saving))
    
    # Order by savings descending
    savings.sort(key = lambda x: x[2], reverse=True)

    return distance_matrix, savings    


def objective_function(tours, distance_matrix):
    """
    La función objetivo debe minimizar la distancia total recorrida.
    """ 
    distances = 0
    for route in tours:
        for i in range(len(route)-1):
            distances += distance_matrix[route[i]][route[i + 1]]
            
    return distances


def is_interior_to_route(node, route):
    return route[1] != node and route[-2] != node


def get_node_route_index(node, tours):
    for route_index in range(len(tours)):
        for node_index in range(len(tours[route_index])):
            if tours[route_index][node_index] == node:
                return route_index

    raise Exception("Node not found")


def init_solution(savings, demands):
    # Clarke & Wright savings heuristic
    visited_nodes = set()
    routes = []
    capacities = []
    savings_index = 0

    while savings_index < len(savings):
        capacity_index = None
        start_node, end_node, _ = savings[savings_index]
        
        # Feasibility check
        edge_demands = 0
        if start_node not in visited_nodes:
            edge_demands += demands[start_node]
        if end_node not in visited_nodes:
            edge_demands += demands[end_node]

        if start_node not in visited_nodes and end_node not in visited_nodes and len(routes) <= VEHICLES:
            # Create new route
            new_route = [0]
            new_route.append(start_node)
            new_route.append(end_node)
            visited_nodes.add(start_node)
            visited_nodes.add(end_node)
            new_route.append(0)
            routes.append(new_route)
            capacities.append(VEHICLE_CAPACITY)
            capacity_index = len(capacities)-1
            
        elif (start_node in visited_nodes) != (end_node in visited_nodes): 
            
            if start_node in visited_nodes:
                start_node_route_index = get_node_route_index(start_node, routes)
                start_node_route = routes[start_node_route_index]
                if not is_interior_to_route(start_node, start_node_route) and capacities[start_node_route_index] - edge_demands >= 0:
                    start_node_route.insert(-1, end_node) 
                    visited_nodes.add(end_node)
                    capacity_index = start_node_route_index

            else:
                end_node_route_index = get_node_route_index(end_node, routes)
                end_node_route = routes[end_node_route_index]
                if not is_interior_to_route(end_node, end_node_route) and capacities[end_node_route_index] - edge_demands >= 0:
                    end_node_route.insert(-1, start_node)
                    visited_nodes.add(start_node)
                    capacity_index = end_node_route_index
        
        elif start_node in visited_nodes and end_node in visited_nodes:
            start_node_route_index = get_node_route_index(start_node, routes)
            end_node_route_index = get_node_route_index(end_node, routes)
            start_node_route = routes[start_node_route_index]
            end_node_route = routes[end_node_route_index]

            # Comparar punteros de objetos en vez de elemento a elemento con "is not"
            if (start_node_route is not end_node_route) and \
            not is_interior_to_route(start_node, start_node_route) and \
            not is_interior_to_route(end_node, end_node_route):
                # Check feasibility of merging two routes
                merged_route_demand = 2 * VEHICLE_CAPACITY - capacities[end_node_route_index] - capacities[start_node_route_index]
                if merged_route_demand <= VEHICLE_CAPACITY:
                    # Remove depot from end of start_node_route and from start of end_node_route
                    start_node_route.pop()

                    # Merge routes
                    routes.remove(end_node_route)
                    start_node_route += end_node_route[1:]

                    # Update capacities
                    capacities[start_node_route_index] = VEHICLE_CAPACITY - merged_route_demand
                    capacities.pop(end_node_route_index)

        if capacity_index != None:
            capacities[capacity_index] -= edge_demands
        
        savings_index += 1
    return routes
            

def two_opt(tours, route_index, i, j):
    route_list = tours[route_index]
    new_route_list = route_list[:i] + route_list[i:j+1][::-1] + route_list[j+1:]
    tours[route_index] = new_route_list
    return tours
    

def shake(tours, k):
    new_tours = tours.copy()
    for _ in range(k):
        route_index = random.randint(0, len(tours)-1)
        i, j = sorted(random.sample(range(1, len(tours[route_index])-1), 2))
        new_tours = two_opt(new_tours, route_index, i, j)
    return new_tours


def local_search(tours, best_distance, distance_matrix):
    better_solution_found = True
    new_tours = tours.copy()
    # Repeat until no new solution is found
    while better_solution_found:
        better_solution_found = False
        for route_index in range(len(tours)):
            for i in range(1, len(tours[route_index]) - 1):
                for j in range(i + 1, len(tours[route_index]) - 1):
                    if j-i == 1: continue
                    new_tours = two_opt(new_tours, route_index, i, j)
                    new_distance = objective_function(new_tours, distance_matrix)
                    if new_distance < best_distance:
                        tours = new_tours.copy()
                        best_distance = new_distance
                        better_solution_found = True
    return tours, best_distance


def vns(distance_matrix, savings, demands, max_k, max_time):
    tours = init_solution(savings, demands)

    # Copy for solution comparison
    init_tours = tours.copy() 
    best_distance = objective_function(tours, distance_matrix)
    print("Init distance:", best_distance)

    if len(tours) > VEHICLES:
        raise Exception("ERROR: More routes than vehicles")
    
    print("Demands:", end=" ")
    for route in tours:
        acc = 0
        for node in route:
            acc += demands[node]
        print(acc, end=" ")
    print()
    
    print("Initial tour:")
    for route in tours:
        print(route)
    
    start_time = time.monotonic() # Evita bugs por cambio de hora del sistema
    current_time = time.monotonic()

    """
        Esto permite que se superen los 5 minutos durante la última iteración, ya que no se produce una interrupción de la ejecución exactamente a los 5 minutos.
        Asumo que es lo que se pedía.
    """
    while current_time - start_time < max_time:
        k = 1
        while k < max_k:
            solution_k = shake(tours, k)
            current_routes, current_distance = local_search(solution_k, best_distance, distance_matrix)
            if current_distance < best_distance:
                tours = current_routes
                best_distance = current_distance
                print("\n#################################")
                print("New solution found")
                print("Distance:", best_distance)
                print("Time elapsed:", (current_time - start_time)/60, "minutes")
                print("#################################")
                k = 1
            else:
                k += 1
        
        current_time = time.monotonic()
    return init_tours, tours, best_distance, (current_time - start_time)/60


def generate_visualization(coordinates, tour, title):
    # Unpack node data
    ids, xs, ys = zip(*coordinates)

    # Create a scatter plot
    plt.figure(figsize=(12, 8))
    xs, ys = zip(*[(x, y) for _, x, y in coordinates])
    plt.scatter(xs, ys, color="blue", marker="o")

    # Draw edges
    colors = ["red", "orange", "gold", "green", "blue", "mediumorchid", "cyan", "black", "slategrey"]
    counter = 0
    for route in tour:
        route_edges = [(route[i], route[i + 1]) for i in range(len(route) - 1)]
        for start_id, end_id in route_edges:
            start_node = next((x, y) for id, x, y in coordinates if id == start_id)
            end_node = next((x, y) for id, x, y in coordinates if id == end_id)
            plt.plot([start_node[0], end_node[0]], [start_node[1], end_node[1]], color=colors[counter], linestyle="-", linewidth=2)
        counter += 1

    # Annotate each point with its id
    for i, id in enumerate(ids):
        plt.text(xs[i], ys[i], str(id), fontsize=9, ha="right")

    plt.title(title)
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")


# Constants
SMALL_FILENAME = "32nodos_5Veh_100Capacidad.txt"
LARGE_FILENAME = "65nodos_9Veh_100Capacidad.txt"
MAX_TIME = 5 * 60 # 5 minutos
MAX_K = 10
VEHICLES = 9 # NOTE: Este algoritmo no utiliza exactamente como mínimo y máximo este número de vehículos. Solamente como máximo, es lo que interpreto del enunciado propuesto.
VEHICLE_CAPACITY = 100

coordinates, demands = read_data(LARGE_FILENAME)

# Initial feasibility check
total_capacity = VEHICLE_CAPACITY * VEHICLES
if total_capacity < sum(demands):
    raise Exception("Infeasible: Not enough vehicles/capacity")

distance_matrix, savings = process_data(coordinates)
init_tour, best_tour, best_distance, time_elapsed = vns(distance_matrix, savings, demands, MAX_K, MAX_TIME)

print("-------------------------------------")
print("Final result")
print("Time elapsed:", time_elapsed)
print("Distance:", best_distance)
print("Tours:")
for route in best_tour:
    print(route)

generate_visualization(coordinates, init_tour, "Initial solution visualization")
generate_visualization(coordinates, best_tour, "Best solution visualization")
plt.show()

