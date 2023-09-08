
# # Instructions to Compile and Run Source Code

# Prerequisites:

# Ensure you have Python installed on your computer. If not, you can download and install Python from the official website. https://www.python.org/downloads/

# ## Steps:

# ## Step 1) Setup:

# Place **"hw1.py"** python file in a directory of your choice.

# Place the input file named **"input.txt"** in the same directory as **"hw1.py"** .

# ## Step 2) Open Terminal or Command Prompt:

# On Windows: Press **"Windows + R"** keys, type cmd, and press Enter.

# On macOS: Press **"Cmd + Space"** , type terminal, and press Enter.

# On Linux: Depending on the distro, you can use shortcuts or search for the terminal in the application menu.

# ## Step 3) Navigate to the Directory:

# Use the cd command followed by the directory path where you've saved hw1.py to navigate to the directory.

# ```shell
# cd path_to_directory
# ```

# ## Step 4) Run the Script:

# Once in the correct directory, type the following command to execute the script:

# ```shell
# python hw1.py
# ```

# ## Step 5) Check the Output:

# After running the script, an **"output.txt"** file will be generated in the same directory as the hw1.py script.

# Open **"output.txt"** to view the results.


# %%
import heapq

def parse_input(filename): 
    # parse input file
    with open(filename, 'r') as f: 
        # read input file
        n = int(f.readline().strip()) 
        # number of vertices
        start = int(f.readline().strip()) 
        # start vertex
        goal = int(f.readline().strip()) 
        # goal vertex
        edges = {} 
        # edges dictionary (key: vertex, value: list of tuples (neighbor, weight))
        for line in f: 
            # read edges and weights and add to edges dictionary
            u, v, w = line.strip().split() 
            # u: vertex, v: neighbor, w: weight
            u, v, w = int(u), int(v), float(w) 
            # convert to int and float as needed
            if u not in edges: 
            # add vertex to edges dictionary if not already present
                edges[u] = [] 
                # initialize list of neighbors and weights for vertex u in edges dictionary
            edges[u].append((v, w)) 
            # add neighbor and weight to list of neighbors and weights for vertex u in edges dictionary
    return n, start, goal, edges 
    # return number of vertices, start vertex, goal vertex, and edges dictionary

def dijkstra(n, start, goal, edges): 
# Dijkstra's algorithm to solve for minimum cost paths on a given graph
    visited = set() 
    # initialize visited set
    distances = {vertex: float('infinity') for vertex in range(1, n + 1)} 
    # initialize distances dictionary (key: vertex, value: distance)
    previous_vertices = {vertex: None for vertex in range(1, n + 1)} 
    # initialize previous vertices dictionary (key: vertex, value: previous vertex)
    distances[start] = 0 
    # set distance of start vertex to 0
    vertices = [(0, start)] 
    # initialize vertices list (tuple: (distance, vertex))
    while vertices: 
    # while vertices list is not empty (i.e., there are still vertices to visit)
        current_distance, current_vertex = heapq.heappop(vertices) 
        # pop vertex with minimum distance from vertices list
        if current_vertex in visited: 
        # if vertex has already been visited, continue
            continue 
            # continue to next iteration of while loop
        visited.add(current_vertex) 
        # add vertex to visited set (i.e., mark vertex as visited)
        for neighbor, weight in edges.get(current_vertex, []): 
        # for each neighbor of current vertex
            distance = current_distance + weight 
            # calculate distance to neighbor
            if distance < distances[neighbor]: 
            # if distance to neighbor is less than current distance to neighbor
                distances[neighbor] = distance 
                # update distance to neighbor
                previous_vertices[neighbor] = current_vertex 
                # update previous vertex of neighbor
                heapq.heappush(vertices, (distance, neighbor)) 
                # push neighbor to vertices list
    path, current_vertex = [], goal 
    # initialize path list and current vertex
    while previous_vertices[current_vertex] is not None: 
    # while current vertex has a previous vertex
        path.append(current_vertex) 
        # append current vertex to path list
        current_vertex = previous_vertices[current_vertex] 
        # update current vertex to previous vertex
    if current_vertex != start:
    # if current vertex is not start vertex
        return [], [] 
        # return empty path and empty distances
    path.append(start) 
    # append start vertex to path list
    return path[::-1], [distances[vertex] for vertex in path[::-1]] 
    # return path list in reverse order and distances list in reverse order

def write_output(path, distances, filename="output.txt"): 
# write output file
    with open(filename, 'w') as f: 
    # write output file (path and distances)
        f.write(' '.join(map(str, path)) + '\n') 
        # write path to output file (space-separated)
        f.write(' '.join(['{:.4f}'.format(dist) for dist in distances]) + '\n') 
        # write distances to output file (space-separated, formatted to 4 decimal places)

n, start, goal, edges = parse_input('input.txt') 
# parse input file and store number of vertices, start vertex, goal vertex, and edges dictionary
path, distances = dijkstra(n, start, goal, edges) 
# run Dijkstra's algorithm to solve for minimum cost paths on a given graph and store path and distances
write_output(path, distances) 
# write output file (path and distances) to output.txt file


# %%



