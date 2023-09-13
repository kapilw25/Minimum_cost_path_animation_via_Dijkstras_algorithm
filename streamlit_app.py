import streamlit as st
import heapq
import networkx as nx
import matplotlib.pyplot as plt
from io import BytesIO
import imageio.v2 as imageio
import os

os.environ['IMAGEIO_FFMPEG_EXE'] = '/opt/homebrew/bin/ffmpeg'


@st.cache_data
def parse_input(filename):
    with open(filename, 'r') as f:
        n = int(f.readline().strip())
        start = int(f.readline().strip())
        goal = int(f.readline().strip())
        edges = {}
        for line in f:
            u, v, w = line.strip().split()
            u, v, w = int(u), int(v), float(w)
            if u not in edges:
                edges[u] = []
            edges[u].append((v, w))
    return n, start, goal, edges


@st.cache_data
def parse_coordinates(filename):
    coords = {}
    with open(filename, 'r') as f:
        for index, line in enumerate(f, start=1):
            x, y = map(float, line.strip().split())
            coords[index] = (x, y)
    return coords


@st.cache_data
def dijkstra(n, start, goal, edges):
    visited = set()
    distances = {vertex: float('infinity') for vertex in range(1, n + 1)}
    previous_vertices = {vertex: None for vertex in range(1, n + 1)}
    distances[start] = 0
    vertices = [(0, start)]
    while vertices:
        current_distance, current_vertex = heapq.heappop(vertices)
        if current_vertex in visited:
            continue
        visited.add(current_vertex)
        for neighbor, weight in edges.get(current_vertex, []):
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_vertices[neighbor] = current_vertex
                heapq.heappush(vertices, (distance, neighbor))
    path, current_vertex = [], goal
    while previous_vertices[current_vertex] is not None:
        path.append(current_vertex)
        current_vertex = previous_vertices[current_vertex]
    if current_vertex != start:
        return [], []
    path.append(start)
    return path[::-1], [distances[vertex] for vertex in path[::-1]]

def write_output(path, distances, filename="output.txt"):
    with open(filename, 'w') as f:
        f.write(' '.join(map(str, path)) + '\n')
        f.write(' '.join(['{:.4f}'.format(dist) for dist in distances]) + '\n')


def visualize_graph(edges, coords, active_vertex=None, partial_path=None, active_edge=None):
    # visualize graph (for graph visualization) 
    G = nx.DiGraph() 
    # initialize graph (for graph visualization) 

    # Add nodes and edges to the graph
    for u, v_w_list in edges.items(): 
    # for each vertex and list of neighbors and weights in edges dictionary (for graph visualization)
        for v, w in v_w_list: 
        # for each neighbor and weight in list of neighbors and weights
            G.add_edge(u, v, weight=w) 
            # add edge to graph (for graph visualization)

    plt.figure(figsize=(10, 6)) 
    # set figure size (for graph visualization)

    node_colors = ['g' if node == active_vertex else 'b' for node in G.nodes()] 
    # set node colors (for graph visualization)
    # green color for active vertex, blue color for all other vertices

    nx.draw_networkx_nodes(G, pos=coords, node_color=node_colors) 
    # pos for coordinates and node_color for node colors (for graph visualization)
    nx.draw_networkx_labels(G, pos=coords) 
    # draw node labels (for graph visualization)

    if partial_path: 
    # if partial path is not empty
        path_edges = list(zip(partial_path[:-1], partial_path[1:])) 
        # create list of edges in partial path
        nx.draw_networkx_edges(G, pos=coords, edgelist=path_edges, edge_color='y', width=1.5) 
        # draw edges in partial path (for graph visualization)

    if active_edge: 
    # if active edge is not empty
        nx.draw_networkx_edges(G, pos=coords, edgelist=[active_edge], edge_color='g', width=2) 
        # draw active edge (for graph visualization)

    else: 
    # if active edge is empty and partial path is empty (i.e., initial graph)
        nx.draw_networkx_edges(G, pos=coords) 
        # draw all edges and set default edge color and position

    plt.axis('off') 
    # turn off axis else it will show axis with vertex numbers
    plt.tight_layout() 
    # tight layout as it will show axis with vertex numbers

    buf = BytesIO() 
    # initialize buffer, which is a BytesIO object (for graph visualization)
    plt.savefig(buf, format='png') 
    # save figure to buffer as png
    buf.seek(0) 
    # seek to the beginning of the buffer
    image_array = imageio.imread(buf) 
    # read buffer as image array, helping to create animation (for graph visualization)

    plt.close() 
    # close figure, guiding matplotlib to free memory

    return image_array 
    # return image array (for graph visualization)


def dijkstra_visualized(n, start, goal, edges, coords):
# Dijkstra's algorithm to solve for minimum cost paths on a given graph (with visualization)
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

    images = [] 
    # initialize images list (for graph visualization)
    partial_paths = {} 
    # initialize partial paths dictionary (key: vertex, value: partial path)

    while vertices: 
        # while vertices list is not empty (i.e., there are still vertices to visit)
        current_distance, current_vertex = heapq.heappop(vertices) 
        # pop vertex with minimum distance from vertices list
        if current_vertex in visited: 
        # if vertex has already been visited, 
            continue 
            # continue to next iteration of while loop
        visited.add(current_vertex) 
        # add vertex to visited set (i.e., mark vertex as visited)
        for neighbor, weight in edges.get(current_vertex, []): 
        # for each neighbor of current vertex 
            active_edge = (current_vertex, neighbor) 
            # set active edge (for graph visualization)

            partial_path = [current_vertex] 
            # initialize partial path (for graph visualization)
            prev_vertex = previous_vertices[current_vertex] 
            # initialize previous vertex (for graph visualization)
            while prev_vertex is not None: 
            # while previous vertex is not None (i.e., while current vertex has a previous vertex)
                partial_path.append(prev_vertex) 
                # append previous vertex to partial path (for graph visualization)
                prev_vertex = previous_vertices[prev_vertex] 
                # update previous vertex to previous vertex of previous vertex (for graph visualization)
            partial_paths[current_vertex] = partial_path[::-1] 
            # add partial path to partial paths dictionary (for graph visualization)

            image_array = visualize_graph(edges, coords, current_vertex, partial_paths.get(current_vertex), active_edge)
            # visualize graph (for graph visualization) 
            images.append(image_array) 
            # append image array to images list (for graph visualization)

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
    # append start vertex to path list (i.e., path list is now complete)

    image_array = visualize_graph(edges, coords, active_vertex=None, partial_path=path, active_edge=None)
    # visualize graph (for graph visualization)  
    images.append(image_array) 
    # append image array to images list (for graph visualization)

    imageio.mimsave('hw1.mp4', images, fps=30) 
    # save images list as mp4 file
    # mimsave() function from imageio module to save images list as mp4 file
    # fps=1 to set frame rate to 1 frame per second

    return path[::-1], [distances[vertex] for vertex in path[::-1]] 
    # return path list in reverse order and distances list in reverse order

coords = parse_coordinates('coords.txt')
n, start, goal, edges = parse_input('input.txt')
path, distances = dijkstra_visualized(n, start, goal, edges, coords)
write_output(path, distances)

print("Visualization saved as 'hw1.mp4'.")


def main():
    st.title('Graph Visualization using Dijkstra Algorithm')

    coords = parse_coordinates('coords.txt')
    n, _, _, edges = parse_input('input.txt')  # Read the file but ignore start and goal values

    start_options = list(range(1, n + 1))  # Create a list of valid start and end options
    start_index = st.selectbox("Select Start Vertex", start_options, index=0)
    destination_index = st.selectbox("Select Destination Vertex", start_options, index=n-1)

    if st.button("Visualize Path"):
        path, distances = dijkstra_visualized(n, start_index, destination_index, edges, coords)
        write_output(path, distances)

        st.success("Visualization saved as 'hw1.mp4'.")
        st.video('hw1.mp4')

if __name__ == "__main__":
    main()