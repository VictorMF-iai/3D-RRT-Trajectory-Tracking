import osmnx as ox
import networkx as nx
import plotly.graph_objects as go
import numpy as np


# Function to plot the path


# Function to generate a list of lines that follow the path
def node_list_to_path (G, node_list):
    """
    Given a list of nodes, return a list of lines that together
    follow the path
    defined by the list of nodes.
    Parameters
    ----------
    G : networkx multidigraph
    route : list
        the route as a list of nodes
    Returns
    -------
    lines : list of lines given as pairs ( (x_start, y_start),
    (x_stop, y_stop) )
    """
    edge_nodes = list(zip(node_list[:-1], node_list[1:]))
    lines = []
    for u, v in edge_nodes:
        # if there are parallel edges, select the shortest in length
        data = min(G.get_edge_data(u, v).values(),
                   key=lambda x: x['length'])
        # if it has a geometry attribute
        if 'geometry' in data:
            # add them to the list of lines to plot
            xs, ys = data['geometry'].xy
            lines.append(list(zip(xs, ys)))
        else:
            # if it doesn't have a geometry attribute,
            # then the edge is a straight line from node to node
            x1 = G.nodes[u]['x']
            y1 = G.nodes[u]['y']
            x2 = G.nodes[v]['x']
            y2 = G.nodes[v]['y']
            line = [(x1, y1), (x2, y2)]
            lines.append(line)
    return lines


def getTrajNodes (origin_point, destination_point):
    # Load the .OSM file (nodes and edges)
    filepath = 'C:/Users/User/Desktop/tfg/path_planning/osms/export.osm'
    G = ox.graph.graph_from_xml(filepath, bidirectional=True, simplify=False, retain_all=True)

    # Get the nearest nodes to the locations
    origin_node = ox.distance.nearest_nodes(G, X=origin_point[1], Y=origin_point[0], return_dist=False)
    destination_node = ox.distance.nearest_nodes(G, X=destination_point[1], Y=destination_point[0], return_dist=False)

    print('Origin node: ', origin_node)

    print('Destination node: ', destination_node)

    # Finding the optimal path
    route = nx.shortest_path(G, origin_node, destination_node, weight='length')  # by default, it uses dijkstra

    lines = node_list_to_path(G, route)
    print(lines)
    dataout = []
    for i in range(len(lines)):
        dataout.append(lines[i][0])
    dataout.append(lines[len(lines) - 1][1])
    print(dataout)

    return dataout


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    getTrajNodes()
