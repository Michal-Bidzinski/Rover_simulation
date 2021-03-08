import osmnx as ox
#import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from shapely.geometry import Point
from shapely.geometry import LineString
import math as m
import pickle
from pathlib import Path

# set start and end point 
def set_start_end_point(req):
    
    # if first argument is 0.0 use default coordinates
    if req[0] == 0:
        # start point
        x_s = 16.2609
        y_s = 52.2324

        # end point
        x_e = 16.2300
        y_e = 52.2392
    else:
        # start point
        x_s = req[0]
        y_s = req[1]

        # end point
        x_e = req[2]
        y_e = req[3]
 
    return x_s, y_s, x_e, y_e

# get graph of area
def get_graph_of_area_from_osm(req):

    if req[8] == 1.0:
        # map of airport
        #graph = ox.graph_from_bbox(req[4], req[5], req[6], req[7], network_type='all', custom_filter = ('["aeroway"]'), simplify=False) 
        #print(type(graph))
        #with Path('./graph.p').open('wb') as pickle_file:
        #    pickle.dump(graph, pickle_file)
        with Path('./graph.p').open('rb') as pickle_file2:
            graph2 = pickle.load(pickle_file2)
    else: 
        # map of area in bbox with all type on road 
        graph2 = ox.graph_from_bbox(req[4], req[5], req[6], req[7], network_type='all', simplify=False)

    return graph2


# add new node to graph
def add_node_to_graph(graph, num, x, y):

    # add node with number "num"
    graph.add_node(num)

    # define x, y and  number of node
    attrs = {num: {"x": x, "y": y, "osmid": num}}

    # set new node attributes
    nx.set_node_attributes(graph, attrs)


# add new edge to graph
def add_edge_to_graph(graph, num_node1, num_node2):

    # count length of new edge
    length = m.sqrt((graph.nodes[num_node1]['x'] - graph.nodes[num_node2]['x'])**2 + (graph.nodes[num_node1]['y'] -   graph.nodes[num_node2]['y'])**2)

    # add edge of specific length in both direction 
    graph.add_edge(num_node1, num_node2, length=length)
    graph.add_edge(num_node2, num_node1, length=length)

# find point on nearest edge - projection of new point to edge 
def project_point_on_nearest_line_edge(graph, num):

    #find nearest edge to point, get two point on ends on this line 
    edge_point_1, edge_point_2, key= ox.get_nearest_edge(graph, [graph.nodes[num]['y'],     graph.nodes[num]['x']])

    # get x, y of points on the nearest line 
    x_1 = graph.nodes[edge_point_1]['x']
    y_1 = graph.nodes[edge_point_1]['y']
    x_2 = graph.nodes[edge_point_2]['x']
    y_2 = graph.nodes[edge_point_2]['y']

    # get point coordinates, point that we want to project, by number
    point = Point(graph.nodes[num]['x'], graph.nodes[num]['y'])

    # define Line and distanece to the point
    dist = LineString([(x_1, y_1), (x_2, y_2)]).project(point)

    #find projection of point
    project_point = list(list(LineString([(x_1, y_1), (x_2, y_2)]).interpolate(dist).coords)[0])

    return project_point, edge_point_1, edge_point_2


# add point to graph and add extra point on nearest edge
def add_point_to_graph(graph, num1, num2):

    #find nearest edge to point
    project_point, edge_point_1, edge_point_2 = project_point_on_nearest_line_edge(graph, num1)

    #add project node
    add_node_to_graph(graph, num2, project_point[0], project_point[1])

    # add edge between point and project point
    add_edge_to_graph(graph, num1, num2)

    # add edge between project point and nearest point on the line 
    add_edge_to_graph(graph, num2, edge_point_1)

    # add edge between project point and second nearest point on the line 
    add_edge_to_graph(graph, num2, edge_point_2)
