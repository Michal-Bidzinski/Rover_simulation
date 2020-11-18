#!/usr/bin/env python3.6
import matplotlib
matplotlib.use('TkAgg')
import osmnx as ox
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import rospy
import cv2
from matplotlib.backends.backend_agg import FigureCanvasAgg
from shapely.geometry import Point
from shapely.geometry import LineString
from osm_server.srv import *
from osm_library import add_node_to_graph, add_edge_to_graph, project_point_on_nearest_line_edge, add_point_to_graph, set_start_end_point, get_graph_of_area_from_osm
from osm_server.msg import Array_my_a, Array_my_b

def handle_numbers(req):
    # read argumnets
    args = list(req.arg)

    # set star and end point
    x_s, y_s, x_e, y_e = set_start_end_point(args)

    # select place
    graph = get_graph_of_area_from_osm(args)

    # add start node
    add_node_to_graph(graph, 1, x_s, y_s)

    # add end node
    add_node_to_graph(graph, 3, x_e, y_e)

    # project graph from lat long to UTM
    graph2 = ox.projection.project_graph(graph)

    # add start point to graph
    add_point_to_graph(graph2, 1, 2)

    # add end point to graph
    add_point_to_graph(graph2, 3, 4)

    # find shortest path:
    route = nx.shortest_path(graph2, 1, 3, weight='length')

    fig, ax = ox.plot_graph_route(graph2, route, show=False, close=False) 

    # PLOT
    show = False
    if show:
        plt.show()

    # creating the returned structure
    node_array = Array_my_a([])
    for i in route:
        node_array.array1.append(Array_my_b([graph2.nodes[i]['x'], graph2.nodes[i]['y']]))
    print("generate_path")


    return OSM_pathResponse(node_array)

def osm_server():
    # ros init
    rospy.init_node('osm_path_generator_servis')
    s = rospy.Service('osm_path', OSM_path, handle_numbers)
    rospy.spin()


if __name__ == "__main__":
    osm_server()


