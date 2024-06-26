#! /usr/bin/env python3

import argparse
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt 
from matplotlib.patches import Polygon as pltPolygon
from shapely.geometry import Point, Polygon
import sys
import copy
from .tools import (
    json_load_wrapper,
    find_mobile_locations,
    find_manipulation_locations_wo_ee,
    select_midpoint_in_bounds,
    bounds2vertices,
)

DEBUG = False
DRAW_MIDPOINTS = False
PLOT_MAP = False

class Roadmap:
    def __init__(self, locations: dict, PLOT_MAP: bool = False) -> None:
        self.graph = nx.Graph()
        self.locations = locations
        self.num_pts = 100
        self.PLOT_MAP = PLOT_MAP
        self.change_location_bound()
        self._build_graph()

    def change_location_bound(self) -> None:
        for _, data in self.locations.items():
            data['bounds'] = data['bounds'][:2]
        return None

    def _build_graph(self) -> None:
        self._bounds2vertices()
        self._add_centers()
        for location, data in self.locations.items():
            # Find adjacent locations, neighbors is a list of tuples of (location names, midpt in the common edge)
            neighbors = self._find_adjacent_locations(location, self.locations)
            for neighbor_location, midpt in neighbors:
                self.graph.add_edge(location, neighbor_location, midpt=midpt)

        if self.PLOT_MAP:
            self.plot_graph()
            self.plot_map()

        return None
    
    def _bounds2vertices(self) -> None:
        """Change location bounds to vertices"""
        for _, data in self.locations.items():
            data['vertices'] = bounds2vertices(data['bounds'])
            if DEBUG:
                print(data['vertices'])
        return None

    def _add_centers(self) -> None:
        """Add midpoints of each location to the graph as a node"""
        for location, data in self.locations.items():
            center = select_midpoint_in_bounds(bounds=data['bounds'], n_dims=len(data['bounds']))
            # print(midpt)
            self.graph.add_node(location, pos=list(np.around(np.array(center), decimals=2)))
        if DEBUG:
            node_positions = nx.get_node_attributes(self.graph, 'pos')
            nx.draw(self.graph, pos=node_positions, with_labels=True, node_size=3000, node_color='skyblue', font_size=10)
            plt.show()
            sys.exit(0)
        return None
    
    def _find_adjacent_locations(self, location: str, locations: dict) -> list:
        """Find adjacent locations of a given location
        
        Given: 
            location: str, name of the location
            locations: dict, all locations
        
        Return:
            neighbors: list of tuples, each tuple is (neighbor location name, midpt in the common edge)
        """
        neighbors = []
        for neighbor_location, _ in locations.items():
            if neighbor_location == location:
                continue
            midpt = self._find_common_edge_midpt(location, neighbor_location)
            if midpt is not None:
                # print(midpt)
                neighbors.append((neighbor_location, midpt))
        return neighbors
    
    def _find_common_edge_midpt(self, location1: str, location2: str) -> list:
        """Find the midpt of the common edge of two locations
        
        Given: 
            location1, location2: str, names of two locations
        
        Return:
            midpt: list, midpt of the common edge [x, y]
                    None, if the two locations do not share an edge
        """
        vertices1 = self.locations[location1]['vertices']
        vertices2 = self.locations[location2]['vertices']
        polygon = Polygon(vertices2)
        for idx, vertice in enumerate(vertices1):
            next_idx = (idx + 1) % len(vertices1)
            next_vertice = vertices1[next_idx]
            x_interpolated = np.linspace(vertice[0], next_vertice[0], self.num_pts)
            y_interpolated = np.linspace(vertice[1], next_vertice[1], self.num_pts)
            check_pts = np.vstack((x_interpolated, y_interpolated)).T
            if DEBUG:
                print(vertice, next_vertice)
                print(check_pts)
                sys.exit(0)
            shapely_pts = [Point(pt) for pt in check_pts]
            if DEBUG:
                print(shapely_pts)
                sys.exit(0)
            points_inside_polygon = np.array([point.touches(polygon) or polygon.contains(point) for point in shapely_pts])
            if DEBUG:
                print(all(points_inside_polygon))
            if all(points_inside_polygon):
                midpt = np.array([(vertice[0] + next_vertice[0]) / 2, (vertice[1] + next_vertice[1]) / 2])
                return list(np.around(midpt, decimals=2))
        return None
    
    def plot_graph(self) -> None:
        node_positions = nx.get_node_attributes(self.graph, 'pos')
        edge_labels = nx.get_edge_attributes(self.graph, 'midpt')
        nx.draw_networkx_edge_labels(self.graph, pos=node_positions, edge_labels=edge_labels, font_color='red')
        nx.draw(self.graph, pos=node_positions, with_labels=True, node_size=3000, node_color='skyblue', font_size=10)
        plt.title('Roadmap with Nodes, Edges, and Edge Attributes (midpoints)')
        plt.show()

    def plot_map(self) -> None:
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        first_pt = True
        for _, data in self.locations.items():
            vertices = data['vertices']
            polygon = pltPolygon(vertices, edgecolor='r', facecolor='none')
            ax.add_patch(polygon)
        if DRAW_MIDPOINTS:
            for edge in self.graph.edges:
                midpt = self.graph.edges[edge]['midpt']
                if first_pt:
                    ax.scatter(midpt[0], midpt[1], color='green', marker='o', s=100, label="Midpoints")
                    first_pt = False
                else:
                    ax.scatter(midpt[0], midpt[1], color='green', marker='o', s=100)
        ax.legend()
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_title('Map with Regions and Edge Midpoints')
        plt.show()

    def get_node_pos(self, node) -> dict:
        return self.graph.nodes[node]['pos']
    
    def get_edge_attributes(self, node1, node2) -> dict:
        return self.graph.get_edge_data(node1, node2)

class RoadmapMobile(Roadmap):
    def __init__(self, files_json_name: str, PLOT_MAP: bool = False) -> None:
        self.locations = find_mobile_locations(json_load_wrapper(json_load_wrapper(files_json_name)['locations']))
        super().__init__(self.locations, PLOT_MAP=PLOT_MAP)

class RoadmapManipulation(Roadmap):
    def __init__(self, files_json_name: str, ) -> None:
        self.locations = find_manipulation_locations_wo_ee(json_load_wrapper(json_load_wrapper(files_json_name)['locations']))
        self.locations3D = copy.deepcopy(self.locations)
        super().__init__(self.locations)
        self.change_center_backto_3D()
    
    def change_center_backto_3D(self) -> None:
        for loc in self.graph.nodes:
            # print(self.locations3D[loc])
            self.graph.nodes[loc]['pos'] = select_midpoint_in_bounds(self.locations3D[loc]['bounds'], n_dims=3)

        if True:
            print("nodes positions: ", nx.get_node_attributes(self.graph, 'pos'))

def test(files_json): 
    PLOT_MAP = True
    # files_name = 'inputs/pickup_dropoff_cup/files.json'
    roadmap = RoadmapMobile(files_json)
    # files_name = 'inputs/pickup_dropoff_cup_block1_min_unrealizable_plus_cup/files.json'
    manipulation_roadmap = RoadmapManipulation(files_json)
    # locations = find_mobile_locations(json_load_wrapper(files['locations']))
    # print(locations)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--files', type=str, help='files.json')
    args = parser.parse_args()
    files_json = args.files

    test(files_json)