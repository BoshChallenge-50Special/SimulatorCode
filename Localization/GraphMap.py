# -*- coding: utf-8 -*-

import xml.etree.ElementTree as ET
import math

import time


class GraphMap(object):

    def __init__(self, path):

        self.Map = dict()

        tree = ET.parse(path)

        root = tree.getroot()

        self.Map = dict()

        for node in root[3].iter('{http://graphml.graphdrawing.org/xmlns}node'):
            id = node.get('id')
            x = node[0].text
            y = node[1].text
            next = list()
            nextDotted = list()
            previous = list()
            self.Map[id] = { "x": x, "y": y, "next": next, "nextDotted": nextDotted, "previous": previous }


        for edge in root[3].iter('{http://graphml.graphdrawing.org/xmlns}edge'):
            source = edge.get('source')
            target = edge.get('target')
            nextDottedEdge = edge[0].text
            self.Map[source]["next"].append(target)
            self.Map[source]["nextDotted"].append(nextDottedEdge)
            self.Map[target]["previous"].append(source)

        return

    def get_location_points(self, x, y, num):
        options = list()
        for node in self.Map:
            distance = math.sqrt( ( ( float(self.Map[node]["x"]) - x ) ** 2 ) + ( ( float(self.Map[node]["y"]) - y ) ** 2 ) )
            
            options.append((node, distance))

        options.sort(key=lambda tup: tup[1])  # sorts in place

        return options[0:num]

    def get_path(self, start, end):
        print("Path from " + start + " to " + end)

        # Create lists for open nodes and closed nodes
        open = []
        closed = []
        
        # Create a start node and an goal node
        start_node = (start, None, 0)
        goal_node = (end, None, 0)
        # Add the start node
        open.append(start_node)
        
        # Loop until the open list is empty
        while len(open) > 0:
            # Sort the open list to get the node with the lowest cost first
            open.sort(key=lambda x:x[2])
            # Get the node with the lowest cost
            current_node = open.pop(0)
            # Add the current node to the closed list
            closed.append(current_node)
            
            # Check if we have reached the goal, return the path
            if current_node[0] == end:
                total = current_node[2]
                path = []
                print(current_node[2])
                while current_node[0] != start_node[0]:
                    path.append(current_node[0])
                    val = float("inf")
                    for node in closed:
                        if(node[0] == current_node[1] and node[2] < val):
                            val = node[2]
                            current_node = node
                            closed.remove(node)
                path.append(start_node[0])
                # Return reversed path and total distance
                return path[::-1], total

            # Get neighbours
            neighbors = self.Map[current_node[0]]["next"]
            
            # Loop neighbors
            for key in neighbors:
                # Create a neighbor node
                neighbor = (key, current_node[0], current_node[2] + self.get_distance(current_node[0], key) )
                # Check if the neighbor is in the closed list
                if(neighbor in closed):
                    continue
                # Check if neighbor is in open list and if it has a lower f value
                for node in open:
                    if (neighbor[0] == node[0] and neighbor[1] == node[1] and neighbor[2] > node[2]):   
                        continue             
                open.append(neighbor)
        # Return None, no path is found
        return None
        
    def get_distance(self, start, end):
        #print("Distance from " + start + " to " + end)

        x_diff = float(self.Map[start]["x"]) - float(self.Map[end]["x"])
        y_diff = float(self.Map[start]["y"]) - float(self.Map[end]["y"])
        
        return math.sqrt((x_diff ** 2) + (y_diff ** 2))




if __name__ == '__main__':

    path = '/home/marco/Documents/BoschChallenge/BFMC_Main/source/templates/Competition_track.graphml'
    #path = '/home/marco/Documents/BoschChallenge/BFMC_Main/source/templates/Test_track.graphml'

    Graph = GraphMap(path)
    
    x = 0
    y = 0
    num = 5

    options = Graph.get_location_points(x , y , num)

    print(options)

    print(Graph.get_distance("561", "559"))

    path, length = Graph.get_path("561", "563")
    print(path)
    print(length)
    