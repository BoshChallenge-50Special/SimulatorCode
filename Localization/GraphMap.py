# -*- coding: utf-8 -*-

import xml.etree.ElementTree as ET
import math

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




if __name__ == '__main__':

    path = '/home/marco/Documents/BoschChallenge/BFMC_Main/source/templates/Competition_track.graphml'
    #path = '/home/marco/Documents/BoschChallenge/BFMC_Main/source/templates/Test_track.graphml'

    Graph = GraphMap(path)

    x = 1
    y = 4
    num = 5

    options = Graph.get_location_points(x , y , num)

    print(options)

    