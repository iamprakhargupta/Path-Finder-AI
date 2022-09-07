"""
CSCI-630: Foundation of AI
vertex.py
Modified by: Prakhar Gupta
Username: pg9349

Credits :
Author: Sean Strout @ RIT CS


This code of making vertex of a graph is borrowed from CS 603 instructor's code

"""

from Pixel  import Pixel


class Pixelvertex:
    """
    An individual vertex in the graph.

    :slots: id:  The identifier for this vertex (user defined, typically
        a string)
    :slots: connectedTo:  A dictionary of adjacent neighbors, where the key is
        the neighbor (Vertex), and the value is the edge cost (int)
    """



    def __init__(self, key,pixel):
        """
        Initialize a vertex
        :param key: The identifier for this vertex
        :return: None
        """
        self.id = key
        self.connectedTo = {}
        self.p=pixel


    def addNeighbor(self, nbr, weight=[]):
        """
        Connect this vertex to a neighbor with a given weight (default is 0).
        :param nbr (Vertex): The neighbor vertex
        :param weight (int): The edge cost
        :return: None
        """
        self.connectedTo[nbr] = weight

    def __str__(self):
        """
        Return a string representation of the vertex and its direct neighbors:

            vertex-id connectedTo [neighbor-1-id, neighbor-2-id, ...]

        :return: The string
        """
        return str(self.id) + " pixel details="+str(self.p) +' connectedTo: ' + str([str(x.id)+ " time taken = "+str(y[0]) +" distance ="+str(y[1]) for x,y in self.connectedTo.items()])

    def getConnections(self):
        """
        Get the neighbor vertices.
        :return: A list of Vertex neighbors
        """
        return self.connectedTo.keys()

    def getWeight(self, nbr):
        """
        Get the edge cost to a neighbor.
        :param nbr (Vertex): The neighbor vertex
        :return: The weight (int)
        """
        return self.connectedTo[nbr]
