"""
CSCI-630: Foundation of AI
lab1 Code - lab1.py
Author: Prakhar Gupta
Username: pg9349

An implementation of A*

Code uses terraingraph.py and pixelvertex.py and Pixel.py

"""

from PIL import Image
from Pixel import Pixel
from Terraingraph import Terraingraph
import math
import numpy as np
from queue import PriorityQueue
import heapq
import queue
from time import time
import sys


def read_img(filename):
    """
    Reads the image file
    :param filename File name

    :return: A list of list of pixels width and height
    """
    im = Image.open(filename)

    pixels = im.convert("RGB").load() # this is not a list, nor is it list()'able
    width, height = im.size

    c=0
    x=[]
    image_terr=[]
    for i in range(height):
        rows = []
        for j in range(width):
            rows.append(pixels[j,i])
            c+=1
            x.append(pixels[j,i])
        image_terr.append(rows)

    return image_terr,pixels,width,height,x







def getNeighbour( x, y, rows_max, columns_max, pixels):
    """
    Finds the Neighbour for each pixel (which is not out of bounds)
    :param filename File name

    :return: A dict of neigbours of a pixel
    """
    final_neighbours = []
    end_columns = columns_max
    end_rows = rows_max

    neighbours = [(1, 0), (0, 1), (1, 1), (-1, 1)   , (1, -1), (-1, -1), (0, -1), (-1, 0)]

    for values in neighbours:
        pt_1_x = x + values[0]
        pt_1_y = y + values[1]
        #print(pt_1_y)
        #print(pt_1_x)
        if pt_1_x >= 0 and\
                        pt_1_x < end_rows and\
                        pt_1_y >= 0 and\
                        pt_1_y < end_columns and\
                        pixels[pt_1_x] [pt_1_y] != (205, 0, 101) :
            final_neighbours.append((pt_1_x, pt_1_y))
    return final_neighbours




def pixelmatrix(l,elevation):
    """
    Creates a pixel matrix (list of list of pixel class object to store relevant information)
    :param l elevation

    :return: A list of list of pixels objects
    """

    openland =      (248, 148, 18)
    roughmeadow =   (255, 192, 0)
    road =          (71, 51, 3)
    footpath =      (0, 0, 0)
    easyforest =    (255, 255, 255)
    slowforest =    (2, 208, 60)
    walkforest =    (2, 136, 40)
    impassable =    (5, 73, 24)
    lake =         (0, 0, 255)
    outofbounds =   (205, 0, 101)

    info={
        openland:[8,"openland",(248, 148, 18)],
        roughmeadow:[7,"roughmeadow",(255, 192, 0)],
        road:[10,"road",(71, 51, 3)],
        footpath:[9,"footpath",(0, 0, 0)],
        easyforest:[6,"easyforest",easyforest],
        slowforest:[5,"slowforest",slowforest],
        walkforest :[4,"walkforest",walkforest],
        impassable:[3,"impassable",impassable],
        lake:[2,"lake",lake],
        outofbounds:[0.1,"outofbounds",outofbounds]
    }
    pmatrix=[]

    for i in range(len(l)):
        rows=[]
        for j in range(len(l[i])):
            pixel=l[i][j]
            elevate=elevation[i][j]
            if pixel in info.keys():
                apixel=Pixel(info[pixel][0],info[pixel][1],info[pixel][2],elevate)
                #print(apixel)
                rows.append(apixel)
        pmatrix.append(rows)

    return pmatrix

def cost_function(pixela,pixelb,corda,cordb, Time=True):
    """
    Cost function
    if time = true calculates time else the euclidean distance between two points


    :return: Cost/ Distance
    """


    speed =pixelb.speed
    x1=corda[0]
    y1=corda[1]
    x2=cordb[0]
    y2=cordb[1]

    distx=((x1-x2)*7.55)**2
    disty = ((y1 - y2) * 10.29)**2
    distz=(pixela.el-pixelb.el)**2

    if Time==False:
        return math.sqrt((distx+disty+distz))
    else:
        return math.sqrt((distx+disty+distz))/pixelb.speed




def make_terrain_graph(graph_list,pixelmatrix):
    """
    Creates a graph using the adjacency list

    :param graph_list Dict of nodes and vertices of pixels and neigbours


    :return: terrain graph  object
    """
    terrain_graph= Terraingraph()
    for nodes, neighbors in graph_list.items():
        for neighbor in neighbors:
            if neighbor is not None:
                # this automatically creates a new vertex if not already present
                sourcepixel=pixelmatrix[nodes[0]][nodes[1]]
                destpixel = pixelmatrix[neighbor[0]][neighbor[1]]
                costt=cost_function(sourcepixel,destpixel,nodes,neighbor)
                costd = cost_function(sourcepixel, destpixel, nodes, neighbor,Time=False)
                terrain_graph.addEdge(nodes, sourcepixel, neighbor,destpixel,[costt,costd])
    return terrain_graph


def read_elevation(filename):
    """
    Reads elevation file
    :param file name


    :return: elevation details as list of list
    """

    lines=[]
    with open(filename) as f:
        # reading the elevations data
        for line in f:
            line = line.strip()
            l = line.split()[:-5]
            for i in range(len(l)):
                l[i] = float(l[i])
            lines.append(l)
    #print(str(len(l))+" : "+str(len(lines)))
    return lines


def astar(terrain_graph,startcoord,stopcoord,pmatrix):
    """
    A* implementation using priority queue
    :param terrain_graph,startcoord,stopcoord,pmatrix
        terrain graph
        start coordinate
        stop coordinate
        list of list of pixel objects


    :return: list of vertexs which includes optimal path
    """

    pixelstart=pmatrix[startcoord[0]][startcoord[1]]
    pixelstop = pmatrix[stopcoord[0]][stopcoord[1]]
    start=terrain_graph.getVertex(startcoord)
    stop=terrain_graph.getVertex(stopcoord)
    # if x is not None:
    #     print(x.connectedTo)
    c=0;
    open={start}


    closed=set()
    gscore={start:0}
    pathdict={start:start}
    ## We divide by speed of road which has the highest value
    hscore={start:cost_function(pixelstart,pixelstop,startcoord,stopcoord,False)/10}
    fscore={start:gscore[start]+hscore[start]}


    ## Priority Queue
    p_queue = queue.PriorityQueue()
    p_queue.put((fscore[start],c, start))
    c+=1 ## value to consider if both vertex have same f(x)
    while not p_queue.empty():
        if not p_queue.empty():
            # getting least f(x) item since vertex is the third element we have use indexing as 2
            current=p_queue.get()[2]
        #print(current)


        if current is None:
            print("No path")
            return None

        if current==stop:
            path=[]
            distance=0
            while pathdict[current]!=current:
                path.append(current)
                current=pathdict[current]

            path.append(start)
            #path[::-1]
            return path[::-1]

        for k,v in current.connectedTo.items():
            if k not in open  and k not in closed:
                open.add(k)
                pathdict[k]=current

                gscore[k]=gscore[current]+current.getWeight(k)[0]
                hscore[k]=cost_function(k.p,pixelstop,k.id,stopcoord,False)/10
                fscore[k]=gscore[k]+hscore[k]

                p_queue.put((fscore[k],c,k))
                c+=1


            else:
                if gscore[k]>gscore[current]+current.getWeight(k)[0]:
                    gscore[k] = gscore[current] + current.getWeight(k)[0]
                    pathdict[k]=current

                    if k in closed:
                        closed.remove(k)

                        p_queue.put((fscore[k],c, k))
                        c+=1
                        open.add(k)

        open.remove(current)
        closed.add(current)

    print("no path")
    return None

def read_checkpoint(filename):
    """
    Reading checkpoint file


    :return: list of checkpoints
    """


    x=[]
    with open(filename) as f:
        for line in f:
            line = line.strip()
            l = line.split()
            l.reverse()
            x.append(l)
    return x

def create_image(filename,final_path,image_terr,width, height):

    """

    This function creates a final image
    :param filename: output file name
    :param final_path: pah
    :param image_terr: pixel list
    :param width: width of image
    :param height: height of image
    :return:
    """
    for i in final_path:
        image_terr[i.id[0]][i.id[1]]=(206,50,76)

    pixel_image=[]
    for i in image_terr:
        pixel_image.extend(i)

    im2 = Image.new("RGB", (width, height))
    im2.putdata(pixel_image)
    im2.save(filename)




def let_find_the_path(imgfile,elevationfile,pathfile,outputfile):
    """
    Driver function

    :param imgfile: image file to be read
    :param elevationfile:  elevation file
    :param pathfile: checkpoint or path file
    :param outputfile: output image name
    :return:
    """

    start_time = time()
    image_terr,pixels,width,height,x=read_img(imgfile)
    d={}
    for i in range(len(image_terr)):

        for j in range(len(image_terr[i])):

            d[(i,j)]=getNeighbour( i, j, height, width, image_terr)
            #print(d[(i,j)])

    elevation=read_elevation(elevationfile)
    pmatrix=pixelmatrix(image_terr,elevation)

    graph_list=d
    terrain_graph=make_terrain_graph(graph_list, pmatrix)

    checkpoints=read_checkpoint(pathfile)
    c = 0
    final_path=[]
    for i in range(0,len(checkpoints)-1):
        start=(int(checkpoints[i][0]),int(checkpoints[i][1]))
        stop=(int(checkpoints[i+1][0]),int(checkpoints[i+1][1]))
        path = astar(terrain_graph, start,stop, pmatrix)
        final_path.extend(path)
        for i in range(0,len(path)-1):
            c+=path[i].getWeight(path[i+1])[1]

    print("Distance of path= "+str(c))

    runtime = f"{(time() - start_time):0.4f} seconds"
    print("It took " + runtime)

    print("exporting image")
    create_image(outputfile,final_path,image_terr,width,height)

if __name__ == '__main__':
    #print(sys.argv)
    if len(sys.argv)==5:
        a=sys.argv[1]
        b = sys.argv[2]
        c = sys.argv[3]
        d = sys.argv[4]
        let_find_the_path(a, b, c, d)
    else:
        print("Check args list")






