from math import *
from Node import *
import heapq
import time

def astar(start,end,key_table):
    end_node = Node(None,key_table, end)
    end_node.g = end_node.h = end_node.f = 0
    start_node = Node(None,key_table, start)
    start_node.g = 0
    start_node.h = (((abs(start_node.x - end_node.x) * length) ** 2 +
                    (abs(start_node.y - end_node.y) * width) ** 2 +
                    abs(start_node.z - end_node.z) ** 2)**0.5)* Max_speed
    start_node.f = start_node.h + start_node.g

    open_list = []
    open_list_node={}
    open_list_node[start_node.key]=start_node
    visited_list = {}

    heapq.heapify(open_list)
    heapq.heappush(open_list, start_node)

    while len(open_list) > 0:

        current_node = heapq.heappop(open_list)
        del open_list_node[current_node.key]
        visited_list[current_node.key]=current_node

        if current_node.position == end_node.position:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]

        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:

            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            if node_position[0] > (size_x - 1) or node_position[0] < 0 or node_position[1] > (size_y - 1) or \
                    node_position[1] < 0:
                continue
            if vitesse[tuple(terrain[node_position[1], node_position[0]])] == 0:
                continue

            new_node = Node(current_node,key_table, node_position)
            if (new_node.key in visited_list) :
               continue

            children.append(new_node)

        for child in children:

            child.g = current_node.g + (((abs(child.x - current_node.x) * length) ** 2 +
                                        (abs(child.y - current_node.y) * width) ** 2 +
                                         abs(child.z - current_node.z) ** 2)**0.5)*pow(child.v, -1)

            #child.h = (((abs(child.x - end_node.x) * length) ** 2 +
                        #(abs(child.y - end_node.y) * width) ** 2 +
                        #abs(child.z - end_node.z) ** 2)**0.5)* Max_speed

            dx = abs(current_node.x- end_node.x)*length
            dy = abs(current_node.y- end_node.y)*width
            dz = abs(current_node.z- end_node.z)
            dmin = min(dx, dy)
            dmax = max(dx, dy)
            child.h = (dx + dy + dz - dmin - dmax)*Max_speed

            child.f = child.g + child.h

            if (child.key in open_list_node):
                idx = open_list.index(child)
                if child.g < open_list[idx].g:

                    open_list[idx].g = child.g
                    open_list[idx].f = child.f
                    open_list[idx].h = child.h
            else:

                heapq.heappush(open_list, child)
                open_list_node[child.key]=child



key_table= {}
key = 0

for y in range(0,terrain.shape[0]+1):
    for x in range(0,terrain.shape[1]+1):
        key_table[x, y] = key
        key = key + 1

path_list=input("Give the filename's list of points to reach : ")
path_list=np.genfromtxt(path_list)

output_filename=input("Give the filename of the ouputmap  : ")
final_path=[]
start_time = time.time()
for i in range(0,len(path_list)-1):
    path=astar(tuple(path_list[i].astype(int)), tuple(path_list[i+1].astype(int)),key_table)
    final_path.append(path)

def flatten(l):
    return [item for sublist in l for item in sublist]
final_path=flatten(final_path)

def distance (p1,p2) :
    distance = ((abs(p1[0]-p2[0])*length)**2 +(abs(p1[1]-p2[1])*width)**2+
            abs((elevation[p1[1],p1[0]])-(elevation[p2[1],p2[0]]))**2)**0.5
    return(distance)

total_dis=0
for node_position1,node_position2 in zip(final_path, final_path[1:]):
    cv2.line(terrain, node_position1, node_position2, (255,0,0), 1)
    total_dis=distance(node_position1,node_position2)+total_dis

text= "Distance : "+str(total_dis)+" meters"
cv2.putText(img=terrain, text=text, org=(20, 20),fontFace=cv2.FONT_HERSHEY_DUPLEX,fontScale=0.5, color=(0, 255, 0),thickness=1)

cv2.imwrite(output_filename,cv2.cvtColor(terrain, cv2.COLOR_RGB2BGR))
stop_time = time.time()
delta=stop_time-start_time
print("Path found in",delta,"s")
cv2.imshow("Terrain Map",cv2.cvtColor(terrain, cv2.COLOR_RGB2BGR))
cv2.waitKey(0)
cv2.destroyAllWindows()