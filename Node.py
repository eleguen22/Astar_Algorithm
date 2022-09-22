import numpy as np
import cv2

class Node():

    global elevation, terrain, vitesse, width, length, size_x, size_y, Max_speed

    terrain_file = input("Give the RGB map of your orienteering: ")
    terrain = cv2.imread(terrain_file)
    terrain = cv2.cvtColor(terrain, cv2.COLOR_BGR2RGB)
    elevation_file = input("Give the complete path of your elevation file : ")
    elevation = np.genfromtxt(elevation_file)
    elevation = elevation[:, :395]

    vitesse = {(248, 148, 18): 0.9, (255, 192, 0): 0.3, (255, 255, 255): 0.5, (2, 208, 60): 0.4, (2, 136, 40): 0.3,
               (5, 73, 24): 0, (0, 0, 255): 0.01,(71, 51, 3): 1, (0, 0, 0): 0.95, (205, 0, 101): 0}
    width = 7.55
    length = 10.29
    size_x = len(elevation[1])
    size_y = len(elevation[:, 1])
    Max_speed = max(vitesse.items(), key=lambda x: x[1])[1]

    def __init__(self,parent=None,keytable=None,position=None):
        self.parent = parent
        self.position = position
        self.x = x = position[0]
        self.y = y = position[1]
        self.c = tuple(terrain[y, x])
        self.z = elevation[y, x]
        self.v = vitesse[tuple(terrain[y, x])]
        self.key = keytable[position]

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __repr__(self):
        return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    def __lt__(self, other):
        return self.f < other.f

    def __gt__(self, other):
        return self.f > other.f

