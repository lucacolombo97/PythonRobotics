import math
from Point import Point
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import from_levels_and_colors

M2G = 0
BF = 1
BF2LP = 2


class Map:

    def __init__(self, x_max, y_max, res, start, end, obstacles):
        self.x_max = x_max
        self.y_max = y_max
        self.res = res
        self.start = Point(math.trunc(start.x / res), math.trunc(start.y / res))
        self.end = Point(math.trunc(end.x / res), math.trunc(end.y / res))
        self.obmap = self.obstacle_map(obstacles)

    def obstacle_map(self, obstacles):
        num_x = math.trunc(self.x_max / self.res)
        num_y = math.trunc(self.y_max / self.res)
        obmap = [[False for i in range(num_x)] for i in range(num_y)]
        for i in range(len(obstacles)):
            maxx = max(obstacles[i].get_vect_x())
            maxy = max(obstacles[i].get_vect_y())
            minx = min(obstacles[i].get_vect_x())
            miny = min(obstacles[i].get_vect_y())
            x, y = minx, miny
            while x <= maxx:
                while y <= maxy:
                    if self.is_point_in_polygon(x, y, obstacles[i]) and \
                            not obmap[math.trunc(x / self.res)][math.trunc(y / self.res)]:
                        obmap[math.trunc(x / self.res)][math.trunc(y / self.res)] = True
                    y += 0.1
                y = miny
                x += 0.1
        return obmap

    def is_point_in_polygon(self, x, y, polygon):
        """
        x, y -- x and y coordinates of point
        polygon -- contains a list of points [(x, y), (x, y), ...]
        """
        num = len(polygon.points)
        j = num - 1
        inside = False
        for i in range(num):
            if ((polygon.get_vect_y()[i] > y) != (polygon.get_vect_y()[j] > y)) and \
                    (x < polygon.get_vect_x()[i] + (polygon.get_vect_x()[j] - polygon.get_vect_x()[i]) *
                     (y - polygon.get_vect_y()[i]) / (polygon.get_vect_y()[j] - polygon.get_vect_y()[i])):
                inside = not inside
            j = i
        return inside

    def print_path(self, path):
        y = self.y_max - 1
        free = True
        while y >= 0:
            x = 0
            while x < self.x_max:
                pos = len(path) - 1
                while pos >= 0:
                    if path[pos].x == x and path[pos].y == y:
                        if x == self.start.x and y == self.start.y:
                            print("S", end='')
                            free = False
                            break
                        elif x == self.end.x and y == self.end.y:
                            print("E", end='')
                            free = False
                            break
                        if path[pos].type == M2G:
                            print("*", end='')
                        if path[pos].type == BF:
                            print("@", end='')
                        if path[pos].type == BF2LP:
                            print("&", end='')
                        free = False
                        break
                    pos -= 1
                if free:
                    if not self.obmap[x][y]:
                        print(".", end='')
                    else:
                        print("X", end='')
                free = True
                x += 1
            y -= 1
            print()
        print("\n")

    def draw_path(self, path):
        grid = [[0 for _ in range(math.trunc(self.x_max / self.res))] for _ in range(math.trunc(self.y_max / self.res))]
        levels = [0, 1, 2, 3, 4, 5, 6, 7]
        colors = ['white', 'green', 'orange', 'magenta', 'red', 'lightskyblue', 'blue']
        cmap, norm = from_levels_and_colors(levels, colors)
        for y in range(self.y_max - 1):
            for x in range(self.x_max - 1):
                if self.obmap[x][y]:
                    grid[y][x] = 1
        grid[self.start.y][self.start.x] = 2
        grid[self.end.y][self.end.x] = 3
        np.array(grid)
        # show graph
        for i in range(path.__len__()):
            if path[i].type == M2G:
                grid[path[i].y][path[i].x] = 4
            if path[i].type == BF:
                grid[path[i].y][path[i].x] = 5
            if path[i].type == BF2LP:
                grid[path[i].y][path[i].x] = 6
            plt.cla()
            plt.axis([0, math.trunc(self.x_max / self.res), 0, math.trunc(self.y_max / self.res)])
            plt.imshow(grid, cmap=cmap, norm=norm, interpolation=None)
            # Uncomment here to save the sequence of frames
            # plt.savefig('frame{:04d}.png'.format(i))
            plt.pause(0.0001)

        if path[path.__len__() - 1].x == self.end.x and path[path.__len__() - 1].y == self.end.y:
            plt.text(self.end.x + 1, self.end.y + 1, "GOAL REACHED")
        else:
            plt.text(self.end.x + 1, self.end.y + 1, "GOAL UNREACHABLE")
        plt.show()

    def read_pgm(self, image_path):
        """Reading from pgm file of the list of obstacles, the starting point and the end point."""
        pgmf = open(image_path, 'rb')
        format = pgmf.readline()
        (self.x_max, self.y_max) = [int(i) for i in pgmf.readline().split()]
        depth = int(pgmf.readline())
        assert depth <= 255

        self.obmap = [[False for _ in range(self.x_max)] for _ in range(self.y_max)]

        for j in range(self.y_max):
            for i in range(self.x_max):
                x = ord(pgmf.read(1))
                if x == 0:
                    self.obmap[i][self.y_max - j] = True
                if x == 50:
                    self.start.x = i
                    self.start.y = self.y_max - j
                if x == 150:
                    self.end.x = i
                    self.end.y = self.y_max - j
