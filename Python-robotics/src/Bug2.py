"""
Bug 2 algorithm
author: Luca Colombo
"""
import math
import sys
from Point import Point
from Polygon import Polygon
from Map import Map

M2G = 0
BF = 1


def get_motion_model():
    # dx, dy
    motion = [[1, 1],
              [1, 0],
              [1, -1],
              [0, -1],
              [-1, -1],
              [-1, 0],
              [-1, 1],
              [0, 1]]

    return motion


class Bug2Planner:

    def __init__(self, map):
        self.res = map.res
        self.start = map.start
        self.end = map.end
        self.obmap = map.obmap

    def verify_point(self, point):
        if self.obmap[point.x][point.y]:
            return False
        return True

    def planning(self):
        """
        current_pos -- current position x, y
        line -- coordinate of the line from the leave_point to the end_point
        leave_point -- coordinates of the point from which the robot starts (or restarts after bf)

        Main loop that chooses between 2 strategies:
        - M2G: when the robot moves towards the goal
        - BF: when the robot encounters an obstacle and must circumnavigate it
        If the robot encounters the same obstacle twice, the goal is unreachable
        """
        current_pos = Point(self.start.x, self.start.y)
        line = Point(self.start.x, self.start.y)
        leave_point = Point(self.start.x, self.start.y)
        path = []
        while True:
            if not self.motion_to_goal(leave_point, current_pos, line, path):
                if self.check_goal_unreachable(current_pos, path):
                    print("Goal unreachable")
                    break
                self.boundary_following(leave_point, current_pos, path)
            else:
                print("Goal found")
                break
        return path

    def motion_to_goal(self, leave_point, current_pos, line, path):
        dx = self.end.x - leave_point.x
        dy = self.end.y - leave_point.y
        line.x, line.y = leave_point.x, leave_point.y
        if dx < 0 or dy < 0:
            arrival_point = Point(leave_point.x, leave_point.y)
            increment = - self.res
        else:
            arrival_point = Point(self.end.x, self.end.y)
            increment = self.res
        if abs(dx) >= abs(dy):
            while line.x != arrival_point.x + 1:
                line.y = leave_point.y + abs(dy) * (line.x - leave_point.x) / abs(dx)
                value = self.calc_path(current_pos, line, path)
                if value == 0:
                    return True
                elif value == 1:
                    return False
                line.x += increment
        else:
            while line.y != arrival_point.y + 1:
                line.x = leave_point.x + abs(dx) * (line.y - leave_point.y) / abs(dy)
                value = self.calc_path(current_pos, line, path)
                if value == 0:
                    return True
                elif value == 1:
                    return False
                line.y += increment

    def boundary_following(self, leave_point, current_pos, path):
        motion = get_motion_model()
        first = True
        while True:
            for k in range(motion.__len__()):
                temp_x = current_pos.x + motion[k][0]
                temp_y = current_pos.y + motion[k][1]
                if not self.verify_point(Point(current_pos.x + motion[k - 1][0], current_pos.y + motion[k - 1][1])):
                    if self.verify_point(Point(temp_x, temp_y)):
                        current_pos.x, current_pos.y = temp_x, temp_y
                        path.append(Point(current_pos.x, current_pos.y, BF))
                        break
            if not first:
                if not self.check_bf(leave_point, current_pos, path):
                    break
            else:
                first = False
        return True

    def calc_path(self, current_pos, line, path):
        motion = get_motion_model()
        min_dist = (current_pos, math.sqrt((line.x - current_pos.x) ** 2 + (line.y - current_pos.y) ** 2))
        for i in range(motion.__len__()):
            temp_x = current_pos.x + motion[i][0]
            temp_y = current_pos.y + motion[i][1]
            dist = math.sqrt((line.x - temp_x) ** 2 + (line.y - temp_y) ** 2)
            if min_dist[1] > dist:
                min_dist = (Point(temp_x, temp_y), dist)
        if self.verify_point(min_dist[0]):
            current_pos.x, current_pos.y = min_dist[0].x, min_dist[0].y
            path.append(Point(current_pos.x, current_pos.y, M2G))
            if current_pos.x == self.end.x and current_pos.y == self.end.y:
                return 0
        else:
            return 1

    def check_bf(self, leave_point, current_pos, path):
        """
        Check if boundary_following is finished.
        For Bug2 algorithm this condition is verified when the current position crossed the line
        that joins the starting point and the ending point.

        :return: False if the robot crosses the line
        """
        dx = self.end.x - leave_point.x
        dy = self.end.y - leave_point.y
        dist_currentpoint = math.sqrt((self.end.x - current_pos.x) ** 2 + (self.end.y - current_pos.y) ** 2)
        i = path.__len__() - 1
        while i >= 0:
            if path[i].type == BF and path[i - 1].type == M2G:
                dist_hitpoint = math.sqrt((self.end.x - path[i - 1].x) ** 2 + (self.end.y - path[i - 1].y) ** 2)
                break
            i -= 1
        if abs(dx) > abs(dy):
            if current_pos.y == math.trunc(self.start.y + abs(dy) * (current_pos.x - self.start.x) / abs(dx)):
                if dist_currentpoint <= dist_hitpoint:
                    leave_point.x, leave_point.y = current_pos.x, current_pos.y
                    return False
        else:
            if current_pos.x == math.trunc(self.start.x + abs(dx) * (current_pos.y - self.start.y) / abs(dy)):
                if dist_currentpoint <= dist_hitpoint:
                    leave_point.x, leave_point.y = current_pos.x, current_pos.y
                    return False
        return True

    def check_goal_unreachable(self, current_pos, path):
        i = path.__len__() - 1
        while i >= 0:
            if path[i].type == BF and path[i - 1].type == M2G:
                if current_pos.x == path[i - 1].x and current_pos.y == path[i - 1].y:
                    return True
            i -= 1


def main():
    x_max = 100
    y_max = 100
    res = 1  # resolution
    obstacles = []

    p1 = [Point(20, 42), Point(25, 28), Point(28, 38), Point(20, 42)]
    polygon1 = Polygon(p1)

    p2 = [Point(35, 65), Point(50, 50), Point(53, 55), Point(35, 65)]
    polygon2 = Polygon(p2)

    obstacles.append(polygon1)
    obstacles.append(polygon2)

    start = Point(15, 30)
    end = Point(70, 90)

    map = Map(x_max, y_max, res, start, end, obstacles)

    if len(sys.argv) > 1:
        map.read_pgm(sys.argv[1])
    bug = Bug2Planner(map)
    path = bug.planning()
    map.draw_path(path)
    # map.print_path(path)


if __name__ == '__main__':
    main()
