class Polygon:

    def __init__(self, points):
        self.points = points

    def get_vect_x(self):
        vect = []
        for i in range(len(self.points)):
            vect.append(self.points[i].x)
        return vect

    def get_vect_y(self):
        vect = []
        for i in range(len(self.points)):
            vect.append(self.points[i].y)
        return vect
