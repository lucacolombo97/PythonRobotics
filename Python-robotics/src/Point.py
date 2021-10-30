class Point:

    def __init__(self, x, y, type=-1):
        """
        x, y -- x and y coordinates of point
        type -- indicates if the point is part of motion-to-goal or boundary-following:
        - 0: m2g
        - 1: bf
        """
        self.x = x
        self.y = y
        self.type = type
