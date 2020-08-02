class Point:
    """
    2d point
    """
    id_counter = 0
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.id = Point.id_counter + 1
        Point.id_counter += 1
    
    def __str__(self):
        s="ID=%d x='%d' y='%d'" % (self.id,self.x,self.y) 
        return s 

    @staticmethod
    def euclidean_distance(p1, p2):
        sqr = (p1.x - p2.x)**2 + (p1.y-p2.y)**2
        r = sqr**0.5
        return r