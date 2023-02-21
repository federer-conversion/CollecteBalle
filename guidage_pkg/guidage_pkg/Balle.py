class Balle():

    def __init__(self, x=0, y=0):
        self.pose = (x, y)
        self.age = 0
        self.tour_pas_vue = 0

    def get_pose(self):
        return self.pose

    def set_pose(self, x, y):
        self.pose = (x, y)

    def vieillir(self):
        self.age += 1

    def __str__(self):
        return "balle en {0}, présente depuis {1} frames".format(self.pose, self.age)

    def __eq__(self, balle):
        return (((balle.get_pose()[0]*1.0-self.pose[0]*1.0)**2+(balle.get_pose()[1]*1.0-self.pose[1]*1.0)**2)**0.5) < 14
