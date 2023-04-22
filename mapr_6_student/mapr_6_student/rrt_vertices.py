import rclpy
import time
from mapr_6_student.grid_map import GridMap
import numpy as np

np.random.seed(444)


class RRT(GridMap):
    def __init__(self):
        super(RRT, self).__init__()
        self.step = 0.1

    def check_if_valid(self, a, b):
        """
        Checks if the segment connecting a and b lies in the free space.
        :param a: point in 2D
        :param b: point in 2D
        :return: boolean
        """
        in_free_space = True
        x_space = np.linspace(a[0], b[0], 100)
        y_space = np.linspace(a[1], b[1], 100)
        for x, y in zip(x_space, y_space):
            x_map = int(x/self.resolution)
            y_map = int(y/self.resolution)
            if self.map[y_map, x_map] == 100:
                in_free_space = False
                break
        return in_free_space

    def random_point(self):
        """
        Draws random point in 2D
        :return: point in 2D
        """
        x = np.random.uniform(0.0, self.width)
        y = np.random.uniform(0.0, self.height)
        return (x, y)

    def find_closest(self, pos):
        """
        Finds the closest vertex in the graph to the pos argument

        :param pos: point id 2D
        :return: vertex from graph in 2D closest to the pos
        """
        min_dist = np.inf
        for node in self.parent:
            dist = np.sqrt((node[0] - pos[0])**2 + (node[1] - pos[1])**2)
            if dist < min_dist:
                min_dist = dist
                closest = node
        return closest

    def new_pt(self, pt, closest):
        """
        Finds the point on the segment connecting closest with pt, which lies self.step from the closest (vertex in graph)

        :param pt: point in 2D
        :param closest: vertex in the tree (point in 2D)
        :return: point in 2D
        """
        d = np.sqrt((pt[0] - closest[0])**2 + (pt[1] - closest[1])**2)
        x = pt[0] - closest[0]
        y = pt[1] - closest[1]
        x_s = (self.step * x) / d
        y_s = (self.step * y) / d
        new_point = (closest[0] + x_s, closest[1] + y_s)
        return new_point

    def search(self):
        """
        RRT search algorithm for start point self.start and desired state self.end.
        Saves the search tree in the self.parent dictionary, with key value pairs representing segments
        (key is the child vertex, and value is its parent vertex).
        Uses self.publish_search() and self.publish_path(path) to publish the search tree and the final path respectively.
        """
        self.parent[self.start] = None

        while True:
            # time.sleep(1)
            random_pt = self.random_point()
            closest_pt = self.find_closest(random_pt)
            new_pt = self.new_pt(random_pt, closest_pt)
            if self.check_if_valid(new_pt, closest_pt):
                self.parent[new_pt] = closest_pt
                self.publish_search()
                if self.check_if_valid(new_pt, self.end):
                    self.parent[self.end] = new_pt
                    self.publish_search() # dlaczego to nie działa, ostatnie 2 połączenia się nie wyświetlają
                    break

        # create and publish path
        path = []
        path.append(self.end)
        prev = new_pt
        while prev != None:
            path.append(prev)
            prev = self.parent[prev]
        self.publish_path(path)



def main(args=None):
    rclpy.init(args=args)
    rrt = RRT()
    while not rrt.data_received():
        rrt.get_logger().info("Waiting for data...")
        rclpy.spin_once(rrt)
        time.sleep(0.5)

    rrt.get_logger().info("Start graph searching!")
    time.sleep(1)
    rrt.search()


if __name__ == '__main__':
    main()
