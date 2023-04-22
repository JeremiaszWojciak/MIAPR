import rclpy
import time
from mapr_6_student.grid_map import GridMap
import numpy as np

np.random.seed(444)


class RRT(GridMap):
    def __init__(self):
        super(RRT, self).__init__()

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
            x = int(x/self.resolution)
            y = int(y/self.resolution)
            if self.map[y, x] == 100:
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
        Finds the closest point in the graph (closest vertex or closes point on edge) to the pos argument
        If the point is on the edge, modifies the graph to obtain the valid graph with the new point and two new edges
        connecting existing vertices

        :param pos: point id 2D
        :return: point from graph in 2D closest to the pos
        """
        min_dist = np.inf
        for node in self.parent:
            dist = np.sqrt((node[0] - pos[0])**2 + (node[1] - pos[1])**2)
            if dist < min_dist:
                min_dist = dist
                closest = node

        prev = None
        next = None
        for node in self.parent:
            parent = self.parent[node]
            if parent != None:
                a = (node[1] - parent[1]) / (node[0] - parent[0])
                b = node[1] - a * node[0]
                a_per = - 1/a
                b_per = pos[1] - a_per * pos[0]
                a_arr = np.array([[-a, 1], [-a_per, 1]])
                b_arr = np.array([b, b_per])
                p = np.linalg.solve(a_arr, b_arr)
                pt_on_line = (p[0], p[1])
                if min(parent[0], node[0]) < pt_on_line[0] < max(parent[0], node[0]) and min(parent[1], node[1]) < pt_on_line[1] < max(parent[1], node[1]):
                    A = -a
                    B = 1
                    C = -b
                    dist_from_line = abs(A*pos[0] + B*pos[1] + C) / np.sqrt(A**2 + B**2)
                    if dist_from_line < min_dist:
                        min_dist = dist_from_line
                        closest = pt_on_line
                        prev = parent
                        next = node

        return closest, prev, next

    def new_pt(self, pt, closest):
        """
        Finds last point in the free space on the segment connecting closest with pt

        :param pt: point in 2D
        :param closest: vertex in the tree (point in 2D)
        :return: point in 2D
        """
        x_space = np.linspace(closest[0], pt[0], 100)
        y_space = np.linspace(closest[1], pt[1], 100)
        for x, y in zip(x_space, y_space):
            x_map = int(x/self.resolution)
            y_map = int(y/self.resolution)
            if self.map[y_map, x_map] == 0:
                new_point = (x, y)
            else:
                break
        if new_point == closest:
            new_point = None
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
            random_pt = self.random_point()
            closest_pt, prev_pt, next_pt = self.find_closest(random_pt)
            if self.check_if_valid(random_pt, closest_pt):
                new_pt = random_pt
            else:
                new_pt = self.new_pt(random_pt, closest_pt)

            if new_pt != None:
                if closest_pt not in self.parent:
                    self.parent[closest_pt] = prev_pt
                    self.parent[next_pt] = closest_pt
                self.parent[new_pt] = closest_pt
                self.publish_search()

                if self.check_if_valid(new_pt, self.end):
                    self.parent[self.end] = new_pt
                    self.publish_search()
                    break
                self.get_logger().info(f" Len parent: {len(self.parent)}")

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

