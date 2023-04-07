import rclpy
import time
from mapr_5_student.grid_map import GridMap
import heapq as pq
import numpy as np


class ASTAR(GridMap):
    def __init__(self):
        super(ASTAR, self).__init__('astar_node')

    def heuristics(self, pos):
        ### YOUR CODE GOES BELOW
        #
        #
        # IMPLEMENT HEURISTICS WITH MANHATTAN METRIC:
        # * follow the formula from the instruction
        #
        #
        ### YOUR CODE GOES ABOVE

        # Manhattan distance 
        # distance = abs(pos[0] - self.end[0]) + abs(pos[1] - self.end[1])

        # Euclidean distance
        # distance = np.sqrt((pos[0] - self.end[0])**2 + (pos[1] - self.end[1])**2)

        # Chebyshev distance
        distance = max([abs(pos[0] - self.end[0]), abs(pos[1] - self.end[1])])

        return distance

    def search(self):
        ### YOUR CODE GOES BELOW
        #
        #
        # IMPLEMENT A* SEARCH ALGORITHM:
        # * save your search in self.map.data
        # * use self.publish_visited() to publish the map every time you visited a new cell
        # * let 100 represent walls, 50 visited cells (useful for visualization)
        # * save the path to the goal found by the algorithm to list of tuples: [(x_n, y_n), ..., (x_2, y_2), (x_1, y_1)]
        # * use self.publish_path(path) to publish the path at the very end
        # * start point is in self.start
        # * end point is in self.end
        #
        #
        ### YOUR CODE GOES ABOVE
        q = []
        visited = set()
        dirs = [(0, -1), (-1, 0), (0, 1), (1, 0)] # S, W, N, E
        parents = {}
        costs = {}

        for x in range(self.map.info.height):
            for y in range(self.map.info.width):
                parents[(x, y)] = None
                costs[(x, y)] = None

        costs[self.start] = 0
        pq.heappush(q, (costs[self.start] + self.heuristics(self.start), self.start))

        while len(q) > 0:
            (priority, curr_n) = pq.heappop(q)
            visited.add(curr_n)
            self.map.data[curr_n[1] * self.map.info.width + curr_n[0]] = 50
            self.publish_visited()

            if curr_n == self.end:
                self.get_logger().info("Found final vertex!")
                break
            else:
                for dir in dirs:
                    next_n = (curr_n[0] + dir[0], curr_n[1] + dir[1])
                    if (next_n not in visited) and (self.map.data[next_n[1] * self.map.info.width + next_n[0]] != 100):
                        next_cost = costs[curr_n] + 1
                        if next_n not in [i[1] for i in q]:
                            parents[next_n] = curr_n
                            costs[next_n] = next_cost
                            pq.heappush(q, (next_cost + self.heuristics(next_n), next_n))
                        else:
                            if next_cost < costs[next_n]:
                                parents[next_n] = curr_n
                                costs[next_n] = next_cost
                                q = [i for i in q if next_n != i[1]]
                                pq.heappush(q, (next_cost + self.heuristics(next_n), next_n))

        path = []
        while curr_n != None:
            path.append(curr_n)
            curr_n = parents[curr_n]

        self.publish_path(reversed(path))


def main(args=None):
    rclpy.init(args=args)
    astar = ASTAR()
    while not astar.data_received():
        astar.get_logger().info("Waiting for data...")
        rclpy.spin_once(astar)
        time.sleep(0.5)

    astar.get_logger().info("Start graph searching!")
    astar.publish_visited()
    time.sleep(1)
    astar.search()

if __name__ == '__main__':
    main()