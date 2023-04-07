import rclpy
import time
from mapr_4_student.grid_map import GridMap
import queue


class BFS(GridMap):
    def __init__(self):
        super(BFS, self).__init__()
        ###  IF YOU NEED SOME ADDITIONAL FILEDS IN BFS OBJECT YOU CAN INITIALIZED THEM HERE

    def search(self):
        ### YOUR CODE GOES BELOW
        #
        #
        #
        # IMPLEMENT BREADTH FIRST SEARCH:
        # * save your search in self.map.data
        # * use self.publish_visited() to publish the map every time you visited a new cell
        # * let 100 represent walls, 50 visited cells (useful for visualization)
        # * start point is in self.start
        # * end point is in self.end
        #
        #
        ### YOUR CODE GOES ABOVE
        q = queue.Queue()
        visited = set()
        dirs = [(0, -1), (-1, 0), (0, 1), (1, 0)] # S, W, N, E
        parents = {}

        for x in range(self.map.info.height):
            for y in range(self.map.info.width):
                parents[(x, y)] = None

        q.put(self.start)

        while not q.empty():
            curr_n = q.get()
            visited.add(curr_n)
            self.map.data[curr_n[1] * self.map.info.width + curr_n[0]] = 50
            self.publish_visited()

            if curr_n == self.end:
                self.get_logger().info("Found final vertex!")
                break
            else:
                for dir in dirs:
                    next_n = (curr_n[0] + dir[0], curr_n[1] + dir[1])
                    if (next_n not in visited) and (self.map.data[next_n[1] * self.map.info.width + next_n[0]] != 100) and (next_n not in list(q.queue)):
                        q.put(next_n)
                        parents[next_n] = curr_n
                        

def main(args=None):
    rclpy.init(args=args)
    bfs = BFS()
    while not bfs.data_received():
        bfs.get_logger().info("Waiting for data...")
        rclpy.spin_once(bfs)
        time.sleep(0.5)

    bfs.get_logger().info("Start graph searching!")
    bfs.publish_visited()
    time.sleep(1)
    bfs.search()

if __name__ == '__main__':
    main()
