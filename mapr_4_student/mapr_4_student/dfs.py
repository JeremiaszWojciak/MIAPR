import rclpy
import time
from mapr_4_student.grid_map import GridMap
import queue


class DFS(GridMap):
    def __init__(self):
        super(DFS, self).__init__()
        ###  IF YOU NEED SOME ADDITIONAL FIELDS IN DFS OBJECT YOU CAN INITIALIZE THEM HERE

    def search(self):
        ### YOUR CODE GOES BELOW
        #
        #
        #
        # IMPLEMENT DEPTH FIRST SEARCH:
        # * save your search in self.map.data
        # * use self.publish_visited() to publish the map every time you visited a new cell
        # * let 100 represent walls, 50 visited cells (useful for visualization)
        # * start point is in self.start
        # * end point is in self.end
        #
        #
        ### YOUR CODE GOES ABOVE
        q = queue.LifoQueue()
        visited = set()
        dirs = [(0, -1), (-1, 0), (0, 1), (1, 0)] # S, W, N, E

        q.put(self.start)

        while not q.empty():
            curr_n = q.queue[-1]
            if curr_n not in visited:
                visited.add(curr_n)
                self.map.data[curr_n[1] * self.map.info.width + curr_n[0]] = 50
                self.publish_visited()

            if curr_n == self.end:
                self.get_logger().info("Found final vertex!")
                break
            else:
                for dir in dirs:
                    next_n = (curr_n[0] + dir[0], curr_n[1] + dir[1])
                    if (next_n not in visited) and self.map.data[next_n[1] * self.map.info.width + next_n[0]] != 100:
                        q.put(next_n)
                        break
                    else:
                        if dir == dirs[-1]:
                            q.get()

def main(args=None):
    rclpy.init(args=args)
    dfs = DFS()
    while not dfs.data_received():
        dfs.get_logger().info("Waiting for data...")
        rclpy.spin_once(dfs)
        time.sleep(0.5)

    dfs.get_logger().info("Start graph searching!")
    dfs.publish_visited()
    time.sleep(1)
    dfs.search()

if __name__ == '__main__':
    main()
