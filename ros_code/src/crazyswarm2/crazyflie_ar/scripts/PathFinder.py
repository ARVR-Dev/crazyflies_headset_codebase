import numpy as np
from pathfinding3d.core.diagonal_movement import DiagonalMovement
from pathfinding3d.core.grid import Grid
from pathfinding3d.finder.a_star import AStarFinder
from pathfinding3d.finder.bi_a_star import BiAStarFinder
from pathfinding3d.finder.theta_star import ThetaStarFinder
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

class PathFinder:
    def __init__(self, grid_unit, resolution, grid_size):
        """
        Initialize the PathFinder class.
        :param grid_unit: the grid size corresponding to the actual unit length.
        :param resolution: resolution of OctoMap.
        :param grid_size: size of the target numpy array in (width, height, depth) format.
        :x_range,y_range,z_range are the boundaries of map
        """
        self.grid_unit = grid_unit
        self.resolution = resolution
        self.grid_size = grid_size
        self.x_range = [-2.82, 3.18]
        self.y_range = [-4.86, 4.35]
        self.z_range = [-0.1, 3]
    
    def load_point_cloud(self, file_path):
        point_cloud = o3d.io.read_point_cloud(file_path)
        return point_cloud
    
    def filter_point_cloud(self, point_cloud):
        """
        Filter the point cloud to keep only the points within the specified range.

        :param point_cloud: The point cloud object of Open3D.
        :return: The filtered point cloud object.
        """
        points = np.asarray(point_cloud.points)
    
        # Filter out points our of range
        mask = (
            (points[:, 0] >= self.x_range[0]) & (points[:, 0] <= self.x_range[1]) &
            (points[:, 1] >= self.y_range[0]) & (points[:, 1] <= self.y_range[1]) &
            (points[:, 2] >= self.z_range[0]) & (points[:, 2] <= self.z_range[1])
        )
    
        # Save points satified the condition 
        filtered_points = points[mask]
        #
        filtered_pcd = o3d.geometry.PointCloud()
        filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
        return filtered_pcd
    
    def point_cloud_to_numpy(self, point_cloud, x_min, y_min, z_min):
        """
        Translate point cloud info to occupancy map

        """
        occupancy_map = np.ones(self.grid_size, dtype=np.int8)
        for point in np.asarray(point_cloud.points):
            # Coordinate transformation
            i = int((point[0] - x_min) / self.grid_unit)
            j = int((point[1] - y_min) / self.grid_unit)
            k = int((point[2] - z_min) / self.grid_unit)
        
            # x->z, y->x, z->y
            # temp = i
            # i = k
            # k = temp
        
            if 0 <= i < self.grid_size[0] and 0 <= j < self.grid_size[1] and 0 <= k < self.grid_size[2]:
                occupancy_map[i, j, k] = 0  # Add occupancy

        return occupancy_map

    def find_path(self, occupancy_map, start_pos, end_pos):
        grid = Grid(matrix=occupancy_map)
        

        start = grid.node(
            int((start_pos[0] - self.x_range[0]) / self.grid_unit),
            int((start_pos[1] - self.y_range[0]) / self.grid_unit),
            int((start_pos[2] - self.z_range[0]) / self.grid_unit)
            )
        end = grid.node(
            int((end_pos[0] - self.x_range[0]) / self.grid_unit),
            int((end_pos[1] - self.y_range[0]) / self.grid_unit),
            int((end_pos[2] - self.z_range[0]) / self.grid_unit)
        )

        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start, end, grid)

        # Visualization of occupancy map
        # grid.visualize(
        #     path=path,  # optionally visualize the path
        #     start=start,
        #     end=end,
        #     visualize_weight=True,  # weights above 1 (default) will be visualized
        #     save_html=True,  # save visualization to html file
        #     save_to="path_visualization.html",  # specify the path to save the html file
        #     always_show=True,  # always show the visualization in the browser
        #     )

        # Transfer the path back to reality coordinate
        real_path = [
            (
                p.x * self.grid_unit + self.x_range[0],
                p.y * self.grid_unit + self.y_range[0],
                p.z * self.grid_unit + self.z_range[0]
            )
            for p in path
        ]

        return real_path

if __name__ == "__main__":
    start_time = time.time()
    grid_unit = 0.1  # m

    x_size = int(6/grid_unit)
    y_size = int(9/grid_unit)
    z_size = int(3/grid_unit)
    grid_size = (x_size, y_size, z_size)  
    
    pathfinder = PathFinder(grid_unit, resolution=None, grid_size=grid_size)
    
    # Load the point cloud
    pcd = pathfinder.load_point_cloud("/home/cpsl/CrazySim/ros2_ws/src/crazyflie_mpc/point cloud/Vicon1.ply")
    
    
    # Filter out points out of range
    filtered_pcd = pathfinder.filter_point_cloud(pcd)
    
    # Translate to occupancy map
    occupancy_map = pathfinder.point_cloud_to_numpy(filtered_pcd, x_min=-2.82, y_min=-4.86, z_min=0.0)

    # Identified start and end
    start_pos = (-2.7, 4, 0.05)
    end_pos = (2.5, -2.6, 0.05)

    
    # Find path
    path = pathfinder.find_path(occupancy_map, start_pos, end_pos)
    end_time = time.time()
    run_time = end_time - start_time
    print(run_time)
    
    # Figure
    x_coords = [p[0] for p in path]
    y_coords = [p[1] for p in path]
    z_coords = [p[2] for p in path]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_coords, y_coords, z_coords, marker='o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()
    
    
