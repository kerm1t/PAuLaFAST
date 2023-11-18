import open3d as o3d
import numpy as np
pcl = o3d.geometry.PointCloud()
a = np.random.randn(3,3)
a = np.array([[10, 11, 12], [20, 21, 22],  [30, 31, 32], [40, 41, 42], [50, 51, 52], [60, 61, 62], [70, 71, 72], [80, 81, 82], [90, 91, 92], [100,101,102]], np.int32)
pcl.points = o3d.utility.Vector3dVector(a)
o3d.visualization.draw_geometries([pcl])


o3d.io.write_point_cloud("10pts_compr.pcd", pcl, write_ascii=False, compressed=True, print_progress=False)
