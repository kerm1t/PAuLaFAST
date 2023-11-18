import open3d as o3d
import numpy as np
pcl = o3d.geometry.PointCloud()
pcl.points = o3d.utility.Vector3dVector(np.random.randn(5000,3))
o3d.visualization.draw_geometries([pcl])

#numpy_points = np.asarray(pcl.points)
# work on points
#pcl.points = numpy_points
#o3d.io.write_point_cloud(r'c:/git/paulafast/scripts/out_compressed.pcd', pcl)#, write_ascii=False, compressed=True, print_progress=False)
#o3d.io.write_point_cloud(r'c:/git/paulafast/scripts/out_compressed.pcd', pcl)
o3d.io.write_point_cloud("out_compressed.pcd", pcl, write_ascii=False, compressed=True, print_progress=False)
