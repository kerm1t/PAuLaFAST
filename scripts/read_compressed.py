import open3d as o3d
pcl = o3d.io.read_point_cloud("../data/table_scene_lms400.pcd")
#pcl = o3d.io.read_point_cloud("../data/table_scene_mug_stereo_textured.pcd")
o3d.visualization.draw_geometries([pcl])
o3d.io.write_point_cloud("table_scene_lms400.pcd", pcl, write_ascii=False, compressed=True, print_progress=False)