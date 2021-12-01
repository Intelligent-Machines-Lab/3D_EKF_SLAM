import open3d as o3d
import numpy as np



def voxel_grid_to_pcd(voxel_grid, n_points=50):
    box_structure = []
    pcd_structure = []
    point_cloud_np = np.asarray([voxel_grid.origin + pt.grid_index*voxel_grid.voxel_size for pt in voxel_grid.get_voxels()])


    for voxel in voxel_grid.get_voxels():

        mesh_box = o3d.geometry.TriangleMesh.create_box(width=voxel_grid.voxel_size,
                                                        height=voxel_grid.voxel_size,
                                                        depth=voxel_grid.voxel_size)
        mesh_box.paint_uniform_color(voxel.color)
        mesh_box.translate(voxel_grid.origin + voxel.grid_index*voxel_grid.voxel_size)
        pcd_node = mesh_box.sample_points_uniformly(number_of_points=n_points)
        box_structure.append(mesh_box)
        pcd_structure.append(pcd_node)
        
    pcd_map = o3d.geometry.PointCloud()
    pcd_map.points = o3d.utility.Vector3dVector([])
    for x in range(len(pcd_structure)):
        pt_antigo = np.asarray(pcd_map.points)
        pt_novo = np.asarray(pcd_structure[x].points)
        cor_antiga = np.asarray(pcd_map.colors)
        cor_nova = np.asarray(pcd_structure[x].colors)
        pcd_map.points = o3d.utility.Vector3dVector(np.append(pt_antigo, pt_novo, axis=0))
        pcd_map.colors = o3d.utility.Vector3dVector(np.append(cor_antiga, cor_nova, axis=0))
    return pcd_map


def pcd_to_voxel_grid(pcd, voxel_size=0.2):
    f_voxel = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)
    return f_voxel

