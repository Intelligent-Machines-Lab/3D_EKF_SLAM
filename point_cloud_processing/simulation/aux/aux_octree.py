import open3d as o3d
import numpy as np

box_structure = []
pcd_structure = []
n_points_per_node = 15

def f_traverse_octree_to_pcd(node, node_info):
    early_stop = False
    global pcd_structure
    global box_structure
    global n_points_per_node

    if isinstance(node, o3d.geometry.OctreeLeafNode):
        if isinstance(node, o3d.geometry.OctreePointColorLeafNode):
            mesh_box = o3d.geometry.TriangleMesh.create_box(width=node_info.size,
                                                            height=node_info.size,
                                                            depth=node_info.size)
            mesh_box.paint_uniform_color(node.color)
            mesh_box.translate(node_info.origin)
            pcd_node = mesh_box.sample_points_uniformly(number_of_points=n_points_per_node)
            box_structure.append(mesh_box)
            pcd_structure.append(pcd_node)
    # early stopping: if True, traversal of children of the current node will be skipped
    return early_stop

def octree_to_pcd(octree, n_points=50):
    global pcd_structure
    global box_structure
    box_structure = []
    pcd_structure = []
    global n_points_per_node
    n_points_per_node = n_points
    octree.traverse(f_traverse_octree_to_pcd)

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


def pcd_to_octree(pcd, voxel_size=0.2):
    f_octree = o3d.geometry.Octree(max_depth=0)
    f_octree.convert_from_point_cloud(pcd, size_expand=0)

    globalsize = f_octree.size
    perfect_depth = np.log2(globalsize/voxel_size)
    rounded_depth = int(np.ceil(perfect_depth))
    final_estimated_size = (voxel_size*2**rounded_depth)
    factor = final_estimated_size/globalsize -1
    octree = o3d.geometry.Octree(max_depth=rounded_depth)
    octree.convert_from_point_cloud(pcd, size_expand=factor)
    return octree

def append_pcd_to_octree(pcd, octree):
    pcd_map = o3d.geometry.PointCloud()
    pcd_map.points = o3d.utility.Vector3dVector([])

    pcd_original = octree_to_pcd(octree)
    pt_antigo = np.asarray(pcd_original.points)
    pt_novo = np.asarray(pcd.points)
    cor_antiga = np.asarray(pcd_original.colors)
    cor_nova = np.asarray(pcd.colors)
    pcd_map.points = o3d.utility.Vector3dVector(np.append(pt_antigo, pt_novo, axis=0))
    pcd_map.colors = o3d.utility.Vector3dVector(np.append(cor_antiga, cor_nova, axis=0))
    o3d.visualization.draw_geometries([pcd_map])

    octree = pcd_to_octree(pcd_map)

    return octree