import open3d as o3d
import numpy as np
import random
import copy 
import json
import pickle
import scipy.io

def get_mem_voxel_grid(voxel_grid, method="open3d"):
    #memq = voxel_grid.get_mem_size()
    if method == "open3d":
        qtd_voxels = len(voxel_grid.get_voxels())
        qtd_bucket = int(qtd_voxels/0.7)
        info =	{"qtd_voxels": qtd_voxels,
                "qtd_buckets": qtd_bucket,
                "grid_size": 144,
                "bucket_size": qtd_bucket*16,
                "voxel_size": qtd_voxels*23,
                "mem_size":qtd_voxels*23 + qtd_bucket*16 +144,
                "mem_size_colorless":qtd_voxels*20 + qtd_bucket*16 +144
                }
    else:
        grid_size = np.asarray(voxel_grid.get_max_bound()-voxel_grid.get_min_bound())
        qtd_cells_space = np.ceil(grid_size/voxel_grid.voxel_size)
        qtd_cells = qtd_cells_space[0]*qtd_cells_space[1]*qtd_cells_space[2]
        info =	{"qtd_voxels": qtd_cells,
                "mem_size":qtd_cells*16 + 24,
                "mem_size_colorless":qtd_cells*13 + 24,
                }
    return info

def get_mem_feature(feature):
    if feature == "plane":
        info =	{"mem_size": 35,
                 "mem_size_colorless": 32
                }
    elif feature == "cylinder":
        info =	{"mem_size": 23,
                 "mem_size_colorless": 20
                }
    elif feature == "cuboid":
        info =	{"mem_size": 27,
                 "mem_size_colorless": 24
                }
    elif feature == "sphere":
        info =	{"mem_size": 19,
                 "mem_size_colorless": 16
                }
    return info


def get_mem_pcd(pcd):
    info =	{"qtd_points": np.asarray(pcd.points).shape[0],
            "mem_size": np.asarray(pcd.points).shape[0]*15,
            "mem_size_colorless": np.asarray(pcd.points).shape[0]*12
            }
    return info



def get_mem_octree(octree, method="open3d"):
    global qtd_root
    global qtd_internal_node
    global qtd_leaf_node
    qtd_root = 1
    qtd_internal_node = 0
    qtd_leaf_node = 0
    def f_traverse(node, node_info):
        global qtd_root
        global qtd_internal_node
        global qtd_leaf_node
        early_stop = False
        if isinstance(node, o3d.geometry.OctreeInternalNode):
            qtd_internal_node = qtd_internal_node+1
        elif isinstance(node, o3d.geometry.OctreeLeafNode):
            qtd_leaf_node = qtd_leaf_node+1
        else:
            raise NotImplementedError('Node type not recognized!')

        return early_stop

    octree.traverse(f_traverse)
    info =	{"qtd_root": qtd_root,
            "qtd_internal_node": qtd_internal_node,
            "qtd_leaf_node": qtd_leaf_node
            }
    if method == "open3d" :
        info["mem_size"] = qtd_root*84 + qtd_internal_node*64 + qtd_leaf_node*3
        info["mem_size_colorless"] = qtd_root*84 + qtd_internal_node*64 + qtd_leaf_node*1
    else:
        info["mem_size"] = qtd_root*80 + qtd_internal_node*88 + qtd_leaf_node*27
        info["mem_size_colorless"] = qtd_root*80 + qtd_internal_node*88 + qtd_leaf_node*1
    return info

class Mem_log_cointainer:
    def __init__(self):
        self.dict_log = {}
        self.step = 0

    def define_log(self, name):
        self.dict_log[name] = {}

    def next(self):
        self.step = self.step +1 

    def log(self, name, data_to_store):
        self.dict_log[name][self.step] = data_to_store

    def save_as_json(self):
        with open('map_outputs/memory_log.json', 'w') as fp:
            json.dump(self.dict_log, fp)

    def save_as_pickle(self):
        with open('map_outputs/memory_log.p', 'wb') as fp:
            pickle.dump(self.dict_log, fp, protocol=pickle.HIGHEST_PROTOCOL)

    def save_as_matlab(self):
        fname = "map_outputs/memory_log_matlab" # arbitrary filename
        scipy.io.savemat(fname, self.dict_log)

