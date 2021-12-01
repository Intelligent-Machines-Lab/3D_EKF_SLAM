import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import cv2
import random
import glob
import copy 
import pandas as pd
import os
from aux.aux import *
from aux.plane import Plane


class Odometer:

    def __init__(self, nomepasta):
        self.nomepasta = nomepasta


    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp])
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        o3d.visualization.draw_geometries([source_temp, target_temp])

    def preprocess_point_cloud(self, pcd, voxel_size):
        #print(":: Downsample with a voxel size %.3f." % voxel_size)
        pcd_down = pcd.voxel_down_sample(voxel_size)

        radius_normal = voxel_size * 2
        #print(":: Estimate normal with search radius %.3f." % radius_normal)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = voxel_size * 5
        #print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
        pcd_fpfh = o3d.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh


    def prepare_dataset(self, source, target, voxel_size=0.05):
        source_down, source_fpfh = self.preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = self.preprocess_point_cloud(target, voxel_size)
        return source, target, source_down, target_down, source_fpfh, target_fpfh

    def execute_global_registration(self, source_down, target_down, source_fpfh,
                                    target_fpfh, voxel_size):
        distance_threshold = voxel_size * 1.5
        #print(":: RANSAC registration on downsampled point clouds.")
        #print("   Since the downsampling voxel size is %.3f," % voxel_size)
        #print("   we use a liberal distance threshold %.3f." % distance_threshold)
        result = o3d.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
            o3d.registration.TransformationEstimationPointToPoint(True), 4, [
                o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.registration.RANSACConvergenceCriteria(4000000, 1000))
        return result

    # def removeMainPlanes(self, pc1):
    #     outlier_cloud = copy.deepcopy(pc1)
    #     npontos = np.asarray(outlier_cloud.points).shape[0]
    #     filter_radius = True
    #     while(True):
    #         # Ransac planar
    #         points = np.asarray(outlier_cloud.points)
            
    #         p = Plane()
    #         best_eq, best_inliers, valid = p.findPlane(points, thresh=0.06, minPoints=100, maxIteration=400)
    #         qtn_inliers = best_inliers.shape[0]
    #         if(qtn_inliers < int(0.2*npontos)):
    #             break
    #         out = copy.deepcopy(outlier_cloud).select_by_index(best_inliers, invert=True)
    #         points = np.asarray(out.points)
    #         if(points.shape[0] < int(0.004*npontos)):
    #             break
    #         outlier_cloud = outlier_cloud.select_by_index(best_inliers, invert=True)
    #         if(filter_radius):
    #             cl, ind = outlier_cloud.remove_radius_outlier(nb_points=int(0.002*npontos), radius=0.12)
    #             #display_inlier_outlier(outlier_cloud, ind)
    #             outlier_cloud = outlier_cloud.select_by_index(ind)
    #     return outlier_cloud

    def get_odometry(self, i, method="auto", icp_refine=True, max_tentativas_ransac = 10, frame='camera'):
        df = pd.read_csv(self.nomepasta+"/data.txt", index_col=False)
        df.columns =[col.strip() for col in df.columns]

        icp_method = "normal"
        deu_boa = False
        n_auto = 0

        while not deu_boa:
            color_raw = o3d.io.read_image(self.nomepasta+"/"+str(i)+"_rgb.png")
            depth_raw = o3d.io.read_image(self.nomepasta+"/"+str(i)+"_depth.png")
            pc1 = open_pointCloud_from_rgb_and_depth(color_raw, depth_raw, meters_trunc=10, showImages = False)

            color_raw = o3d.io.read_image(self.nomepasta+"/"+str(i+1)+"_rgb.png")
            depth_raw = o3d.io.read_image(self.nomepasta+"/"+str(i+1)+"_depth.png")
            pc2 = open_pointCloud_from_rgb_and_depth(color_raw, depth_raw, meters_trunc=10, showImages = False)

            # pc1 = self.removeMainPlanes(pc1)
            # pc2 = self.removeMainPlanes(pc2)

            voxel_size = 0.05 # means 5cm for this dataset
            source, target, source_down, target_down, source_fpfh, target_fpfh = self.prepare_dataset(pc1, pc2, voxel_size)

            T = np.identity(4)
            t_dur = df['t_command'].values[i+1]
            c_linear = [df['c_linear_x'].values[i+1],
                        df['c_linear_y'].values[i+1], df['c_linear_z'].values[i+1]]
            c_angular = [df['c_angular_x'].values[i+1], df['c_angular_y'].values[i+1], df['c_angular_z'].values[i+1]]
            T[:3,:3] = get_rotation_matrix_bti([c_angular[1]*t_dur,c_angular[2]*t_dur, c_angular[0]*t_dur]).T
            T[0,3] = c_linear[2]*t_dur
            T[1,3] = c_linear[1]*t_dur
            T[2,3] = c_linear[0]*t_dur # Odometria X significa aumento do Z
            odom_transf = T

            #print("Odometry: ", odom_transf)
            x, y, z, yaw, pitch, roll = get_xyz_yawpitchroll_from_transf_matrix(odom_transf)
            print("ODOM  ------------------")
            print("yaw: ", yaw)
            print("pitch: ", pitch)
            print("roll: ", roll)
            print("X: ", x)
            print("Y: ", y)
            print("Z: ", z)


            if(method == "auto" or method == "ransac" or icp_refine):
                if(method == "auto" or method == "ransac"):
                    result_ransac = self.execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
                    glob_reg = result_ransac.transformation
                    x, y, z, yaw, pitch, roll = get_xyz_yawpitchroll_from_transf_matrix(glob_reg)
                    print("RANSAC  ------------------")
                    print("yaw: ", yaw)
                    print("pitch: ", pitch)
                    print("roll: ", roll)
                    print("X: ", x)
                    print("Y: ", y)
                    print("Z: ", z)
            
                if(icp_refine):
                    voxel_radius = [0.1, 0.05, 0.04, 0.02, 0.01]
                    max_iter = [50000, 10000, 5000, 300, 140]
                    #current_transformation = np.identity(4)
                    if(method == "odom"):
                        current_transformation = odom_transf
                    else:
                        current_transformation = glob_reg
                    
                    #self.draw_registration_result(source, target, current_transformation)
                    #print(current_transformation)

                    for scale in range(5):
                        iter = max_iter[scale]
                        radius = voxel_radius[scale]
                        #print([iter, radius, scale])


                        if(icp_method == "colored"):
                            #print("3-3. Applying colored point cloud registration")
                            result_icp = o3d.registration.registration_icp(
                                source_down, target_down, radius, current_transformation,
                                o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                                        relative_rmse=1e-6,
                                                                        max_iteration=iter))
                            current_transformation = result_icp.transformation
                            #print(result_icp)
                        else:
                            #print("3-1. Downsample with a voxel size %.2f" % radius)
                            source_down = source.voxel_down_sample(radius)
                            target_down = target.voxel_down_sample(radius)

                            #print("3-2. Estimate normal.")
                            source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
                            target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
                            #print("3-3. Applying normal point cloud registration")
                            result_icp = o3d.registration.registration_icp(
                                source_down, target_down, radius, current_transformation,
                                o3d.registration.TransformationEstimationPointToPlane())
                            current_transformation = result_icp.transformation

                    x, y, z, yaw, pitch, roll = get_xyz_yawpitchroll_from_transf_matrix(current_transformation)
                    print("ICP ------------------")
                    print("yaw: ", yaw)
                    print("pitch: ", pitch)
                    print("roll: ", roll)
                    print("X: ", x)
                    print("Y: ", y)
                    print("Z: ", z)
                    #self.draw_registration_result(source, target, current_transformation)
            if not icp_refine:
                if(method == "odom"):
                    current_transformation = odom_transf
                else:
                    current_transformation = glob_reg
                #self.draw_registration_result(source, target, current_transformation)
            if(method == "odom" or method == "ransac"):
                deu_boa = True
            else:
                # auto
                # verify consistency
                odomx, odomy, odomz, odomyaw, odompitch, odomroll = get_xyz_yawpitchroll_from_transf_matrix(odom_transf)
                ranx, rany, ranz, ranyaw, ranpitch, ranroll = get_xyz_yawpitchroll_from_transf_matrix(glob_reg)

                diffpos = np.abs(np.asarray([odomx-ranx, odomy-rany, odomz-ranz]))
                diffangle = np.abs(np.asarray([get_diff_angles(odomyaw, ranyaw), get_diff_angles(odompitch, ranpitch), get_diff_angles(odomroll, ranroll)]))

                max_desl_error = np.amax(diffpos)
                max_angle_error = np.amax(diffangle)

                if(max_desl_error > 0.2 or max_angle_error > 0.2):
                    print("deu ruim, erros pos: ", diffpos, " erro anglo: ", diffangle)
                    if(n_auto < max_tentativas_ransac):
                        deu_boa = False
                        n_auto = n_auto+1
                    else: # muda pra odometria
                        print("Usando odometria e foda-se")
                        deu_boa = False
                        method = "odom"
                else:
                    #print("deu boa, erros pos: ", diffpos, " erro anglo: ", diffangle)
                    deu_boa = True
                    current_transformation = glob_reg


        self.draw_registration_result(source, target, current_transformation)
        return current_transformation












        