import open3d as o3d
import numpy as np
import settings

import random
import copy 
import time
import matplotlib.pyplot as plt
from aux.LocalScene import LocalScene
from aux.cylinder import Cylinder
from aux.plane import Plane
from aux.generic_feature import Generic_feature
from aux.aux import *
from aux.aux_memory import *
from aux.aux_ekf import *
from aux.cuboid import Cuboid
import threading
from tkinter import *
from tkinter import ttk
from tkinter import Toplevel
import pickle
from mpl_toolkits.mplot3d import Axes3D
from timeit import default_timer as timer


class GlobalScene:

    def __init__(self):
        self.list_scenes = []
        self.scenes_rotation = [] # List of euclidian angles
        self.scenes_translation = [] # list of translations
        

        self.pcd_total = []
        self.groundNormal = []

        self.features_objects = []
        self.fet_geo = []

        self.ground_normal = []
        self.ground_equation = [0, 0, -1, 1.2]
        self.lc_atual = []
        self.propwindow = []

        self.updated_global = threading.Event()
        self.updated_global.clear()
        self.iatual = 0

        self.ekf = ekf()

        self.use_planes = True
        self.use_cylinders = True

        self.store_point_bucket = settings.get_setting('save_point_cloud')
        self.store_octree_model = settings.get_setting('save_octree')
        self.store_voxel_grid_model = settings.get_setting('save_voxel_grid')


        self.memLogger = Mem_log_cointainer()

        # log definitions
        self.memLogger.define_log("pcd")
        self.memLogger.define_log("pcd_colorless")

        self.memLogger.define_log("octree")
        self.memLogger.define_log("octree_colorless")
        self.memLogger.define_log("octree_open3d")
        self.memLogger.define_log("octree_open3d_colorless")

        self.memLogger.define_log("voxel_grid")
        self.memLogger.define_log("voxel_grid_colorless")
        self.memLogger.define_log("voxel_grid_open3d")
        self.memLogger.define_log("voxel_grid_open3d_colorless")

        self.memLogger.define_log("only_octree")
        self.memLogger.define_log("only_octree_colorless")
        self.memLogger.define_log("only_octree_open3d")
        self.memLogger.define_log("only_octree_open3d_colorless")

        self.memLogger.define_log("only_voxel_grid")
        self.memLogger.define_log("only_voxel_grid_colorless")
        self.memLogger.define_log("only_voxel_grid_open3d")
        self.memLogger.define_log("only_voxel_grid_open3d_colorless")

        self.memLogger.define_log("low_level_world")
        self.memLogger.define_log("low_level_world_colorless")
        self.memLogger.define_log("high_level_world")
        self.memLogger.define_log("high_level_world_colorless")

        self.memLogger.define_log("voxel_grid_truncate")
        self.memLogger.define_log("octree_truncate")

        self.memLogger.define_log("only_voxel_grid_truncate")
        self.memLogger.define_log("only_octree_truncate")

        self.memLogger.define_log("time_log")
        self.t__ = {}

        self.paint_features = True
        self.save_maps = True

    def get_feature_from_id(self, id):
        for i_global in self.features_objects:
            if id == i_global.id:
                return i_global

    def custom_draw_geometry(self):
        # The following code achieves the same effect as:
        # o3d.visualization.draw_geometries([pcd])
        rotatex = 500
        rotatey = 550
        rotatey2 = 100
        showimages = True
        save_image = False
        vis_original = o3d.visualization.Visualizer()

        vis_original.create_window(window_name='Original', width=960, height=540, left=0, top=0)
        vis_original.add_geometry(self.lc_atual.pointCloud)
        vis_original.get_view_control().rotate(0, rotatey)
        vis_original.get_view_control().rotate(rotatex, 0)
        vis_original.get_view_control().rotate(0, rotatey2)
        vis_original.poll_events()
        vis_original.update_renderer()
        if(save_image):
            vis_original.capture_screen_image("animations/original-"+str(self.iatual)+".png")

        feat = self.lc_atual.getMainPlanes()
        feat.extend(self.lc_atual.getCylinders(showPointCloud=False))
        feat.extend(self.lc_atual.getSecundaryPlanes())
        vis_feature = o3d.visualization.Visualizer()
        vis_feature.create_window(window_name='Feature', width=960, height=540, left=960, top=0)
        for x in range(len(feat)):
            vis_feature.add_geometry(feat[x])
        vis_feature.get_view_control().rotate(0, rotatey)
        vis_feature.get_view_control().rotate(rotatex, 0)
        vis_feature.get_view_control().rotate(0, rotatey2)
        vis_feature.poll_events()
        vis_feature.update_renderer()
        if(save_image):
            vis_feature.capture_screen_image("animations/feature-"+str(self.iatual)+".png")

        if self.store_point_bucket:
            global_bucket = []
            vis_feature_point = o3d.visualization.Visualizer()
            vis_feature_point.create_window(window_name='Feature point', width=960, height=540, left=0, top=540)
            for x in range(len(self.features_objects)):
                pcd_feat_copy = copy.deepcopy(self.features_objects[x])
                if self.paint_features:
                    pcd_feat_copy.feat.bucket.paint_uniform_color(pcd_feat_copy.feat.color)
                global_bucket.append(pcd_feat_copy.feat.bucket)
            for x in range(len(global_bucket)):
                vis_feature_point.add_geometry(global_bucket[x])
            vis_feature_point.get_view_control().rotate(0, rotatey)
            vis_feature_point.get_view_control().rotate(rotatex, 0)
            vis_feature_point.get_view_control().rotate(0, rotatey2*2)
            vis_feature_point.poll_events()
            vis_feature_point.update_renderer()
            if(save_image):
                vis_feature_point.capture_screen_image("animations/point-"+str(self.iatual)+".png")


        vis_feature_global = o3d.visualization.Visualizer()
        vis_feature_global.create_window(window_name='Feature global', width=960, height=540, left=960, top=540)
        for x in range(len(self.fet_geo)):
            vis_feature_global.add_geometry(self.fet_geo[x])
        vis_feature_global.get_view_control().rotate(0, rotatey)
        vis_feature_global.get_view_control().rotate(rotatex, 0)
        vis_feature_global.get_view_control().rotate(0, rotatey2*2)
        vis_feature_global.poll_events()
        vis_feature_global.update_renderer()
        if(save_image):
            vis_feature_global.capture_screen_image("animations/global-"+str(self.iatual)+".png")
        if(showimages):
            while True:
                vis_original.update_geometry(self.lc_atual.pointCloud)
                if not vis_original.poll_events():
                    break
                vis_original.update_renderer()
                #vis_original.capture_screen_image("animations/original-"+str(self.iatual)+".png")

                feat = self.lc_atual.getMainPlanes()
                feat.extend(self.lc_atual.getCylinders(showPointCloud=False))
                feat.extend(self.lc_atual.getSecundaryPlanes())
                for x in range(len(feat)):
                    vis_feature.update_geometry(feat[x])
                
                if not vis_feature.poll_events():
                    break
                vis_feature.update_renderer()

                if self.store_point_bucket:
                    global_bucket = []
                    for x in range(len(self.features_objects)):
                        pcd_feat_copy = copy.deepcopy(self.features_objects[x])
                        if self.paint_features:
                            pcd_feat_copy.feat.bucket.paint_uniform_color(pcd_feat_copy.feat.color)
                        global_bucket.append(pcd_feat_copy.feat.bucket)


                    for x in range(len(global_bucket)):
                        vis_feature_point.update_geometry(global_bucket[x])
                    if not vis_feature_point.poll_events():
                        break
                    vis_feature_point.update_renderer()

                #vis_feature.capture_screen_image("animations/feature-"+str(self.iatual)+".png")
                for x in range(len(self.fet_geo)):
                    vis_feature_global.update_geometry(self.fet_geo[x])
                if not vis_feature_global.poll_events():
                    break
                vis_feature_global.update_renderer()
                #vis_feature_global.capture_screen_image("animations/global-"+str(self.iatual)+".png")



        vis_feature_global.destroy_window()
        vis_original.destroy_window()
        vis_feature.destroy_window()
        if self.store_point_bucket:
            vis_feature_point.destroy_window()

    # def draw_geometries_pick_points(self, geometries):
    #     vis = o3d.visualization.VisualizerWithEditing()
    #     vis.create_window()
    #     for geometry in geometries:
    #         vis.add_geometry(geometry)
    #     vis.run()
    #     vis.destroy_window()


    def add_pcd(self, pcd, commands_odom_linear, commands_odom_angular, duration, i, vel_pos_real=[0,0,0], vel_orienta_real=[0, 0, 0,]):
        self.t__ = {}
        t__start = timer()
        self.iatual = i
        last_loc = np.asarray([0, 0, 0])
        last_angulo = np.asarray([0, 0, 0])
        x_m_last, P_m_last = init_x_P()
        if not len(self.scenes_translation) == 0:
            # last_loc = self.scenes_translation[-1]
            last_angulo = self.scenes_rotation[-1]
            # x_m_last = self.ekf.x_m
            # P_m_last = self.P_m


        # print("Odometria linear: "+str(commands_odom_linear))
        # print("Odometria angular: "+str(commands_odom_angular))

        # in the body frame
        # transform commands into the right frame
        vel_angular_body = np.asarray(commands_odom_angular)
        vel_linear_body = np.asarray(commands_odom_linear)
        vel_angular_body[2] = - vel_angular_body[2]
        vel_angular_body[1] = - vel_angular_body[1]
        vel_linear_body[1] = - vel_linear_body[1]
        vel_linear_body[2] = - vel_linear_body[2]


        vel_pos_real = np.asarray(vel_pos_real)
        vel_orienta_real = np.asarray(vel_orienta_real)
        vel_orienta_real[2] = - vel_orienta_real[2]
        vel_orienta_real[1] = - vel_orienta_real[1]
        vel_pos_real[1] = - vel_pos_real[1]
        vel_pos_real[2] = - vel_pos_real[2]

        u_real = duration*np.asarray([[vel_pos_real[0]], [vel_orienta_real[2]]])

        # print("vel_angular_body: "+str(vel_angular_body))
        # print("vel_linear_body: "+str(vel_linear_body))
        # Change to inertial frame
        # vel_angular_inertial = np.dot(get_rotation_angvel_matrix_bti(last_angulo),vel_angular_body.T)
        # vel_linear_inertial = np.dot(get_rotation_matrix_bti(last_angulo),vel_linear_body.T)


        # KALMAN FILTER --------------------------------------------
        
        if not (self.ekf.x_m[0,0]==0 and self.ekf.x_m[1,0]==0 and self.ekf.x_m[2,0]==0):
            mv = get_V()
            mu, sigma = 0, np.sqrt(mv[0,0]) # mean and standard deviation
            noise_x = np.random.normal(mu, sigma, 1)[0]

            mu, sigma = 0, np.sqrt(mv[1,1]) # mean and standard deviation
            noise_theta = np.random.normal(mu, sigma, 1)[0]
        else:
            noise_x = 0
            noise_theta = 0

        if not settings.get_setting('add_noise'):
            noise_x = 0
            noise_theta = 0
        print('noise x: \n', noise_x)
        print('noise_theta: \n', noise_theta)
        u = duration*np.asarray([[vel_linear_body.T[0] + noise_x],
                                 [vel_angular_body.T[2] + noise_theta]])

        print(u)

        self.ekf.propagate(u)
        self.ekf.update_real_odom_states(u_real, u)
        # if self.ekf.num_total_features['feature'] > 3:
        #     neweq = self.ekf.upload_plane(np.asarray([[1], [0], [0]]), 0)
        #     print('neweq: ', neweq)
            # self.ekf.upload_plane(np.asarray([[1], [0], [0]]), 1)
            # self.ekf.upload_plane(np.asarray([[1], [0], [0]]), 2)
        

        # ----------------------------------------------------------



        # print("vel_angular_inertial: "+ str(vel_angular_inertial))
        # print("vel_linear_inertial: "+ str(vel_linear_inertial))

        # Propagação 
        atual_loc = [self.ekf.x_m[0,0], self.ekf.x_m[1,0], 0]
        atual_angulo = [0, 0, self.ekf.x_m[2,0]]
        
        #atual_loc = last_loc + vel_linear_inertial*duration
        #atual_angulo = last_angulo + vel_angular_inertial*duration

        print("atual_loc: "+ str(atual_loc))
        print("atual_angulo: "+ str(atual_angulo))
        #print("last_loc: "+ str(last_loc))
        #print("last_angulo: "+ str(last_angulo))
        #print("vel_linear_inertial: "+ str(vel_linear_inertial))
        #print("vel_angular_inertial: "+ str(vel_angular_inertial))
        #print("duration: "+ str(duration))
        t__end = timer()
        self.t__['ekf_propagate'] = t__end - t__start

        t__start = timer()
        from_camera_to_inertial(pcd)

        # print("AAAAAAAAAAAAAAAAAAAh")
        # print(get_rotation_matrix_bti(atual_angulo).T)
        ls = LocalScene(pcd)
        self.lc_atual = ls
        
        #Ransac total
        ls.findMainPlanes()
        ls.defineGroundNormal(ground_eq_reserva=self.ground_equation)
        #o3d.visualization.draw_geometries(ls.getMainPlanes())
        #ls.showNotPlanes()
        ls.clusterizeObjects()
        #ls.showObjects()
        ls.fitCylinder()
        ls.findSecundaryPlanes()
        self.ground_normal = ls.groundNormal
        self.ground_equation = ls.groundEquation
        #print("GND EQUATION: ", self.ground_equation)
        t__end = timer()
        self.t__['feature_extraction'] = t__end - t__start

        scene_features=[]

        planes_list = []
        if self.use_planes:
            planes_list.extend(ls.mainPlanes.copy())
            planes_list.extend(ls.secundaryPlanes.copy())

        mundinho = []
        #print('mundinho: ', mundinho)
        plane_list2 = copy.deepcopy(planes_list)
        for x in range(len(plane_list2)):
            plane_list2[x].move(self.ekf)
            mundinho.extend(plane_list2[x].get_geometry())
        #o3d.visualization.draw_geometries(mundinho)    

        t__start_feat_augmentation = timer()
        t__full_bucket_augmentation=0

        t__full_mahala = 0
        t__full_update = 0
        oldstate = copy.deepcopy(self.ekf)
        for x in range(len(planes_list)):
            mundinho = []
            mundinho.extend(self.fet_geo)


            t__start = timer()
            id_list = self.ekf.calculate_mahalanobis(planes_list[x])
            t__end = timer()
            t__full_mahala = t__full_mahala + (t__end-t__start)

            inliers_raw = planes_list[x].inliers
            #id = -1
            z_medido = np.asarray([[planes_list[x].equation[0], planes_list[x].equation[1], planes_list[x].equation[2], planes_list[x].equation[3]]]).T
            normal_feature = np.asarray([planes_list[x].equation[0], planes_list[x].equation[1], planes_list[x].equation[2]])
            bigger_axis = np.argmax(np.abs(normal_feature))
            if settings.get_setting('bypass_ground_plane'):
                if bigger_axis == 2:
                    continue

            planes_list[x].move(self.ekf)
            t__start_feat_augmentation = t__start_feat_augmentation - planes_list[x].t__bucket_debug
            gfeature = Generic_feature(planes_list[x], ground_equation=self.ground_equation)
            id = id_list[0]
            if settings.get_setting('no_data_association'):
                id=-1
            if(not id == -1):
                for id in id_list:
                    print("ANALIZANDO ID ", id)
                    older_feature = self.get_feature_from_id(id)

                    is_correspondence = older_feature.correspond(gfeature, self.ekf)
                    t__full_update = t__full_update + older_feature.t__update
                    t__full_bucket_augmentation = t__full_bucket_augmentation + older_feature.t__bucket_augmentation
                    if not is_correspondence:
                        id = -1
                    else:
                        # Encontrou correspondência na lista, não precisa seguir procurando
                        break
                        # measured_plane = copy.deepcopy(gfeature)
                        # measured_plane.feat.color = [1, 0, 0]
                        # older_feature2 = copy.deepcopy(older_feature)
                        # older_feature2.feat.color = [0, 1, 0]
                        # mundinho.extend(measured_plane.feat.get_geometry())
                        # mundinho.extend(older_feature2.feat.get_geometry())
                        #o3d.visualization.draw_geometries(mundinho) 
                    
            if id == -1:
                i = self.ekf.add_plane(z_medido)
                gfeature.id = i
                self.features_objects.append(gfeature)


        if self.use_cylinders:
            for x in range(len(ls.mainCylinders)):
                #i = self.ekf.add_plane(z_medido)
                #gfeature.id = i
                #self.features_objects.append(gfeature)
                t__start = timer()
                cent = np.asarray([[ls.mainCylinders[x].center[0]],[ls.mainCylinders[x].center[1]],[ls.mainCylinders[x].center[2]]])
                id_list = self.ekf.calculate_mahalanobis(ls.mainCylinders[x])
                t__end = timer()
                t__full_mahala = t__full_mahala + (t__end-t__start)

                ls.mainCylinders[x].move(self.ekf)
                t__start_feat_augmentation = t__start_feat_augmentation - ls.mainCylinders[x].t__bucket_debug
                
                gfeature = Generic_feature(ls.mainCylinders[x], ground_equation=self.ground_equation)
                id = id_list[0]
                if settings.get_setting('no_data_association'):
                    id=-1
                if not id == -1:
                    for id in id_list:
                        print("ANALIZANDO ID ", id)
                        older_feature = self.get_feature_from_id(id)
                        is_correspondence = older_feature.correspond(gfeature, self.ekf)
                        t__full_update = t__full_update + older_feature.t__update
                        t__full_bucket_augmentation = t__full_bucket_augmentation + older_feature.t__bucket_augmentation
                        if not is_correspondence:
                            id = -1
                        else:
                            break
                if id == -1:
                    i = self.ekf.add_point(cent)
                    gfeature.id = i

                    self.features_objects.append(gfeature)

        t__end_feat_augmentation = timer()
        self.t__['ekf_data_association'] = t__full_mahala
        self.t__['ekf_update'] = t__full_update
        self.t__['feature_augmentation'] = t__end_feat_augmentation - t__start_feat_augmentation - t__full_update - t__full_mahala - t__full_bucket_augmentation
        self.t__['bucket_augmentation'] = t__full_bucket_augmentation
        self.fet_geo = []

        if self.iatual % 10 == 0:
            if(self.ekf.is_updated):
                for i_global in self.features_objects:
                    self.features_objects.feat.update_new_fqt(self.ekf)

        self.scenes_rotation.append(atual_angulo)
        self.scenes_translation.append(atual_loc)
        #self.fet_geo = []
        for ob in self.features_objects:
            if(ob.running_geo["total"] >= 0):
                #print("EQUACAO FINAL: ", ob.feat.equation)
                self.fet_geo.extend(ob.feat.get_geometry())

        self.fet_geo.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0]).rotate(get_rotation_matrix_bti(atual_angulo), center=(0,0,0)).translate(atual_loc))

        #ls.custom_draw_geometry()
        if(i >4000):#126):
            threading.Thread(target=self.custom_draw_geometry, daemon=True).start()
            #fig = plt.figure()
            #ax = fig.add_subplot(111, projection='3d')
            #posi = np.asarray(self.scenes_translation)
            #print(posi)
            #ax.plot3D(posi[:, 0], posi[:, 1], posi[:, 2], 'gray')
            #plt.show()
            #ax.plot(atual_loc[:, 0],atual_loc[:, 1],atual_loc[:, 2])
        

        self.ekf.save_file()
        
        if(i >=settings.get_setting('stop_photo')):
            self.showPoints(True, self.paint_features)
        else:
            self.showPoints(False, self.paint_features)

        f = open('map_outputs/feat.pckl', 'wb')
        pickle.dump(self.getProprieties(), f)
        f.close()
        #threading.Thread(target=self.showGUI, args=(self.getProprieties(),), daemon=True)
        #self.custom_draw_geometry()

        
        
    def getGroundPlaneId(self):
        modelGround = np.asarray([[0], [0], [1.2]])
        for id in range(self.ekf.num_total_features['feature']):
            if(self.ekf.type_feature_list[id] == self.ekf.types_feat['plane']):
                # The first feature that are most similar with ground model is considered ground
                #print("Dist planos ground: ", np.linalg.norm(np.absolute(self.ekf.get_feature_from_id(id)) - np.abs(modelGround)))
                #print("Plano analizado: ", np.absolute(self.ekf.get_feature_from_id(id)))
                if np.linalg.norm(np.absolute(self.ekf.get_feature_from_id(id)) - np.abs(modelGround)) < 0.8:
                    return id



    def getProprieties(self):
        vet_mainPlanes = []
        vet_mainCylinders = []
        vet_mainCuboids = []
        for x in range(len(self.features_objects)):
            if isinstance(self.features_objects[x].feat,Plane):
                vet_mainPlanes.append(self.features_objects[x].getProprieties())
            elif isinstance(self.features_objects[x].feat,Cylinder):
                vet_mainCylinders.append(self.features_objects[x].getProprieties())
            else:
                vet_mainCuboids.append(self.features_objects[x].getProprieties())

        
        vet_secondaryCylinders = []
        return {"groundNormal": self.groundNormal, "planes": vet_mainPlanes, "cylinders": vet_mainCylinders, "cuboids": vet_mainCuboids}
        #pcd_moved = pcd_moved.voxel_down_sample(voxel_size=0.1)
        
        # self.pcd_total.append(mesh_frame)

        # o3d.visualization.draw_geometries(self.pcd_total)



    

    def showPoints(self, show=True, paint_features=False):
        
        if self.store_voxel_grid_model:
            global_bucket = []
            for x in range(len(self.features_objects)):
                pcd_feat_copy = copy.deepcopy(self.features_objects[x])
                #if paint_features:
                #    pcd_feat_copy.feat.bucket_voxel_grid.paint_uniform_color(pcd_feat_copy.feat.color)
                global_bucket.append(pcd_feat_copy.feat.bucket_voxel_grid)
            if show:
                o3d.visualization.draw_geometries(global_bucket)

        if self.store_octree_model:
            global_bucket = []
            for x in range(len(self.features_objects)):
                pcd_feat_copy = copy.deepcopy(self.features_objects[x])
                #if paint_features:
                #    pcd_feat_copy.feat.bucket_octree.paint_uniform_color(pcd_feat_copy.feat.color)
                global_bucket.append(pcd_feat_copy.feat.bucket_octree)
            if show:
                o3d.visualization.draw_geometries(global_bucket)


        if self.store_point_bucket:

            global_bucket = []
            for x in range(len(self.features_objects)):
                pcd_feat_copy = copy.deepcopy(self.features_objects[x])
                if paint_features:
                    pcd_feat_copy.feat.bucket.paint_uniform_color(pcd_feat_copy.feat.color)
                global_bucket.append(pcd_feat_copy.feat.bucket)
                global_bucket.append(pcd_feat_copy.feat.get_geometry()[0])
            if show:
                o3d.visualization.draw_geometries(global_bucket)

            global_bucket = []
            for x in range(len(self.features_objects)):
                pcd_feat_copy = copy.deepcopy(self.features_objects[x])
                if paint_features:
                    pcd_feat_copy.feat.bucket_odom.paint_uniform_color(pcd_feat_copy.feat.color)
                global_bucket.append(pcd_feat_copy.feat.bucket_odom)
            if show:
                o3d.visualization.draw_geometries(global_bucket)

            global_bucket = []
            for x in range(len(self.features_objects)):
                pcd_feat_copy = copy.deepcopy(self.features_objects[x])
                if paint_features:
                    pcd_feat_copy.feat.bucket_pos.paint_uniform_color(pcd_feat_copy.feat.color)
                global_bucket.append(pcd_feat_copy.feat.bucket_pos)
            if show:
                o3d.visualization.draw_geometries(global_bucket)




            global_bucket = []
            data_log = {'total':0}
            data_log_colorless = {'total':0}
            t__render = 0
            t__full = 0
            for x in range(len(self.features_objects)):
                pcd_feat_copy = copy.deepcopy(self.features_objects[x])
                if paint_features:
                    pcd_feat_copy.feat.bucket.paint_uniform_color(pcd_feat_copy.feat.color)
                t__start = timer()
                global_bucket.append(pcd_feat_copy.feat.bucket)
                
                t__end = timer()
                t__full = t__full + t__end - t__start
                if self.save_maps:
                    o3d.io.write_point_cloud("map_outputs/feature_wise/feature_"+str(x)+"pcd_map.pcd", pcd_feat_copy.feat.bucket)

                mem_usage = get_mem_pcd(self.features_objects[x].feat.bucket)['mem_size']
                data_log[self.features_objects[x].id] = mem_usage
                data_log['total'] = data_log['total'] + mem_usage

                mem_usage_colorless = get_mem_pcd(self.features_objects[x].feat.bucket)['mem_size_colorless']
                data_log_colorless[self.features_objects[x].id] = mem_usage_colorless
                data_log_colorless['total'] = data_log_colorless['total'] + mem_usage_colorless

            if show:
                t__start = timer()
                o3d.visualization.draw_geometries(global_bucket)
                t__end = timer()
                self.t__['pcd_render'] = t__end - t__start

            self.memLogger.log("pcd", data_log)
            self.memLogger.log("pcd_colorless", data_log_colorless)
            self.t__['pcd'] = t__full



            t__full_octree = 0
            t__full_voxel_grid = 0
            t__render = 0

            # fixed_voxel_cize
            global_bucket_octree = []
            global_bucket_voxel_grid = []

            data_log_voxel_grid = {'total':0}
            data_log_voxel_grid_colorless = {'total':0}
            data_log_voxel_grid_open3d = {'total':0}
            data_log_voxel_grid_open3d_colorless = {'total':0}

            data_log_octree = {'total':0}
            data_log_octree_colorless = {'total':0}
            data_log_octree_open3d = {'total':0}
            data_log_octree_open3d_colorless = {'total':0}

            for x in range(len(self.features_objects)):
                pcd_feat_copy = copy.deepcopy(self.features_objects[x])
                if paint_features:
                    pcd_feat_copy.feat.bucket.paint_uniform_color(pcd_feat_copy.feat.color)

                f_octree = pcd_feat_copy.feat.get_octree()
                globalsize = f_octree.size
                cell_size = 0.2
                perfect_depth = np.log2(globalsize/cell_size)
                rounded_depth = int(np.ceil(perfect_depth))
                final_estimated_size = (cell_size*2**rounded_depth)
                factor = final_estimated_size/globalsize -1

                t__start = timer()
                f_octree = pcd_feat_copy.feat.get_octree(rounded_depth, factor)
                t__end = timer()
                t__full_octree = t__full_octree + t__end - t__start

                globalsize = f_octree.size
                mini_voxel_size = f_octree.size/(2**(f_octree.max_depth))

                t__start = timer()
                f_voxel = pcd_feat_copy.feat.getVoxelStructure(mini_voxel_size)
                t__end = timer()
                t__full_voxel_grid = t__full_voxel_grid + t__end - t__start

                global_bucket_octree.append(f_octree)
                global_bucket_voxel_grid.append(f_voxel)

                mem_usage = get_mem_voxel_grid(f_voxel, 'traditional')['mem_size']
                data_log_voxel_grid[pcd_feat_copy.id] = mem_usage
                data_log_voxel_grid['total'] = data_log_voxel_grid['total'] + mem_usage

                mem_usage_colorless = get_mem_voxel_grid(f_voxel, 'traditional')['mem_size_colorless']
                data_log_voxel_grid_colorless[pcd_feat_copy.id] = mem_usage_colorless
                data_log_voxel_grid_colorless['total'] = data_log_voxel_grid_colorless['total'] + mem_usage_colorless

                mem_usage = get_mem_voxel_grid(f_voxel)['mem_size']
                data_log_voxel_grid_open3d[pcd_feat_copy.id] = mem_usage
                data_log_voxel_grid_open3d['total'] = data_log_voxel_grid_open3d['total'] + mem_usage

                mem_usage_colorless = get_mem_voxel_grid(f_voxel)['mem_size_colorless']
                data_log_voxel_grid_open3d_colorless[pcd_feat_copy.id] = mem_usage_colorless
                data_log_voxel_grid_open3d_colorless['total'] = data_log_voxel_grid_open3d_colorless['total'] + mem_usage_colorless

                mem_usage = get_mem_octree(f_octree, 'traditional')['mem_size']
                data_log_octree[pcd_feat_copy.id] = mem_usage
                data_log_octree['total'] = data_log_octree['total'] + mem_usage

                mem_usage_colorless = get_mem_octree(f_octree, 'traditional')['mem_size_colorless']
                data_log_octree_colorless[pcd_feat_copy.id] = mem_usage_colorless
                data_log_octree_colorless['total'] = data_log_octree_colorless['total'] + mem_usage_colorless

                mem_usage = get_mem_octree(f_octree)['mem_size']
                data_log_octree_open3d[pcd_feat_copy.id] = mem_usage
                data_log_octree_open3d['total'] = data_log_octree_open3d['total'] + mem_usage

                mem_usage_colorless = get_mem_octree(f_octree)['mem_size_colorless']
                data_log_octree_open3d_colorless[pcd_feat_copy.id] = mem_usage_colorless
                data_log_octree_open3d_colorless['total'] = data_log_octree_open3d_colorless['total'] + mem_usage_colorless


            if show:
                t__start = timer()
                o3d.visualization.draw_geometries(global_bucket_octree, 'Octree feature-wise fixed size')
                t__end = timer()
                self.t__['octree_open3d_render'] = t__end - t__start

                t__start = timer()
                o3d.visualization.draw_geometries(global_bucket_voxel_grid, 'Voxel-grid feature-wise fixed size')
                t__end = timer()
                self.t__['voxel_grid_open3d_render'] = t__end - t__start


            self.memLogger.log("voxel_grid", data_log_voxel_grid)
            self.memLogger.log("voxel_grid_colorless", data_log_voxel_grid_colorless)
            self.memLogger.log("voxel_grid_open3d", data_log_voxel_grid_open3d)
            self.memLogger.log("voxel_grid_open3d_colorless", data_log_voxel_grid_open3d_colorless)

            self.memLogger.log("octree", data_log_octree)
            self.memLogger.log("octree_colorless", data_log_octree_colorless)
            self.memLogger.log("octree_open3d", data_log_octree_open3d)
            self.memLogger.log("octree_open3d_colorless", data_log_octree_open3d_colorless)
            self.t__['octree_open3d'] = t__full_octree
            self.t__['voxel_grid_open3d'] = t__full_voxel_grid





            t__full_octree = 0
            t__full_voxel_grid = 0
            # Truncate octree
            global_bucket_octree = []
            global_bucket_voxel_grid = []

            data_log_voxel_grid = {'total':0}
            data_log_octree = {'total':0}

            for x in range(len(self.features_objects)):
                pcd_feat_copy = copy.deepcopy(self.features_objects[x])
                if paint_features:
                    pcd_feat_copy.feat.bucket.paint_uniform_color(pcd_feat_copy.feat.color)
                
                t__start = timer()
                f_octree = pcd_feat_copy.feat.get_octree()
                t__end = timer()
                t__full_octree = t__full_octree + t__end - t__start

                mini_voxel_size = f_octree.size/(2**(f_octree.max_depth))

                t__start = timer()
                f_voxel = pcd_feat_copy.feat.getVoxelStructure(mini_voxel_size)
                t__end = timer()
                t__full_voxel_grid = t__full_voxel_grid + t__end - t__start

                global_bucket_octree.append(f_octree)
                global_bucket_voxel_grid.append(f_voxel)

                mem_usage = get_mem_voxel_grid(f_voxel)['mem_size']
                data_log_voxel_grid[pcd_feat_copy.id] = mem_usage
                data_log_voxel_grid['total'] = data_log_voxel_grid['total'] + mem_usage

                mem_usage = get_mem_octree(f_octree)['mem_size']
                data_log_octree[pcd_feat_copy.id] = mem_usage
                data_log_octree['total'] = data_log_octree['total'] + mem_usage
            if show:
                t__start = timer()
                o3d.visualization.draw_geometries(global_bucket_octree, 'Octree feature-wise truncate')
                t__end = timer()
                self.t__['octree_truncate_render'] = t__end - t__start

                t__start = timer()
                o3d.visualization.draw_geometries(global_bucket_voxel_grid, 'Voxel-grid feature-wise truncate')
                t__end = timer()
                self.t__['voxel_grid_truncate_render'] = t__end - t__start


            self.memLogger.log("voxel_grid_truncate", data_log_voxel_grid)
            self.memLogger.log("octree_truncate", data_log_octree)
            self.t__['octree_truncate'] = t__full_octree
            self.t__['voxel_grid_truncate'] = t__full_voxel_grid


            # if self.save_maps:
            #     #Generate pure PCD World
            #     pcd_map = o3d.geometry.PointCloud()
            #     pcd_map.points = o3d.utility.Vector3dVector([])
            #     for x in range(len(self.features_objects)):
            #         pcd_feat_copy = copy.deepcopy(self.features_objects[x])
            #         if paint_features:
            #             pcd_feat_copy.feat.bucket.paint_uniform_color(pcd_feat_copy.feat.color)
            #         #o3d.visualization.draw_geometries([pcd_feat_copy.feat.bucket])
            #         pt_antigo = np.asarray(pcd_map.points)
            #         pt_novo = np.asarray(pcd_feat_copy.feat.bucket.points)
            #         cor_antiga = np.asarray(pcd_map.colors)
            #         cor_nova = np.asarray(pcd_feat_copy.feat.bucket.colors)
            #         #print("apendando", np.append(pt_antigo, pt_novo, axis=0))
            #         pcd_map.points = o3d.utility.Vector3dVector(np.append(pt_antigo, pt_novo, axis=0))
            #         pcd_map.colors = o3d.utility.Vector3dVector(np.append(cor_antiga, cor_nova, axis=0))
                
            
            #     o3d.io.write_point_cloud("map_outputs/pcd_map.pcd", pcd_map)


            if self.save_maps:
                #Generate pure PCD World
                pcd_map = o3d.geometry.PointCloud()
                pcd_map.points = o3d.utility.Vector3dVector([])
                for x in range(len(self.features_objects)):
                    pcd_feat_copy = copy.deepcopy(self.features_objects[x])
                    if paint_features:
                        pcd_feat_copy.feat.bucket.paint_uniform_color(pcd_feat_copy.feat.color)
                    #o3d.visualization.draw_geometries([pcd_feat_copy.feat.bucket])
                    pt_antigo = np.asarray(pcd_map.points)
                    pt_novo = np.asarray(pcd_feat_copy.feat.bucket.points)
                    cor_antiga = np.asarray(pcd_map.colors)
                    cor_nova = np.asarray(pcd_feat_copy.feat.bucket.colors)
                    #print("apendando", np.append(pt_antigo, pt_novo, axis=0))
                    pcd_map.points = o3d.utility.Vector3dVector(np.append(pt_antigo, pt_novo, axis=0))
                    pcd_map.colors = o3d.utility.Vector3dVector(np.append(cor_antiga, cor_nova, axis=0))
                
            
            
                o3d.io.write_point_cloud("map_outputs/pcd_map.pcd", pcd_map)

                #Generate pure PCD World
                pcd_map = o3d.geometry.PointCloud()
                pcd_map.points = o3d.utility.Vector3dVector([])
                for x in range(len(self.features_objects)):
                    pcd_feat_copy = copy.deepcopy(self.features_objects[x])
                    if paint_features:
                        pcd_feat_copy.feat.bucket_odom.paint_uniform_color(pcd_feat_copy.feat.color)
                    #o3d.visualization.draw_geometries([pcd_feat_copy.feat.bucket])
                    pt_antigo = np.asarray(pcd_map.points)
                    pt_novo = np.asarray(pcd_feat_copy.feat.bucket_odom.points)
                    cor_antiga = np.asarray(pcd_map.colors)
                    cor_nova = np.asarray(pcd_feat_copy.feat.bucket_odom.colors)
                    #print("apendando", np.append(pt_antigo, pt_novo, axis=0))
                    pcd_map.points = o3d.utility.Vector3dVector(np.append(pt_antigo, pt_novo, axis=0))
                    pcd_map.colors = o3d.utility.Vector3dVector(np.append(cor_antiga, cor_nova, axis=0))
                
            
                o3d.io.write_point_cloud("map_outputs/pcd_odom.pcd", pcd_map)



            t__full_octree = 0
            t__full_voxel_grid = 0


            #Generate pure octree World
            # fixed_voxel_cize
            global_bucket_octree = []
            global_bucket_voxel_grid = []

            data_log_voxel_grid = {'total':0}
            data_log_voxel_grid_colorless = {'total':0}
            data_log_voxel_grid_open3d = {'total':0}
            data_log_voxel_grid_open3d_colorless = {'total':0}

            data_log_octree = {'total':0}
            data_log_octree_colorless = {'total':0}
            data_log_octree_open3d = {'total':0}
            data_log_octree_open3d_colorless = {'total':0}

            pcd_map = o3d.geometry.PointCloud()
            pcd_map.points = o3d.utility.Vector3dVector([])
            for x in range(len(self.features_objects)):
                pcd_feat_copy = copy.deepcopy(self.features_objects[x])
                if paint_features:
                    pcd_feat_copy.feat.bucket.paint_uniform_color(pcd_feat_copy.feat.color)

                pt_antigo = np.asarray(pcd_map.points)
                pt_novo = np.asarray(pcd_feat_copy.feat.bucket.points)
                cor_antiga = np.asarray(pcd_map.colors)
                cor_nova = np.asarray(pcd_feat_copy.feat.bucket.colors)
                #print("apendando", np.append(pt_antigo, pt_novo, axis=0))
                pcd_map.points = o3d.utility.Vector3dVector(np.append(pt_antigo, pt_novo, axis=0))
                pcd_map.colors = o3d.utility.Vector3dVector(np.append(cor_antiga, cor_nova, axis=0))
            
            f_octree = o3d.geometry.Octree(max_depth=5)
            f_octree.convert_from_point_cloud(pcd_map)
            globalsize = f_octree.size
            cell_size = 0.2
            perfect_depth = np.log2(globalsize/cell_size)
            rounded_depth = int(np.ceil(perfect_depth))
            final_estimated_size = (cell_size*2**rounded_depth)
            factor = final_estimated_size/globalsize -1

            t__start = timer()
            f_octree = o3d.geometry.Octree(max_depth=rounded_depth)
            f_octree.convert_from_point_cloud(pcd_map, size_expand=factor)
            t__end = timer()
            t__full_octree = t__full_octree + t__end - t__start

            globalsize = f_octree.size
            mini_voxel_size = f_octree.size/(2**(f_octree.max_depth))

            t__start = timer()
            f_voxel = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd_map, voxel_size=mini_voxel_size)
            t__end = timer()
            t__full_voxel_grid = t__full_voxel_grid + t__end - t__start


            data_log_octree['total'] = get_mem_octree(f_octree, 'traditional')['mem_size']
            data_log_octree_colorless['total'] = get_mem_octree(f_octree, 'traditional')['mem_size_colorless']
            data_log_octree_open3d['total'] = get_mem_octree(f_octree)['mem_size']
            data_log_octree_open3d_colorless['total'] = get_mem_octree(f_octree)['mem_size_colorless']

            data_log_voxel_grid['total'] = get_mem_voxel_grid(f_voxel, 'traditional')['mem_size']
            data_log_voxel_grid_colorless['total'] = get_mem_voxel_grid(f_voxel, 'traditional')['mem_size_colorless']
            data_log_voxel_grid_open3d['total'] = get_mem_voxel_grid(f_voxel)['mem_size']
            data_log_voxel_grid_open3d_colorless['total'] = get_mem_voxel_grid(f_voxel)['mem_size_colorless']

            if show:
                t__start = timer()
                o3d.visualization.draw_geometries([f_octree], 'Octree global fixed voxel size')
                t__end = timer()
                self.t__['only_octree_open3d_render'] = t__end - t__start

                t__start = timer()
                o3d.visualization.draw_geometries([f_voxel], 'Voxel grid global fixed voxel size')
                t__end = timer()
                self.t__['only_voxel_grid_open3d_render'] = t__end - t__start


            self.memLogger.log("only_octree", data_log_octree)
            self.memLogger.log("only_octree_colorless", data_log_octree_colorless)
            self.memLogger.log("only_octree_open3d", data_log_octree_open3d)
            self.memLogger.log("only_octree_open3d_colorless", data_log_octree_open3d_colorless)
            self.memLogger.log("only_voxel_grid", data_log_voxel_grid)
            self.memLogger.log("only_voxel_grid_colorless", data_log_voxel_grid_colorless)
            self.memLogger.log("only_voxel_grid_open3d", data_log_voxel_grid_open3d)
            self.memLogger.log("only_voxel_grid_open3d_colorless", data_log_voxel_grid_open3d_colorless)
            self.t__['only_octree_open3d'] = t__full_octree
            self.t__['only_voxel_grid_open3d'] = t__full_voxel_grid






            t__full_octree = 0
            t__full_voxel_grid = 0

            #Generate pure octree World
            # truncate octree
            global_bucket_octree = []
            global_bucket_voxel_grid = []

            data_log_voxel_grid = {'total':0}
            data_log_octree = {'total':0}


            pcd_map = o3d.geometry.PointCloud()
            pcd_map.points = o3d.utility.Vector3dVector([])
            for x in range(len(self.features_objects)):
                pcd_feat_copy = copy.deepcopy(self.features_objects[x])
                if paint_features:
                    pcd_feat_copy.feat.bucket.paint_uniform_color(pcd_feat_copy.feat.color)

                pt_antigo = np.asarray(pcd_map.points)
                pt_novo = np.asarray(pcd_feat_copy.feat.bucket.points)
                cor_antiga = np.asarray(pcd_map.colors)
                cor_nova = np.asarray(pcd_feat_copy.feat.bucket.colors)
                #print("apendando", np.append(pt_antigo, pt_novo, axis=0))
                pcd_map.points = o3d.utility.Vector3dVector(np.append(pt_antigo, pt_novo, axis=0))
                pcd_map.colors = o3d.utility.Vector3dVector(np.append(cor_antiga, cor_nova, axis=0))

            t__start = timer()
            f_octree = o3d.geometry.Octree(max_depth=5)
            f_octree.convert_from_point_cloud(pcd_map)
            t__end = timer()
            t__full_octree = t__full_octree + t__end - t__start

            mini_voxel_size = f_octree.size/(2**(f_octree.max_depth))

            t__start = timer()
            f_voxel = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd_map, voxel_size=mini_voxel_size)
            t__end = timer()
            t__full_voxel_grid = t__full_voxel_grid + t__end - t__start

            data_log_octree['total'] = get_mem_octree(f_octree)['mem_size']
            data_log_voxel_grid['total'] = get_mem_voxel_grid(f_voxel)['mem_size']

            if show:
                t__start = timer()
                o3d.visualization.draw_geometries([f_octree], 'Octree global truncate')
                t__end = timer()
                self.t__['only_octree_truncate_render'] = t__end - t__start

                t__start = timer()
                o3d.visualization.draw_geometries([f_voxel], 'Voxel grid global truncate')
                t__end = timer()
                self.t__['only_voxel_grid_truncate_render'] = t__end - t__start

            self.memLogger.log("only_octree_truncate", data_log_octree)
            self.memLogger.log("only_voxel_grid_truncate", data_log_voxel_grid)
            self.t__['only_octree_truncate'] = t__full_octree
            self.t__['only_voxel_grid_truncate'] = t__full_voxel_grid


            #if self.save_maps:
            #    o3d.io.write_octree("map_outputs/global_octree_fixed_size.json", f_octree)
            #    o3d.io.write_voxel_grid("map_outputs/global_voxel_grid_fixed_size.ply", f_voxel)

            t__full = 0
            # Low level world:
            data_log = {'total':0}
            data_log_colorless = {'total':0}
            global_bucket = []
            
            for x in range(len(self.features_objects)):
                if isinstance(self.features_objects[x].feat,Plane):
                    t__start = timer()
                    global_bucket.append(self.features_objects[x].feat.get_geometry()[0])
                    t__end = timer()
                    t__full = t__full + t__end - t__start
                    
                    data_log[self.features_objects[x].id] = get_mem_feature("plane")['mem_size']
                    data_log['total'] = data_log['total'] + get_mem_feature("plane")['mem_size']
                    data_log_colorless[self.features_objects[x].id] = get_mem_feature("plane")['mem_size_colorless']
                    data_log_colorless['total'] = data_log_colorless['total'] + get_mem_feature("plane")['mem_size_colorless']


                elif isinstance(self.features_objects[x].feat,Cylinder):
                    t__start = timer()
                    global_bucket.append(self.features_objects[x].feat.get_geometry()[0])
                    t__end = timer()
                    t__full = t__full + t__end - t__start

                    data_log[self.features_objects[x].id] = get_mem_feature("cylinder")['mem_size']
                    data_log['total'] = data_log['total'] + get_mem_feature("cylinder")['mem_size']
                    data_log_colorless[self.features_objects[x].id] = get_mem_feature("cylinder")['mem_size_colorless']
                    data_log_colorless['total'] = data_log_colorless['total'] + get_mem_feature("cylinder")['mem_size_colorless']
            
            if show:
                t__start = timer()
                o3d.visualization.draw_geometries(global_bucket)
                t__end = timer()
                self.t__['low_level_world_render'] = t__end - t__start
            self.memLogger.log("low_level_world", data_log)
            self.memLogger.log("low_level_world_colorless", data_log_colorless)
            self.t__['low_level_world'] = t__full

            if self.save_maps:
                sum_map = o3d.geometry.TriangleMesh()
                for geo in global_bucket:
                    sum_map += geo
                o3d.io.write_triangle_mesh("map_outputs/ll_map.ply", sum_map)

            # High level world:
            t__full = 0
            global_bucket = []
            data_log = {'total':0}
            data_log_colorless = {'total':0}
            for x in range(len(self.features_objects)):
                if isinstance(self.features_objects[x].feat,Plane):
                    t__start = timer()
                    global_bucket.append(self.features_objects[x].feat.get_geometry()[0])
                    t__end = timer()
                    t__full = t__full + t__end - t__start

                    data_log[self.features_objects[x].id] = get_mem_feature("plane")['mem_size']
                    data_log['total'] = data_log['total'] + get_mem_feature("plane")['mem_size']
                    data_log_colorless[self.features_objects[x].id] = get_mem_feature("plane")['mem_size_colorless']
                    data_log_colorless['total'] = data_log_colorless['total'] + get_mem_feature("plane")['mem_size_colorless']

                elif isinstance(self.features_objects[x].feat,Cylinder):
                    pcd_feat_copy = self.features_objects[x]
                    if paint_features:
                        pcd_feat_copy.feat.bucket.paint_uniform_color(pcd_feat_copy.feat.color)
                    t__start = timer()
                    high_level_feature, mesh = pcd_feat_copy.feat.get_high_level_feature()
                    t__end = timer()
                    t__full = t__full + t__end - t__start
                    if high_level_feature == 'cuboid':
                        data_log[self.features_objects[x].id] = get_mem_feature("cuboid")['mem_size']
                        data_log['total'] = data_log['total'] + get_mem_feature("cuboid")['mem_size']
                        data_log_colorless[self.features_objects[x].id] = get_mem_feature("cuboid")['mem_size_colorless']
                        data_log_colorless['total'] = data_log_colorless['total'] + get_mem_feature("cuboid")['mem_size_colorless']
                    elif high_level_feature == 'cylinder':
                        data_log[self.features_objects[x].id] = get_mem_feature("cylinder")['mem_size']
                        data_log['total'] = data_log['total'] + get_mem_feature("cylinder")['mem_size']
                        data_log_colorless[self.features_objects[x].id] = get_mem_feature("cylinder")['mem_size_colorless']
                        data_log_colorless['total'] = data_log_colorless['total'] + get_mem_feature("cylinder")['mem_size_colorless']
                    else:
                        data_log[self.features_objects[x].id] = get_mem_pcd(self.features_objects[x].feat.bucket)['mem_size']
                        data_log['total'] = data_log['total'] + get_mem_pcd(self.features_objects[x].feat.bucket)['mem_size']
                        data_log_colorless[self.features_objects[x].id] = get_mem_pcd(self.features_objects[x].feat.bucket)['mem_size_colorless']
                        data_log_colorless['total'] = data_log_colorless['total'] + get_mem_pcd(self.features_objects[x].feat.bucket)['mem_size_colorless']
                        mesh = copy.deepcopy(mesh)
                        if paint_features:
                            mesh = mesh.paint_uniform_color(self.features_objects[x].feat.color)
                    global_bucket.append(mesh)
            
            if show:
                t__start = timer()
                o3d.visualization.draw_geometries(global_bucket)
                t__end = timer()
                self.t__['high_level_world_render'] = t__end - t__start

            self.memLogger.log("high_level_world", data_log)
            self.memLogger.log("high_level_world_colorless", data_log_colorless)
            self.t__['high_level_world'] = t__full

            if self.save_maps:
                sum_map = o3d.geometry.TriangleMesh()
                pcd_map = o3d.geometry.PointCloud()
                pcd_map.points = o3d.utility.Vector3dVector([])
                
                for geo in global_bucket:
                    print(type(geo))
                    if type(geo) == o3d.geometry.TriangleMesh:
                        sum_map += geo
                    elif type(geo) == o3d.geometry.PointCloud:
                        pcd_map += geo
                o3d.io.write_triangle_mesh("map_outputs/hl_map_mesh.ply", sum_map)
                o3d.io.write_point_cloud("map_outputs/hl_map_pcd.pcd", pcd_map)


        self.memLogger.log("time_log", self.t__)
        self.memLogger.next()
        self.memLogger.save_as_json()
        # self.memLogger.save_as_matlab()
        # for x in range(len(self.features_objects)):
        #     if isinstance(self.features_objects[x].feat,Cylinder):
        #         self.features_objects[x].feat.get_high_level_feature()


        # global_bucket = []
        # for x in range(len(self.features_objects)):
        #     if isinstance(self.features_objects[x].feat,Plane):
        #         self.features_objects[x].feat.bucket.paint_uniform_color(self.features_objects[x].feat.color)
        #         pontos = copy.deepcopy(self.features_objects[x].feat.bucket)
        #         pontos.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=10))
        #         radii = [0.1, 0.3]
        #         rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pontos, o3d.utility.DoubleVector(radii))
        #         global_bucket.append(rec_mesh)
        #         print("passou aqui")

        # o3d.visualization.draw_geometries(global_bucket, mesh_show_back_face=True)
        





