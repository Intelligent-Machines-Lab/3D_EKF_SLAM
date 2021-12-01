import open3d as o3d
import numpy as np
import random
import copy 
import matplotlib.pyplot as plt
from aux.plane import Plane
from aux.cylinder import Cylinder
from aux.aux import *
import settings
class LocalScene:

    def __init__(self, pointCloud, filter_radius=True):
        self.pointCloud = pointCloud # Total point cloud
        self.npontos = np.asarray(self.pointCloud.points).shape[0]
        self.filter_radius = filter_radius
        #print(self.npontos)
        self.pointCloud_notMainPlanes = []
        self.pointCloud_objects = []

        self.mainPlanes = [] # List of Plane
        self.secundaryPlanes = [] # If is not a cylinder, fit smaller planes
        self.mainCylinders = [] # List of Cylinder

        self.groundNormal = [] # Vector normal to the ground
        self.groundEquation = []
        self.groundID = 0

        self.circulation_cylinder = 0.04
        
        #self.percentage_std_radius = 0.1 #good one in gazebo
        self.percentage_std_radius = settings.get_setting('percentage_std_radius')
        
    def custom_draw_geometry(self):
        # The following code achieves the same effect as:
        # o3d.visualization.draw_geometries([pcd])
        vis_original = o3d.visualization.Visualizer()

        vis_original.create_window(window_name='Original', width=960, height=540, left=0, top=0)
        vis_original.add_geometry(self.pointCloud)

        feat = self.getMainPlanes()
        feat.extend(self.getCylinders(showPointCloud=False))
        feat.extend(self.getSecundaryPlanes())
        vis_feature = o3d.visualization.Visualizer()
        vis_feature.create_window(window_name='Feature', width=960, height=540, left=960, top=0)
        for x in range(len(feat)):
            vis_feature.add_geometry(feat[x])

        while True:
            vis_original.update_geometry(self.pointCloud)
            if not vis_original.poll_events():
                break
            vis_original.update_renderer()

            feat = self.getMainPlanes()
            feat.extend(self.getCylinders(showPointCloud=False))
            for x in range(len(feat)):
                vis_feature.update_geometry(feat[x])
            
            if not vis_feature.poll_events():
                break
            vis_feature.update_renderer()

        vis_original.destroy_window()
        vis_feature.destroy_window()




    def findMainPlanes(self):
        outlier_cloud = copy.deepcopy(self.pointCloud)
        inlier_cloud_list = []
        cloud_sobras = o3d.geometry.PointCloud()
        while(True):
            # Ransac planar
            points = outlier_cloud
            if np.asarray(points.points).shape[0] < 4:
                break
            
            p = Plane()
            best_eq, best_inliers, valid = p.findPlane(points, thresh=settings.get_setting('ransac_tresh'), minPoints=3, maxIteration=1000)
            inlier_cloud_list.append(copy.deepcopy(outlier_cloud).select_by_index(best_inliers).paint_uniform_color([random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)]))
            qtn_inliers = best_inliers.shape[0]
            if(qtn_inliers < int(0.15*self.npontos)):
                break
            out = copy.deepcopy(outlier_cloud).select_by_index(best_inliers, invert=True)
            points = out



            if(valid):

                p.color = [random.uniform(0.3, 1), random.uniform(0.3, 1), random.uniform(0.3, 1)]
                self.mainPlanes.append(p)
            else:
                cloud_sobras += copy.deepcopy(outlier_cloud).select_by_index(best_inliers)

            outlier_cloud = outlier_cloud.select_by_index(best_inliers, invert=True)

            if(np.asarray(points.points).shape[0] < int(0.004*self.npontos)):
                break
            if(self.filter_radius):
                #print("TA NO MAIN")
                cl, ind = outlier_cloud.remove_radius_outlier(nb_points=int(0.0012*self.npontos), radius=0.2)
                #display_inlier_outlier(outlier_cloud, ind)
                outlier_cloud = outlier_cloud.select_by_index(ind)
        
        #o3d.visualization.draw_geometries(inlier_cloud_list)
        self.pointCloud_notMainPlanes = outlier_cloud + cloud_sobras


    def getMainPlanes(self):
        pointCloudList = []
        for i in range(len(self.mainPlanes)):
            #print(i)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.mainPlanes[i].inliers)
            #print(self.mainPlanes[i].equation)
            
            pointCloudList.append(pcd.paint_uniform_color(self.mainPlanes[i].color))
            
        if(self.groundNormal and not self.groundID == -1):
            mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(origin=[0, 0, 0], size=0.5)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.mainPlanes[self.groundID].inliers)
            centerPCD = pcd.get_center()
            #print(self.groundNormal)
            mesh.rotate(get_rotationMatrix_from_vectors([0, 0, 1], self.groundNormal), center=(0, 0, 0)).translate(centerPCD)
            pointCloudList.append(mesh)
        return pointCloudList
        #o3d.visualization.draw_geometries(pointCloudList)

    def showNotPlanes(self):
        o3d.visualization.draw_geometries([self.pointCloud_notMainPlanes])

    def showObjects(self):
        o3d.visualization.draw_geometries(self.pointCloud_objects)



    def defineGroundNormal(self, ground_eq_reserva = [0, 0, -1, 1.2]):
        normalCandidatesY = []
        for i in range(len(self.mainPlanes)):
            normalCandidatesY.append(abs(self.mainPlanes[i].equation[2]))
        # for i in range(len(self.mainPlanes)):
        #     normalCandidatesY.append(abs(self.secundaryPlanes[i].equation[2]))
            #print("Equacaodoplano ",self.mainPlanes[i].equation)
        if normalCandidatesY:
            valMax = max(normalCandidatesY)
            idMax = normalCandidatesY.index(valMax)
            if(valMax > 0.95):
                self.groundNormal = np.asarray([self.mainPlanes[idMax].equation[0], self.mainPlanes[idMax].equation[1], self.mainPlanes[idMax].equation[2]])
                self.groundEquation = np.asarray(self.mainPlanes[idMax].equation)
                if(self.groundNormal[2] > 0):
                    self.groundNormal = -self.groundNormal
                    self.groundEquation = -self.groundEquation
                self.groundID = idMax
            else:
                #print("Tendo que usar equação reserva do chão: ", self.groundEquation)
                self.groundEquation = np.asarray(ground_eq_reserva)
                self.groundNormal = np.asarray([self.groundEquation[0], self.groundEquation[1], self.groundEquation[2]])
                self.groundID = -1
        else:
            print("Tendo que usar equação reserva do chão: ", ground_eq_reserva)
            self.groundEquation = np.asarray(ground_eq_reserva)
            self.groundNormal = np.asarray([self.groundEquation[0], self.groundEquation[1], self.groundEquation[2]])
            self.groundID = -1
        self.groundNormal = self.groundNormal.tolist()
        self.groundEquation = self.groundEquation.tolist()
        #print("Ground normal: "+str(self.groundNormal))


    def clusterizeObjects(self):
        filtered_not_planes = self.pointCloud_notMainPlanes

        if  (np.asarray(filtered_not_planes.points).shape[0]) > 100 and np.asarray(filtered_not_planes.points).shape[0] < 100000:
            with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
                labels = np.array(filtered_not_planes.cluster_dbscan(eps=0.1, min_points=int(0.001*self.npontos/10), print_progress=False))

            max_label = labels.max()
            print(f"point cloud has {max_label + 1} clusters")
            # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
            # colors[labels < 0] = 0
            # filtered_not_planes.colors = o3d.utility.Vector3dVector(colors[:, :3])
            # o3d.visualization.draw_geometries([filtered_not_planes])
            cluster_array = []
            for n_cluster in range(max_label+1):
                index_from_cluster = np.where(labels == n_cluster)[0]
                cluster = filtered_not_planes.select_by_index( index_from_cluster.tolist())
                cluster_qnt_points = np.asarray(cluster.points).shape[0]
                if(cluster_qnt_points > int(0.004*self.npontos)):
                    self.pointCloud_objects.append(cluster)


    def fitCylinder(self):
        for i_obj in range(len(self.pointCloud_objects)):
            cyl = Cylinder()
            cyl.find(self.pointCloud_objects[i_obj], thresh=0.05, maxIteration=1000, forceAxisVector = self.groundNormal, useRANSAC = False)
            cyl.color = [random.uniform(0.3, 1), random.uniform(0.3, 1), random.uniform(0.3, 1)]
            cyl.calculatePlanification(showNormal=False)
            self.mainCylinders.append(cyl)


    def getCylinders(self, maxRadius= 9999999, showPointCloud = True):
        cymesh = []
        for i_obj in range(len(self.mainCylinders)):
            if(self.mainCylinders[i_obj].radius < maxRadius):
                R = get_rotationMatrix_from_vectors([0, 0, 1], self.mainCylinders[i_obj].normal)
                mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=self.mainCylinders[i_obj].radius, height=(self.mainCylinders[i_obj].height[1]-self.mainCylinders[i_obj].height[0]))
                mesh_cylinder.compute_vertex_normals()
                mesh_cylinder.paint_uniform_color(self.mainCylinders[i_obj].color)
                mesh_cylinder = mesh_cylinder.rotate(R, center=[0, 0, 0])
                mesh_cylinder = mesh_cylinder.translate((self.mainCylinders[i_obj].center[0], self.mainCylinders[i_obj].center[1], self.mainCylinders[i_obj].center[2]))
                
            # if(self.mainCylinders[i_obj].circulation_mean > self.circulation_cylinder):
            if(self.is_cylinder(self.mainCylinders[i_obj])):
                cymesh.append(mesh_cylinder)
                obb = cymesh[-1].get_oriented_bounding_box()
                obb.color = (0,1,0)
                cymesh.append(obb)
            else:
                cymesh.append(mesh_cylinder)
                obb = cymesh[-1].get_oriented_bounding_box()
                obb.color = (1,0,0)
                cymesh.append(obb)

        obcylinder = []     

        if(showPointCloud):
            obcylinder = copy.deepcopy(self.pointCloud_objects)
            obcylinder.extend(cymesh)
        else:
            obcylinder = cymesh
        return obcylinder
        #o3d.visualization.draw_geometries(obcylinder)

    def is_cylinder(self, cylinder):
        cylinder_condition = True
        cylinder_condition = cylinder_condition and (cylinder.radius < 1.5)
        
        #cylinder_condition = cylinder_condition and (cylinder.circulation_mean > self.circulation_cylinder)
        cylinder_condition = cylinder_condition and (cylinder.radius_std/cylinder.radius_mean < self.percentage_std_radius)
        return cylinder_condition
            


    def showFeatures(self):
        feat = self.getMainPlanes()
        feat.extend(self.getCylinders(showPointCloud=False))
        #o3d.visualization.draw_geometries(feat)

    def getProprieties(self):
        vet_mainPlanes = []
        for x in range(len(self.mainPlanes)):
            vet_mainPlanes.append(self.mainPlanes[x].getProrieties())

        vet_mainCylinders = []
        for x in range(len(self.mainCylinders)):
            vet_mainCylinders.append(self.mainCylinders[x].getProrieties())

        vet_secondaryCylinders = []
        for x in range(len(self.secundaryPlanes)):
            vet_secondaryCylinders.append(self.secundaryPlanes[x].getProrieties())
        
        return {"groundNormal": self.groundNormal, "planes": vet_mainPlanes, "cylinders": vet_mainCylinders, "secundaryplanes": vet_secondaryCylinders}


    def findSecundaryPlanes(self):
        removeIndexes = []
        for i_cyl in range(len(self.mainCylinders)):
            print('ANALIZANDO CILINDRO COM RAIO: ', self.mainCylinders[i_cyl].radius)
            # if(self.mainCylinders[i_cyl].circulation_mean < self.circulation_cylinder):
            if(not self.is_cylinder(self.mainCylinders[i_cyl])):
                removeIndexes.append(i_cyl)
                outlier_cloud = copy.deepcopy(self.pointCloud_objects[i_cyl])
                while(True):
                    # Ransac planar
                    points = outlier_cloud
                    
                    p = Plane()
                    best_eq, best_inliers, valid = p.findPlane(points, thresh=0.05, minPoints=int(0.0004*self.npontos), maxIteration=400)
                    qtn_inliers = np.asarray(best_inliers).shape[0]
                    if(qtn_inliers < int(0.0005*self.npontos)):
                        break
                    out = copy.deepcopy(outlier_cloud).select_by_index(best_inliers, invert=True)
                    points = out
                    outlier_cloud = outlier_cloud.select_by_index(best_inliers, invert=True)
                    if(valid):
                        p.color = [random.uniform(0.3, 1), random.uniform(0.3, 1), random.uniform(0.3, 1)]
                        self.secundaryPlanes.append(p)
                        
                    if(np.asarray(outlier_cloud.points).shape[0] < 10):
                        break
                    if(self.filter_radius):
                        #print("TA NO SECUNDÁRIO")
                        cl, ind = outlier_cloud.remove_radius_outlier(nb_points=int(0.0012*self.npontos), radius=0.2)
                        #display_inlier_outlier(outlier_cloud, ind)
                        outlier_cloud = outlier_cloud.select_by_index(ind)
                    if(np.asarray(outlier_cloud.points).shape[0] < 10):
                        break
            #else:
                #removeIndexes.append(i_cyl)
        self.mainCylinders = [i for j, i in enumerate(self.mainCylinders) if j not in removeIndexes]

    def getSecundaryPlanes(self):
        pointCloudList = []
        for i in range(len(self.secundaryPlanes)):
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.secundaryPlanes[i].inliers)
            pointCloudList.append(pcd.paint_uniform_color(self.secundaryPlanes[i].color))
        return pointCloudList


                