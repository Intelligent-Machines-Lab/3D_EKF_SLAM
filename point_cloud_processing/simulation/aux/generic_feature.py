import open3d as o3d
import numpy as np
import random
import copy 
from aux.cylinder import Cylinder
from aux.plane import Plane
from aux.cuboid import Cuboid
from aux import *
from aux.aux_ekf import *
from timeit import default_timer as timer
import settings
class Generic_feature:

    def __init__(self, feat, id=0, ground_equation = [0, 0, 0, 0]):
        self.ground_equation = ground_equation
        self.feat = feat
        self.id = id
        self.t__update = 0
        self.t__bucket_augmentation =0
        self.running_geo = {"total": 1, "plane":0, "cylinder":0, "cuboid":0}
        if isinstance(self.feat,Plane):
            self.running_geo['plane'] = 1

    # FUNÇÃO NÃO USADA!!!!!!!!
    def verifyCorrespondence(self, compare_feat, ekf = ekf):
        if isinstance(self.feat,Plane):
            if isinstance(compare_feat.feat,Plane):


                normal_feature = np.asarray([self.feat.equation[0], self.feat.equation[1], self.feat.equation[2]])
                normal_candidate = np.asarray([compare_feat.feat.equation[0], compare_feat.feat.equation[1], compare_feat.feat.equation[2]])
                # Align normals
                bigger_axis = np.argmax(np.abs(normal_feature))
                if not (np.sign(normal_feature[bigger_axis]) == np.sign(normal_candidate[bigger_axis])):
                    normal_candidate = -normal_candidate
                errorNormal = (np.abs((normal_feature[0]-normal_candidate[0]))+np.abs((normal_feature[1]-normal_candidate[1]))+np.abs((normal_feature[2]-normal_candidate[2])))
                
                if(errorNormal>0.3):
                    return False
                else:

                    d = aux.distance_from_points_to_plane( compare_feat.feat.centroid, self.feat.equation)
                    #print("DISTANCIA DO PLANO PRA CENTROIDE DO OUTRO PLANO: "+str(d))
                    if np.abs(d[0]) > 0.2:
                        return False
                    else:
                        area1 = self.feat.width*self.feat.height
                        area2 = compare_feat.feat.width*compare_feat.feat.height

                        if(area1/area2 < 0.05or area1/area2 > 20):
                            return False
                        else:
                            d_maior = np.amax([self.feat.width,self.feat.height, compare_feat.feat.width,compare_feat.feat.height])
                            if(np.linalg.norm((self.feat.centroid - compare_feat.feat.centroid)) < d_maior):
                                #print("Encontrou correspondencia")
                                #print("Original feature: "+str(self.feat.equation))
                                #print("Candidate feature: "+str(compare_feat.feat.equation))
                                #print("Erro "+str(errorNormal))
                                Z = compare_feat.feat.equation[3]*np.asarray([[compare_feat.feat.equation[0]],[compare_feat.feat.equation[1]],[compare_feat.feat.equation[2]]])
                                Z = apply_h_plane(ekf.x_m, Z)
                                N = ekf.upload_plane(Z, self.id)
                                d = np.linalg.norm(N)
                                a = N[0,0]/d
                                b = N[1,0]/d
                                c = N[2,0]/d
                                neweq = [a, b, c, d]
                                if(self.feat.append_plane(compare_feat, neweq)):
                                    self.running_geo["plane"] = self.running_geo["plane"]+1
                                    self.running_geo["total"] = self.running_geo["total"]+1

                                    innovation = 0
                                    print("INNOVATION:   :   ", innovation)
                                    return True
                                else:
                                    return True
            if isinstance(compare_feat.feat,Cylinder):
                cyl = compare_feat.feat
                pla = self.feat
                plane_height_cylinder_normal = pla.get_height(cyl.normal)
                cylinder_height = cyl.height[1]-cyl.height[0]
                #print("Altura plano")
                #print(plane_height_cylinder_normal)
                #print("Altura cilindro")
                #print(cylinder_height)

                if(np.abs(plane_height_cylinder_normal - cylinder_height) > 0.5):
                    centroid_plane_to_cylinder_axis = aux.distance_from_points_to_axis(pla.centroid, cyl.normal, cyl.center)
                    if((np.abs(centroid_plane_to_cylinder_axis[0]) < cyl.radius*1.2)):
                        dim_media = (pla.width+pla.height)/2
                        #print("Erro entre dim média e raio do cilindro: ", np.abs(dim_media -cyl.radius)/cyl.radius)
                        if (np.abs(dim_media -cyl.radius)/cyl.radius) < 0.5:
                            #print("plano tampa do cilindro")
                            self.running_geo["plane"] = self.running_geo["plane"]+1
                            self.running_geo["total"] = self.running_geo["total"]+1
                            return True
                    return False
                else:
                    #print("Verificando distância entre plano e cilindro")
                    centroid_plane_to_cylinder_axis = aux.distance_from_points_to_axis(pla.centroid, cyl.normal, cyl.center)
                    #print(centroid_plane_to_cylinder_axis)
                    if((np.abs(centroid_plane_to_cylinder_axis[0]) > cyl.radius*1.2) ):
                        return False
                    else:
                        #print("Encontrou correspondencia")
                        self.running_geo["plane"] = self.running_geo["plane"]+1
                        self.running_geo["total"] = self.running_geo["total"]+1
                        return True
                

        if isinstance(self.feat,Cylinder):
            if isinstance(compare_feat.feat,Cylinder):
                if(np.linalg.norm(self.feat.center - compare_feat.feat.center) > 1):
                    return False
                else:
                    if(self.feat.radius - compare_feat.feat.radius)>0.5:
                        return False
                    else:
                        #print("Encontrou correspondencia")
                        #print("Original feature: "+str(self.feat.center))
                        #print("Candidate feature: "+str(compare_feat.feat.center))
                        #print("Original feature radius: "+str(self.feat.radius))
                        #print("Candidate feature radius: "+str(compare_feat.feat.radius))
                        self.running_geo["cylinder"] = self.running_geo["cylinder"]+1
                        self.running_geo["total"] = self.running_geo["total"]+1
                        return True
            if isinstance(compare_feat.feat,Plane):
                pla= compare_feat.feat
                cyl= self.feat
                plane_height_cylinder_normal = pla.get_height(cyl.normal)
                cylinder_height = cyl.height[1]-cyl.height[0]
                #print("Altura plano")
                #print(plane_height_cylinder_normal)
                #print("Altura cilindro")
                #print(cylinder_height)

                #Vamos verificar se a centroide até o chão tem a altura do cilindro (chapéuzinho do cilindro)


                if(np.abs(plane_height_cylinder_normal - cylinder_height) > 0.5):
                    centroid_plane_to_cylinder_axis = aux.distance_from_points_to_axis(pla.centroid, cyl.normal, cyl.center)
                    if((np.abs(centroid_plane_to_cylinder_axis[0]) < cyl.radius*1.2)):
                        dim_media = (pla.width+pla.height)/2
                        #print("Erro entre dim média e raio do cilindro: ", np.abs(dim_media -cyl.radius)/cyl.radius)
                        if (np.abs(dim_media -cyl.radius)/cyl.radius) < 1:
                            #print("plano tampa do cilindro")
                            self.running_geo["plane"] = self.running_geo["plane"]+1
                            self.running_geo["total"] = self.running_geo["total"]+1
                            return True
                    return False
                else:
                    #print("Verificando distância entre plano e cilindro")
                    centroid_plane_to_cylinder_axis = aux.distance_from_points_to_axis(pla.centroid, cyl.normal, cyl.center)
                    #print(centroid_plane_to_cylinder_axis)
                    if((np.abs(centroid_plane_to_cylinder_axis[0]) > cyl.radius*1.2) ):
                        return False
                    else:
                        #print("Encontrou correspondencia")
                        self.running_geo["plane"] = self.running_geo["plane"]+1
                        self.running_geo["total"] = self.running_geo["total"]+1
                        return True

        if isinstance(self.feat,Cuboid):
            if isinstance(compare_feat.feat,Plane):
                dim_cuboid = np.asarray([self.feat.width, self.feat.depth, self.feat.height])
                dimax = np.amax(dim_cuboid)
                # Verifica se o centroide do plano está em um raio de alcance de maior dimensão do centroide do cuboide
                if(np.linalg.norm(self.feat.centroid - compare_feat.feat.centroid) < dimax*1.2):
                    compara1 = dim_cuboid - compare_feat.feat.width
                    compara2 = dim_cuboid - compare_feat.feat.height
                    dimproxima1 = np.amin(np.abs(compara1))
                    dimproxima2 = np.amin(np.abs(compara2))
                    # Verifica se dimensões do cupo são menores ou próximas do plano que está sendo comparado
                    if((dimproxima1 < 0.2 or dimproxima2 < 0.2) or (np.all(compara1>0) and np.all(compara2>0) )):
                        #print("Encontrou correspondencia NO CUBOIDE")
                        self.running_geo["cuboid"] = self.running_geo["cuboid"]+1
                        self.running_geo["total"] = self.running_geo["total"]+1
                        return True


    def correspond(self, compare_feat, ekf = ekf):
        print(compare_feat.feat)
        print(self.feat)
        self.t__bucket_augmentation = 0
        if isinstance(self.feat,Plane):
            if isinstance(compare_feat.feat,Plane):
                t__start = timer()
                d_maior = np.amax([self.feat.width,self.feat.height, compare_feat.feat.width,compare_feat.feat.height])
                if(np.linalg.norm((self.feat.centroid - compare_feat.feat.centroid)) < d_maior*settings.get_setting('merge_plane_size_percentage')):
                    Z = np.asarray([[compare_feat.feat.equation[0],compare_feat.feat.equation[1],compare_feat.feat.equation[2],compare_feat.feat.equation[3]]]).T
                    Z = apply_h_plane(ekf.x_m, Z)
                    N_TESTE = ekf.upload_plane(Z, self.id, only_test= True)
                    plane_cobaia = copy.deepcopy(self.feat)
                    if(plane_cobaia.append_plane(copy.deepcopy(compare_feat), copy.deepcopy(from_feature_to_equation(N_TESTE)), is_cobaia = True)):
                        N = ekf.upload_plane(Z, self.id, only_test= False)
                        self.t__update = timer() - t__start
                        self.feat.append_plane(compare_feat, copy.deepcopy(from_feature_to_equation(N)))
                        self.t__bucket_augmentation = self.feat.t__bucket
                        return True
                    else:
                            return False
                else:
                    return False
                # neweq = [N[0,0], N[1,0], N[2,0], N[3,0]]

                # plane_cobaia = copy.deepcopy(self.feat)
                # if(plane_cobaia.append_plane(compare_feat, neweq)):
                #     N = ekf.upload_plane(Z, self.id, only_test= False)
                #     self.feat.append_plane(compare_feat, neweq)
                #     self.running_geo["plane"] = self.running_geo["plane"]+1
                #     self.running_geo["total"] = self.running_geo["total"]+1
                #     return True
                # else:
                #     return False

        if isinstance(self.feat,Cylinder):
            if isinstance(compare_feat.feat,Cylinder): 
                Z = np.asarray([[compare_feat.feat.center[0]],[compare_feat.feat.center[1]],[compare_feat.feat.center[2]]])
                C = apply_h_point(ekf.x_m, Z)
                Z_new = ekf.upload_point(C, self.id, only_test= True)
                cylinder_cobaia = copy.deepcopy(self.feat)
                if(cylinder_cobaia.append_cylinder(compare_feat, Z_new)):
                    t__start = timer()
                    Z_new_2 = ekf.upload_point(C, self.id, only_test= False)
                    self.t__update = timer() - t__start
                    self.feat.append_cylinder(compare_feat, Z_new_2)
                    self.running_geo["cylinder"] = self.running_geo["cylinder"]+1
                    self.running_geo["total"] = self.running_geo["total"]+1
                    self.t__bucket_augmentation = self.feat.t__bucket*2
                    return True
                else:
                    self.t__bucket_augmentation = self.feat.t__bucket
                    return False

    def getProprieties(self):
        prop = self.feat.getProrieties()
        prop["runner_geo"] = self.running_geo
        prop["id"] = self.id
        return prop