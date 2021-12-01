import open3d as o3d
import numpy as np
import random
import copy 
from aux import *
from aux.qhull_2d import *
from aux.min_bounding_rect import *
from aux.plane import Plane

class Cuboid:

    def __init__(self, plane1, plane2, plane3, ground_normal):
        self.ground_normal = ground_normal
        self.color = plane1.color
        normal1 = np.asarray([plane1.equation[0],plane1.equation[1],plane1.equation[2]])
        normal2 = np.asarray([plane2.equation[0],plane2.equation[1],plane2.equation[2]])
        normal3 = np.asarray([plane3.equation[0],plane3.equation[1],plane3.equation[2]])
        encontro = aux.get_point_between_two_lines(normal1, normal2, plane1.centroid, plane2.centroid)
        encontro2 = aux.get_point_between_two_lines(normal1, normal3, plane1.centroid, plane3.centroid)
        encontro3 = aux.get_point_between_two_lines(normal2, normal3, plane2.centroid, plane3.centroid)
        self.centroid = (encontro+encontro2+encontro3)/3
        alturas = np.asarray([plane1.get_height(self.ground_normal), plane2.get_height(self.ground_normal), plane3.get_height(self.ground_normal)])
        temp = np.partition(-alturas, 2)
        result = -temp[:2]
        self.height = (result[0]+result[1])/2
        #print("ALTURA DO CUBOIDE: ", self.height)
        dim1 = np.asarray([plane1.width, plane1.height])
        dim2 = np.asarray([plane2.width, plane2.height])
        dim3 = np.asarray([plane3.width, plane3.height])
        #print("Dim1 : ", dim1)
        #print("Dim2 : ", dim2)
        #print("Dim3 : ", dim3)
        inlier_cubo = np.vstack((plane1.points_main, plane2.points_main, plane3.points_main))



        inliers_cubo = aux.rodrigues_rot(copy.deepcopy(inlier_cubo), [self.ground_normal[0], self.ground_normal[1], self.ground_normal[2]], [0, 0, 1])
        dd_plano = np.delete(inliers_cubo, 2, 1)
        # Fita retângulo de menor área
        hull_points = qhull2D(dd_plano)
        hull_points = hull_points[::-1]
        (rot_angle, area, width, height, center_point, corner_points) = minBoundingRect(hull_points)
        self.depth = height
        self.width = width
        self.rot_angle = rot_angle
        #print("largura: ",width)
        #print("comprimento:  ",height)


    def get_geometry(self):
        mesh_box = o3d.geometry.TriangleMesh.create_box(width=self.width, height=self.depth, depth=self.height)
        mesh_box = mesh_box.translate(np.asarray([-self.width/2, -self.depth/2, -self.height/2]))
        mesh_box = mesh_box.rotate(aux.get_rotation_matrix_bti([0, 0, self.rot_angle]), center=np.asarray([0, 0, 0]))
        mesh_box.compute_vertex_normals()
        mesh_box.paint_uniform_color(self.color)
        # center the box on the frame
        # move to the plane location
        mesh_box = mesh_box.rotate(aux.get_rotationMatrix_from_vectors([0, 0, 1], [self.ground_normal[0], self.ground_normal[1], self.ground_normal[2]]), center=np.asarray([0, 0, 0]))
        mesh_box = mesh_box.translate(np.asarray(self.centroid))

        return mesh_box

    def getProrieties(self):
        return {"color": self.color, "centroid":self.centroid,
                "height": self.height, "width": self.width, "depth":self.width}