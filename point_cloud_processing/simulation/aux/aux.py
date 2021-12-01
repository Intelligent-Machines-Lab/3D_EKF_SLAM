import open3d as o3d
import numpy as np
import random
import copy 
import matplotlib.pyplot as plt
import cv2
from aux.qhull_2d import *
from aux.min_bounding_rect import *


def get_voxel_vis_from_octree(octree):
    print("converteu")
    voxel_grid = octree.to_voxel_grid()
    octree_copy = voxel_grid.to_octree(max_depth=5)
    o3d.visualization.draw_geometries([octree])
    o3d.visualization.draw_geometries([voxel_grid])
    o3d.visualization.draw_geometries([octree_copy])
    return octree.to_voxel_grid()


def get_plane_segment_info(points, eq):
    dd_plano = get_2d_plane_projection(points, eq)

    hull_points = qhull2D(dd_plano)
    hull_points = hull_points[::-1]
    
    return minBoundingRect(hull_points) # (rot_angle, area, width, height, center_point, corner_points)

def projected_point_into_plane(points, eq):
    dd_plano = get_2d_plane_projection(points, eq)

    ddd_plano= np.c_[ dd_plano, np.zeros(dd_plano.shape[0]) ] + np.asarray([0, 0, -eq[3]])
    # Agora ta tudo em z=0
    inliers_plano_desrotacionado = rodrigues_rot(ddd_plano, [0, 0, 1], [eq[0], eq[1], eq[2]])

    return inliers_plano_desrotacionado

def get_2d_plane_projection(points, eq):

    inliers_plano = rodrigues_rot(copy.deepcopy(points), [eq[0], eq[1], eq[2]], [0, 0, 1])- np.asarray([0, 0, -eq[3]])
    dd_plano = np.delete(inliers_plano, 2, 1)

    return dd_plano


def drawPlane(plane):
    #print(plane.rMatrix.T)
    box = o3d.geometry.TriangleMesh.create_box(width=plane.size[0], height=plane.size[1], depth=0.05).translate(plane.limits_f_plane[0, :])
    box = box.rotate(plane.rMatrix.T, center=(0,0,0)).translate(plane.tMatrix)
    return box


def get_rotationMatrix_from_vectors(u, v):
    # Lets find a vector which is ortogonal to both u and v
    w = np.cross(u, v)
    
    # This orthogonal vector w has some interesting proprieties
    # |w| = sin of the required rotation 
    # dot product of w and goal_normal_plane is the cos of the angle
    c = np.dot(u, v)
    s = np.linalg.norm(w)


    # Now, we compute rotation matrix from rodrigues formula 
    # https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
    # https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d

    # We calculate the skew symetric matrix of the ort_vec
    Sx = np.asarray([[0, -w[2], w[1]],
                   [w[2], 0, -w[0] ],
                   [-w[1], w[0], 0]])
    R = np.eye(3) + Sx + Sx.dot(Sx) * ((1 - c) / (s ** 2))
    return R



def rodrigues_rot(P, n0, n1):
    
    # If P is only 1d array (coords of single point), fix it to be matrix
    P = np.asarray(P)
    if P.ndim == 1:
        P = P[np.newaxis,:]
    
    # Get vector of rotation k and angle theta
    n0 = n0/np.linalg.norm(n0)
    n1 = n1/np.linalg.norm(n1)
    k = np.cross(n0,n1)
    P_rot = np.zeros((len(P),3))
    if(np.linalg.norm(k)!=0):
        k = k/np.linalg.norm(k)

        theta = np.arccos(np.dot(n0,n1))
        
        # Compute rotated points
       
        for i in range(len(P)):
            P_rot[i] = P[i]*np.cos(theta) + np.cross(k,P[i])*np.sin(theta) + k*np.dot(k,P[i])*(1-np.cos(theta))
    else:
        P_rot = P
    return P_rot


def get_rotation_angvel_matrix_bti(angles):

    phi = angles[0]
    theta = angles[1]
    psi = angles[2]

    matr = np.asarray([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                       [0, np.cos(phi), -np.sin(phi)],
                       [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])
    
    return matr


def get_rotation_matrix_bti(angles):
    #print("sedfefaw"+str(angles))
    phi = angles[0]
    theta = angles[1]
    psi = angles[2]

    R11 = np.cos(theta)*np.cos(psi)
    R12 = np.sin(phi)*np.sin(theta)*np.cos(psi) - np.cos(phi)*np.sin(psi)
    R13 = np.cos(phi)*np.sin(theta)*np.cos(psi) + np.sin(phi)*np.sin(psi)

    R21 = np.cos(theta)*np.sin(psi)
    R22 = np.sin(phi)*np.sin(theta)*np.sin(psi) + np.cos(phi)*np.cos(psi)
    R23 = np.cos(phi)*np.sin(theta)*np.sin(psi) - np.sin(phi)*np.cos(psi)

    R31 = -np.sin(theta)
    R32 = np.sin(phi)*np.cos(theta)
    R33 = np.cos(phi)*np.cos(theta)

    matr = np.asarray([[R11, R12, R13],
                       [R21, R22, R23],
                       [R31, R32, R33]])

    #print("matriz de rotação: "+str(matr))
    
    return matr    




def open_pointCloud_from_files(n = 1, folder="images_a_gazebo", end_color = "_rgb.png", end_depth="_depth.png", meters_trunc = 6, showImages = True):
    depth_scale=1/1000
    color_raw = o3d.io.read_image(folder+"/"+str(n)+end_color)
    depth_raw = o3d.io.read_image(folder+"/"+str(n)+end_depth)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw, depth_scale=1/depth_scale, depth_trunc=meters_trunc, convert_rgb_to_intensity=False)
    print(rgbd_image)

    
    if(showImages):
        plt.subplot(1, 2, 1)
        plt.title('Redwood grayscale image')
        plt.imshow(rgbd_image.color)
        plt.subplot(1, 2, 2)
        plt.title('Redwood depth image')
        plt.imshow(rgbd_image.depth)
        plt.show()

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    return pcd



def open_pointCloud_from_rgb_and_depth(color_raw, depth_raw, meters_trunc=5, showImages = True):
    depth_scale=1/1000

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw, depth_scale=1/depth_scale, depth_trunc=meters_trunc, convert_rgb_to_intensity=False)
    



    if(showImages):
        plt.subplot(1, 2, 1)
        plt.title('Redwood grayscale image')
        plt.imshow(rgbd_image.color)
        plt.subplot(1, 2, 2)
        plt.title('Redwood depth image')
        plt.imshow(rgbd_image.depth)
        plt.show()

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    return pcd

def get_yawpitchroll_from_rotation_matrix(rot_matrix):
    yaw = np.arctan2(rot_matrix[1][0],rot_matrix[0][0])
    pitch = np.arctan2(-rot_matrix[2][0],np.sqrt(rot_matrix[2][1]**2+rot_matrix[2][2]**2))
    roll = np.arctan2(rot_matrix[2][1],rot_matrix[2][2])
    
    return yaw, pitch, roll  

def get_xyz_yawpitchroll_from_transf_matrix(transf_matrix):
    rot_matrix = transf_matrix[:3,:3]
    yaw, pitch, roll = get_yawpitchroll_from_rotation_matrix(rot_matrix)
    x, y, z = transf_matrix[0,3], transf_matrix[1,3], transf_matrix[2,3]
    return x, y, z, yaw, pitch, roll

def get_diff_angles(ang1, ang2):
    return np.arctan2(np.sin(ang1-ang2), np.cos(ang1-ang2))

def get_sum_angles(ang1, ang2):
    return np.arctan2(np.sin(ang1+ang2), np.cos(ang1+ang2))



def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    #print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

def from_camera_to_inertial(pcd):
    pcd = pcd.rotate(get_rotationMatrix_from_vectors([0, 0, -1], [1,0,0]), center=(0,0,0))
    pcd = pcd.rotate(get_rotationMatrix_from_vectors([0, 1, 0], [0,0,-1]), center=(0,0,0))
    return pcd

def distance_from_points_to_axis(p, axis, p_axis):
    p = np.asarray([p])

    n_points = p.shape[0]

    axis_stack =  np.stack([axis]*n_points,0)

    dist_pt = np.cross(axis_stack, (p_axis- p))
    dist_pt = np.linalg.norm(dist_pt, axis=1)
    return dist_pt

def distance_from_points_to_plane(p, plane_eq):
    #print(p)
    p = np.asarray([p])

    dist_pt = (plane_eq[0]*p[:,0]+plane_eq[1]*p[:, 1]+plane_eq[2]*p[:, 2]+plane_eq[3])/np.sqrt(plane_eq[0]**2+plane_eq[1]**2+plane_eq[2]**2)
    return dist_pt


def distance_from_two_lines(e1, e2, r1, r2):
    # e1, e2 = Direction vector
    # r1, r2 = Point where the line passes through

    # Find the unit vector perpendicular to both lines
    n = np.cross(e1, e2)
    n /= np.linalg.norm(n)

    # Calculate distance
    d = np.dot(n, r1 - r2)

    return d

def get_point_between_two_lines(e1, e2, r1, r2):
    # e1, e2 = Direction vector
    # r1, r2 = Point where the line passes through

    # Find the unit vector perpendicular to both lines
    n = np.cross(e1, e2)
    n /= np.linalg.norm(n)

    # Calculate distance
    d = np.dot(n, r1 - r2)

    RHS = r1 - r2
    LHS = np.array([e1, -e2, n]).T
    res = np.linalg.solve(LHS, RHS)
    q1 = r1-res[0]*e1
    #print(np.linalg.solve(LHS, RHS))

    return q1

# def create_animation():
#     image_folder = 'animations'
#     video_name = 'video.avi'

#     images = [img for img in os.listdir(image_folder) if (img.startswith("global-") and img.endswith(".png"))]
#     frame = cv2.imread(os.path.join(image_folder, images[0]))
#     height, width, layers = frame.shape

#     video = cv2.VideoWriter(video_name, 0, 1, (width,height))

#     for image in images:
#         video.write(cv2.imread(os.path.join(image_folder, image)))

#     cv2.destroyAllWindows()
#     video.release()