import numpy as np
import sys
import copy 

def from_equation_to_feature(eq):
    return np.asarray([[eq[0],eq[1],eq[2],eq[3]]]).T

def from_feature_to_equation(Z):
    return [Z[0,0],Z[1,0],Z[2,0],Z[3,0]]

def from_any_to_feature(Z):
    return Z

def normalize_plane_feature(Z1):
    Z = copy.deepcopy(Z1)
    if Z[3,0] < 0:
        Z[0,0] = Z[0,0]*-1
        Z[1,0] = Z[1,0]*-1
        Z[2,0] = Z[2,0]*-1
        Z[3,0] = Z[3,0]*-1

    Z[0,0] = Z[0,0]/np.sqrt(np.linalg.norm(Z[0:3]))
    Z[1,0] = Z[1,0]/np.sqrt(np.linalg.norm(Z[0:3]))
    Z[2,0] = Z[2,0]/np.sqrt(np.linalg.norm(Z[0:3]))
    #Z[3,0] = Z[3,0]/np.linalg.norm(Z[0:3])

    Z= np.round(Z,4)
    return Z



def apply_h_plane(x1, N1):
    N = copy.deepcopy(N1)
    x = copy.deepcopy(x1)
    N = normalize_plane_feature(N)
    na = N[0,0]
    nb = N[1,0]
    nc = N[2,0]
    nd  = N[3,0]

    n = np.asarray([[na*np.cos(x[2,0]) + nb*np.sin(x[2,0])],
                    [-na*np.sin(x[2,0]) + nb*np.cos(x[2,0])],
                    [nc],
                    [na*x[0,0] + nb*x[1,0] + nd]])

    zp = normalize_plane_feature(n)
    return zp






def get_Hxv_plane(x_state1, N1):
    N = copy.deepcopy(N1)
    x_state = copy.deepcopy(x_state1)

    na = N[0,0]
    nb = N[1,0]
    nc = N[2,0]
    nd = N[3,0]

    x = x_state[0,0]
    y = x_state[1,0]
    theta = x_state[2,0]

    Hx =    np.asarray([[  0,  0,   nb*np.cos(theta) - na*np.sin(theta)],
             [  0,  0, - na*np.cos(theta) - nb*np.sin(theta)],
             [  0,  0,                               0],
             [ na, nb,                               0]])

    return Hx






def get_Hxp_plane(x_state1, N1):
    N = copy.deepcopy(N1)
    x_state = copy.deepcopy(x_state1)

    na = N[0,0]
    nb = N[1,0]
    nc = N[2,0]
    nd = N[3,0]

    x = x_state[0,0]
    y = x_state[1,0]
    theta = x_state[2,0]

    Hz =    np.asarray([[  np.cos(theta), np.sin(theta), 0, 0],
             [ -np.sin(theta), np.cos(theta), 0, 0],
             [           0,          0, 1, 0],
             [           x,          y, 0, 1]])

    return Hz







def apply_g_plane(x1, Zp1):
    Zp= copy.deepcopy(Zp1)
    x = copy.deepcopy(x1)
    Zp = normalize_plane_feature(Zp)
    na = Zp[0,0]
    nb = Zp[1,0]
    nc = Zp[2,0]
    nd  = Zp[3,0]

    n = np.asarray([[na*np.cos(x[2,0]) - nb*np.sin(x[2,0]) ],
                    [na*np.sin(x[2,0]) + nb*np.cos(x[2,0])],
                    [nc],
                    [-(na*np.cos(x[2,0]) - nb*np.sin(x[2,0]))*x[0,0]  -(na*np.sin(x[2,0]) + nb*np.cos(x[2,0]))*x[1,0] + nd]])

    zp = normalize_plane_feature(n)
    return zp




def get_Gx_plane(x_state1, N1):
    N = copy.deepcopy(N1)
    x_state = copy.deepcopy(x_state1)

    na = N[0,0]
    nb = N[1,0]
    nc = N[2,0]
    nd = N[3,0]

    x = x_state[0,0]
    y = x_state[1,0]
    theta = x_state[2,0]

    Gx =    np.asarray([[                             0,                               0,                                       - nb*np.cos(theta) - na*np.sin(theta)],
                         [                             0,                               0,                                         na*np.cos(theta) - nb*np.sin(theta)],
                         [                             0,                               0,                                                                     0],
                         [ nb*np.sin(theta) - na*np.cos(theta), - nb*np.cos(theta) - na*np.sin(theta), x*(nb*np.cos(theta) + na*np.sin(theta)) - y*(na*np.cos(theta) - nb*np.sin(theta))]])

    return Gx




def get_Gz_plane(x_state1, N1):
    N = copy.deepcopy(N1)
    x_state = copy.deepcopy(x_state1)

    na = N[0,0]
    nb = N[1,0]
    nc = N[2,0]
    nd = N[3,0]

    x = x_state[0,0]
    y = x_state[1,0]
    theta = x_state[2,0]

    Gx =    np.asarray([[                    np.cos(theta),                 -np.sin(theta), 0, 0],
                         [                    np.sin(theta),                  np.cos(theta), 0, 0],
                         [                             0,                           0, 1, 0],
                         [ - x*np.cos(theta) - y*np.sin(theta), x*np.sin(theta) - y*np.cos(theta), 0, 1]])


    return Gx





def get_W_plane():
    sigma_x = 0.04/3
    sigma_y = 0.04/3
    sigma_z = 0.04/3
    sigma_d = 0.08/3

    W = np.asarray([[sigma_x**2, 0, 0, 0],
                    [0, sigma_y**2, 0, 0],
                    [0, 0, sigma_z**2, 0],
                    [0, 0, 0, sigma_d**2] ])
    return W

def get_Hw_plane():
    Hw = np.asarray([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    return Hw
