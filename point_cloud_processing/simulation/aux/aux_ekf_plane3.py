import numpy as np
import sys
import copy 
from aux.aux import *

def from_equation_to_feature(eq):
    return eq[3]*np.asarray([[eq[0]], [eq[1]], [eq[2]]])

def from_feature_to_equation(Z):
    d = np.linalg.norm(Z)
    a = Z[0,0]/d
    b = Z[1,0]/d
    c = Z[2,0]/d
    return [a,b,c,d]

def from_any_to_feature(Z):
    Zp= copy.deepcopy(Z)

    plane_4_param = Zp.shape[0] == 4
    if plane_4_param:
        print('Era vir 3, mas veio 4')
        Zp = Zp[3,0]*np.asarray([[Zp[0,0]], [Zp[1,0]], [Zp[2,0]]])

    return Zp

def normalize_plane_feature(Z1):
    Z = copy.deepcopy(Z1)

    Z= np.round(Z,4)
    return Z

def apply_h_plane(x1, N1):
    N = copy.deepcopy(N1)
    x = copy.deepcopy(x1)
    plane_4_param = N.shape[0] == 4
    N = from_any_to_feature(N)

    d = np.linalg.norm(N)
    a = N[0,0]/d
    b = N[1,0]/d
    c = N[2,0]/d

    ux = a*(d + a*x[0,0] + b*x[1,0])
    uy = b*(d + a*x[0,0] + b*x[1,0])
    uz = c*(d + a*x[0,0] + b*x[1,0])

    zp = np.asarray( [[np.cos(x[2,0])*ux + np.sin(x[2,0])*uy],
                      [-np.sin(x[2,0])*ux + np.cos(x[2,0])*uy],
                      [uz]])

    if plane_4_param:
        d = np.linalg.norm(zp)
        a = zp[0,0]/d
        b = zp[1,0]/d
        c = zp[2,0]/d
        zp = np.asarray([[a], [b], [c], [d]])

    return zp


def get_Hxv_plane(x1, N1):
    
    N = copy.deepcopy(N1)
    x = copy.deepcopy(x1)

    N = from_any_to_feature(N)

    d = np.linalg.norm(N)
    a = N[0,0]/d
    b = N[1,0]/d
    c = N[2,0]/d

    ux = a*(d+a*x[0,0] + b*x[1,0])
    uy = b*(d+a*x[0,0] + b*x[1,0])
    uz = c*(d+a*x[0,0] + b*x[1,0])

    hx11 = a*a*np.cos(x[2,0]) + a*b*np.sin(x[2,0])
    hx12 = a*b*np.cos((x[2,0])) + b*b*np.sin((x[2,0]))
    hx13 = -np.sin(x[2,0])*ux + np.cos(x[2,0])*uy 

    hx21 = -a*a*np.sin(x[2,0]) + a*b* np.cos(x[2,0])
    hx22 = -a*b*np.sin(x[2,0]) + b*b* np.cos(x[2,0])
    hx23 = -np.cos(x[2,0]) *ux - np.sin(x[2,0])*uy

    hx31 = c*a
    hx32 = b*c
    hx33 = 0

    Hx = np.asarray([[hx11, hx12, hx13],
                     [hx21, hx22, hx23],
                     [hx31, hx32, hx33]])
    return Hx


def get_Hxp_plane(x1, N1):
    N = copy.deepcopy(N1)
    x = copy.deepcopy(x1)

    N = from_any_to_feature(N)


    d = np.linalg.norm(N)
    a = N[0,0]/d
    b = N[1,0]/d
    c = N[2,0]/d

    dh1dnx =  (np.cos(x[2,0])*(- 2*x[0,0]*a**3 - 2*b*x[1,0]*a**2 + 2*x[0,0]*a + d + b*x[1,0]))/d - (b*np.sin(x[2,0])*(2*x[0,0]*a**2 + 2*b*x[1,0]*a - x[0,0]))/d
    dh1dny =  (np.sin(x[2,0])*(- 2*x[1,0]*b**3 - 2*a*x[0,0]*b**2 + 2*x[1,0]*b + d + a*x[0,0]))/d - (a*np.cos(x[2,0])*(2*x[1,0]*b**2 + 2*a*x[0,0]*b - x[1,0]))/d
    dh1dnz = -(2*c*(a*np.cos(x[2,0]) + b*np.sin(x[2,0]))*(a*x[0,0] + b*x[1,0]))/d

    dh2dnx = -(np.sin(x[2,0])*(- 2*x[0,0]*a**3 - 2*b*x[1,0]*a**2 + 2*x[0,0]*a + d + b*x[1,0]))/d - (b*np.cos(x[2,0])*(2*x[0,0]*a**2 + 2*b*x[1,0]*a - x[0,0]))/d
    dh2dny =  (np.cos(x[2,0])*(- 2*x[1,0]*b**3 - 2*a*x[0,0]*b**2 + 2*x[1,0]*b + d + a*x[0,0]))/d + (a*np.sin(x[2,0])*(2*x[1,0]*b**2 + 2*a*x[0,0]*b - x[1,0]))/d
    dh2dnz = -(2*c*(b*np.cos(x[2,0]) - a*np.sin(x[2,0]))*(a*x[0,0] + b*x[1,0]))/d

    dh3dnx = -(c*(2*x[0,0]*a**2 + 2*b*x[1,0]*a - x[0,0]))/d
    dh3dny = -(c*(2*x[1,0]*b**2 + 2*a*x[0,0]*b - x[1,0]))/d
    dh3dnz =  (d + a*x[0,0] + b*x[1,0] - 2*a*c**2*x[0,0] - 2*b*c**2*x[1,0])/d

    Hxp = np.asarray([[dh1dnx, dh1dny, dh1dnz],
                     [dh2dnx, dh2dny, dh2dnz],
                     [dh3dnx, dh3dny, dh3dnz]])
    return Hxp




def apply_g_plane(x1, Zp1):

    Zp= copy.deepcopy(Zp1)
    x = copy.deepcopy(x1)
    plane_4_param = Zp.shape[0] == 4

    Zp = from_any_to_feature(Zp)


    Zp = np.dot(get_rotation_matrix_bti([0, 0, x[2,0]]), Zp)

    eta = np.asarray([[x[0,0]], [x[1,0]], [0]])
    # print('eta: ', eta)
    # eta = np.dot(get_rotation_matrix_bti([0, 0, x[2,0]]), eta)
    # print('eta2: ', eta)

    corre = (np.dot(eta.T, Zp)/(np.linalg.norm(Zp)**2))
    u = Zp - corre*Zp

    if plane_4_param:
        d = np.linalg.norm(u)
        a = u[0,0]/d
        b = u[1,0]/d
        c = u[2,0]/d
        u = np.asarray([[a], [b], [c], [d]])
    return u

def get_Gx_plane(x1, Zp1):

    Zp= copy.deepcopy(Zp1)
    x = copy.deepcopy(x1)
    Zp = from_any_to_feature(Zp)


    d = np.linalg.norm(Zp)
    a = Zp[0,0]/d
    b = Zp[1,0]/d
    c = Zp[2,0]/d

    dg1dx = (d**2*(a*np.cos(x[2,0]) - b*np.sin(x[2,0]))**2)/(d**2*(a**2 + b**2 + c**2))**(1/2)
    dg1dy = (d**2*(b*np.cos(x[2,0]) + a*np.sin(x[2,0]))*(a*np.cos(x[2,0]) - b*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2)
    dg1dpsi = - a*d*np.sin(x[2,0]) - b*d*np.cos(x[2,0]) - (d**2*(b*np.cos(x[2,0]) + a*np.sin(x[2,0]))*(a*x[0,0]*np.cos(x[2,0]) + b*x[1,0]*np.cos(x[2,0]) + a*x[1,0]*np.sin(x[2,0]) - b*x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2) - (d**2*(a*np.cos(x[2,0]) - b*np.sin(x[2,0]))*(b*x[0,0]*np.cos(x[2,0]) - a*x[1,0]*np.cos(x[2,0]) + a*x[0,0]*np.sin(x[2,0]) + b*x[1,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2)

    dg2dx = (d**2*(b*np.cos(x[2,0]) + a*np.sin(x[2,0]))*(a*np.cos(x[2,0]) - b*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2)
    dg2dy = (d**2*(b*np.cos(x[2,0]) + a*np.sin(x[2,0]))**2)/(d**2*(a**2 + b**2 + c**2))**(1/2)
    dg2dpsi = a*d*np.cos(x[2,0]) - b*d*np.sin(x[2,0]) - (d**2*(b*np.cos(x[2,0]) + a*np.sin(x[2,0]))*(b*x[0,0]*np.cos(x[2,0]) - a*x[1,0]*np.cos(x[2,0]) + a*x[0,0]*np.sin(x[2,0]) + b*x[1,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2) + (d**2*(a*np.cos(x[2,0]) - b*np.sin(x[2,0]))*(a*x[0,0]*np.cos(x[2,0]) + b*x[1,0]*np.cos(x[2,0]) + a*x[1,0]*np.sin(x[2,0]) - b*x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2)

    dg3dx = (c*d**2*(a*np.cos(x[2,0]) - b*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2)
    dg3dy = (c*d**2*(b*np.cos(x[2,0]) + a*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2)
    dh3dpsi = -(c*d**2*(b*x[0,0]*np.cos(x[2,0]) - a*x[1,0]*np.cos(x[2,0]) + a*x[0,0]*np.sin(x[2,0]) + b*x[1,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2)

    Gxp = np.asarray([[dg1dx, dg1dy, dg1dpsi],
                     [dg2dx, dg2dy, dg2dpsi],
                     [dg3dx, dg3dy, dh3dpsi]])
    print(Zp.shape[0])
    return Gxp

def get_Gz_plane(x1, Zp1):
    Zp= copy.deepcopy(Zp1)
    x = copy.deepcopy(x1)

    Zp = from_any_to_feature(Zp)


    d = np.linalg.norm(Zp)
    a = Zp[0,0]/d
    b = Zp[1,0]/d
    c = Zp[2,0]/d

    dg1dnx = np.cos(x[2,0]) + (d*(a*np.cos(x[2,0]) - b*np.sin(x[2,0]))*(x[0,0]*np.cos(x[2,0]) + x[1,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2) + (d*np.cos(x[2,0])*(a*x[0,0]*np.cos(x[2,0]) + b*x[1,0]*np.cos(x[2,0]) + a*x[1,0]*np.sin(x[2,0]) - b*x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2) - (a*d**3*(a*np.cos(x[2,0]) - b*np.sin(x[2,0]))*(a*x[0,0]*np.cos(x[2,0]) + b*x[1,0]*np.cos(x[2,0]) + a*x[1,0]*np.sin(x[2,0]) - b*x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(3/2)
    dg1dny = (d*(a*np.cos(x[2,0]) - b*np.sin(x[2,0]))*(x[1,0]*np.cos(x[2,0]) - x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2) - np.sin(x[2,0]) - (d*np.sin(x[2,0])*(a*x[0,0]*np.cos(x[2,0]) + b*x[1,0]*np.cos(x[2,0]) + a*x[1,0]*np.sin(x[2,0]) - b*x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2) - (b*d**3*(a*np.cos(x[2,0]) - b*np.sin(x[2,0]))*(a*x[0,0]*np.cos(x[2,0]) + b*x[1,0]*np.cos(x[2,0]) + a*x[1,0]*np.sin(x[2,0]) - b*x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(3/2)
    dg1dnz = -(c*d**3*(a*np.cos(x[2,0]) - b*np.sin(x[2,0]))*(a*x[0,0]*np.cos(x[2,0]) + b*x[1,0]*np.cos(x[2,0]) + a*x[1,0]*np.sin(x[2,0]) - b*x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(3/2)
    
    dg2dnx = np.sin(x[2,0]) + (d*(b*np.cos(x[2,0]) + a*np.sin(x[2,0]))*(x[0,0]*np.cos(x[2,0]) + x[1,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2) + (d*np.sin(x[2,0])*(a*x[0,0]*np.cos(x[2,0]) + b*x[1,0]*np.cos(x[2,0]) + a*x[1,0]*np.sin(x[2,0]) - b*x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2) - (a*d**3*(b*np.cos(x[2,0]) + a*np.sin(x[2,0]))*(a*x[0,0]*np.cos(x[2,0]) + b*x[1,0]*np.cos(x[2,0]) + a*x[1,0]*np.sin(x[2,0]) - b*x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(3/2)
    dg2dny = np.cos(x[2,0]) + (d*(b*np.cos(x[2,0]) + a*np.sin(x[2,0]))*(x[1,0]*np.cos(x[2,0]) - x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2) + (d*np.cos(x[2,0])*(a*x[0,0]*np.cos(x[2,0]) + b*x[1,0]*np.cos(x[2,0]) + a*x[1,0]*np.sin(x[2,0]) - b*x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2) - (b*d**3*(b*np.cos(x[2,0]) + a*np.sin(x[2,0]))*(a*x[0,0]*np.cos(x[2,0]) + b*x[1,0]*np.cos(x[2,0]) + a*x[1,0]*np.sin(x[2,0]) - b*x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(3/2)
    dg2dnz = -(c*d**3*(b*np.cos(x[2,0]) + a*np.sin(x[2,0]))*(a*x[0,0]*np.cos(x[2,0]) + b*x[1,0]*np.cos(x[2,0]) + a*x[1,0]*np.sin(x[2,0]) - b*x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(3/2)

    dg3dnx = (c*d**3*(b**2*x[0,0]*np.cos(x[2,0]) + c**2*x[0,0]*np.cos(x[2,0]) + b**2*x[1,0]*np.sin(x[2,0]) + c**2*x[1,0]*np.sin(x[2,0]) - a*b*x[1,0]*np.cos(x[2,0]) + a*b*x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(3/2)
    dg3dny = -(c*d**3*(a**2*x[0,0]*np.sin(x[2,0]) - c**2*x[1,0]*np.cos(x[2,0]) - a**2*x[1,0]*np.cos(x[2,0]) + c**2*x[0,0]*np.sin(x[2,0]) + a*b*x[0,0]*np.cos(x[2,0]) + a*b*x[1,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(3/2)
    dg3dnz = (d*(a*x[0,0]*np.cos(x[2,0]) + b*x[1,0]*np.cos(x[2,0]) + a*x[1,0]*np.sin(x[2,0]) - b*x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(1/2) - (c**2*d**3*(a*x[0,0]*np.cos(x[2,0]) + b*x[1,0]*np.cos(x[2,0]) + a*x[1,0]*np.sin(x[2,0]) - b*x[0,0]*np.sin(x[2,0])))/(d**2*(a**2 + b**2 + c**2))**(3/2) + 1

    Gz = np.asarray([[dg1dnx, dg1dny, dg1dnz],
                     [dg2dnx, dg2dny, dg2dnz],
                     [dg3dnx, dg3dny, dg3dnz]])
    print(Zp.shape[0])
    return Gz

def get_W_plane():
    sigma_x = 0.1/3
    sigma_y = 0.1/3
    sigma_z = 0.1/3

    W = np.asarray([[sigma_x**2, 0, 0],
                    [0, sigma_y**2, 0],
                    [0, 0, sigma_z**2] ])
    return W

def get_Hw_plane():
    Hw = np.asarray([[1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]])

    return Hw





    # def upload_plane(self, Z, id, only_test=False):
    #     Z = Z.copy()

    #     Hxv = get_Hxv_plane(self.x_m, self.get_feature_from_id(id))
    #     Hxp = get_Hxp_plane(self.x_m, self.get_feature_from_id(id))
    #     Hx = get_Hx(Hxv, Hxp, id, self.P_m)
    #     Hw = get_Hw_plane()


    #     #invz = np.asarray([[1/Z[0]], [1/Z[1]], [1/Z[2]]])
    #     W = get_W_plane()#*modiff

    #     S = Hx @ self.P_m @ Hx.T + Hw @ W @ Hw.T

    #     if not np.linalg.cond(S) < 1/sys.float_info.epsilon:
    #         print("S é singular RETORNANDO VALOR SEM ATUALIZAÇÃO")
    #         print(S)
    #         return self.x_m[(3+id*3):(3+(id+1)*3)]

    #     K = self.P_m @ Hx.T @ np.linalg.inv(S)

    #     print("Feature nova: ", Z.T)
    #     print("Feature antiga: ", apply_h_plane(self.x_m, self.x_m[(3+id*3):(3+(id+1)*3)]).T)
    #     v = Z - apply_h_plane(self.x_m, self.x_m[(3+id*3):(3+(id+1)*3)])

    #     if only_test:
    #         x_m_test = self.x_m + K @ v
    #         P_m_test = self.P_m - K @ Hx @ self.P_m
    #         return x_m_test[(3+id*3):(3+(id+1)*3)]
    #     else:
    #         print("Atualizando plano: ", Z.T)
    #         init = self.x_m[0:3].copy()
    #         print("Posição inicial: ", init.T)
    #         self.x_m = self.x_m + K @ v
    #         print("Posição final: ", self.x_m[0:3].T)
    #         print("Movimento: ",(self.x_m[0:3] - init).T*100)
    #         self.P_m = self.P_m - K @ Hx @ self.P_m

    #         return self.x_m[(3+id*3):(3+(id+1)*3)]









    # def add_plane(self, Z):
    #     Z = Z.copy()

    #     i_plano = self.num_total_features['feature'] # pega id do próximo plano
    #     self.num_total_features['feature'] = self.num_total_features['feature']+1 # soma contador de planos
    #     self.type_feature_list.append(self.types_feat['plane'])
        

    #     Gx = get_Gx_plane(self.x_m, Z)
    #     Gz = get_Gz_plane(self.x_m, Z)
    #     Yz = get_Yz(Gx, Gz, self.P_m)
    #     # print('Yz ', Yz)

    #     N = apply_g_plane(self.x_m, Z)
    #     self.x_m = np.vstack((self.x_m, N))
    #     # print("New Feature: ", N)
    #     # print("New x: ", self.x_m)
    #     W = get_W_plane()*np.linalg.norm(Z)
    #     meio_bloco = np.block([[self.P_m,                                   np.zeros((self.P_m.shape[0], W.shape[1]))],
    #                            [np.zeros((W.shape[0], self.P_m.shape[1])),  W]])
    #     # print('meio_bloco: \n', meio_bloco)

    #     self.P_m = Yz @ meio_bloco @ Yz.T
    #     return i_plano
