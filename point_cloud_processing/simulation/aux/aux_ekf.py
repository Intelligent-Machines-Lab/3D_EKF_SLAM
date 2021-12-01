import numpy as np

import pickle
from operator import itemgetter
from scipy.spatial import distance
from aux.aux import *
import aux.cylinder
#import aux.plane
import sys
# from aux.cuboid import Cuboid
import settings
plane_feat_size = 3

if plane_feat_size == 3:
    from aux.aux_ekf_plane3 import *
else:
    from aux.aux_ekf_plane4 import *

class ekf:

    def __init__(self):
        self.init_angle = 0
        self.x_m, self.P_m  = init_x_P(self.init_angle)
        self.x_p, self.P_p  = init_x_P(self.init_angle)

        is_updated = False

        self.x_real = np.asarray([[0],[0],[self.init_angle]])
        self.x_errado = np.asarray([[0],[0],[self.init_angle]])

        self.x_p_list = []
        self.P_p_list = []

        self.x_m_list = []
        self.P_m_list = []

        self.x_real_list= []
        self.x_errado_list= []

        self.num_total_features = {'feature':0}

        self.types_feat = {'plane':1, 'point':2}
        self.type_feature_list = []


    def propagate(self, u):
        u = copy.deepcopy(u)
        x_m_last = self.x_m[:3,:]
        P_m_last = self.P_m[:3,:3]
        self.x_p = copy.deepcopy(self.x_m)
        self.P_p = copy.deepcopy(self.P_m)
        print('u ',u )
        #print("self.P_m: ", self.P_m)
        #print("x_m_last: \n",x_m_last)
        #print("P_m_last: \n",P_m_last)
        #print("u: \n",u)

        Fx = get_Fx(x_m_last, u)
        Fv = get_Fv(x_m_last, u)
        V  = get_V()


        new_x_p = apply_f(x_m_last, u) # Propagation

        #print("x_p: \n",x_p)
        new_P_p = Fx @ P_m_last @ Fx.T +  Fv @ V @ Fv.T
        #print("new_P_p: ", new_P_p)



        self.x_p[:3,:] = new_x_p

        self.P_p[:3,:3] = new_P_p
        #print("self.P_p: ", self.P_p)

        self.P_m = self.P_p
        self.x_m = self.x_p



    def add_plane(self, Z1):
        Z = copy.deepcopy(Z1)
        Z = from_any_to_feature(Z)
        i_plano = self.num_total_features['feature'] # pega id do próximo plano
        self.num_total_features['feature'] = self.num_total_features['feature']+1 # soma contador de planos
        self.type_feature_list.append(self.types_feat['plane'])
        
        print("Z: ", Z)
        Gx = get_Gx_plane(self.x_m, Z)
        Gz = get_Gz_plane(self.x_m, Z)
        Yz = get_Yz(Gx, Gz, self.P_m)
        # print('Yz ', Yz)

        N = apply_g_plane(self.x_m, Z)
        self.x_m = np.vstack((self.x_m, N))
        #print("New Feature: ", N)
        #print("New x: ", self.x_m)
        W = get_W_plane()
        meio_bloco = np.block([[self.P_m,                                   np.zeros((self.P_m.shape[0], W.shape[1]))],
                               [np.zeros((W.shape[0], self.P_m.shape[1])),  W]])
        # print('meio_bloco: \n', meio_bloco)

        self.P_m = Yz @ meio_bloco @ Yz.T
        return i_plano


    def add_point(self, C):
        i_plano = self.num_total_features['feature'] # pega id do próximo plano
        self.num_total_features['feature'] = self.num_total_features['feature']+1 # soma contador de planos
        self.type_feature_list.append(self.types_feat['point'])

        Gx = get_Gx_point(self.x_m, C)
        Gz = get_Gz_point(self.x_m, C)
        Yz = get_Yz(Gx, Gz, self.P_m)
        # print('Yz ', Yz)

        Z = apply_g_point(self.x_m, C)
        self.x_m = np.vstack((self.x_m, Z))
        # print("New Feature: ", N)
        # print("New x: ", self.x_m)
        W = get_W_point()
        meio_bloco = np.block([[self.P_m,                                   np.zeros((self.P_m.shape[0], W.shape[1]))],
                               [np.zeros((W.shape[0], self.P_m.shape[1])),  W]])
        # print('meio_bloco: \n', meio_bloco)

        self.P_m = Yz @ meio_bloco @ Yz.T
        return i_plano


    def upload_plane(self, Z1, id, only_test=False):
        Z = copy.deepcopy(Z1)
        Z = from_any_to_feature(Z)
        Z = normalize_plane_feature(Z)
        Z = copy.deepcopy(Z)
        # if np.argmax(np.abs(Z)) == 2:
        #     print("chão")
        #     # Z plane
        #     Z[0] = 0.0001
        #     Z[1] = 0.0001
        # else:
        #     # XY Plane
        #     Z[2] = 0.0001
        Hxv = get_Hxv_plane(self.x_m, self.x_m[(3+id*plane_feat_size):(3+(id+1)*plane_feat_size)])
        Hxp = get_Hxp_plane(self.x_m, self.x_m[(3+id*plane_feat_size):(3+(id+1)*plane_feat_size)])
        Hx = get_Hx(Hxv, Hxp, id, self.P_m)
        Hw = get_Hw_plane()


        #invz = np.asarray([[1/Z[0]], [1/Z[1]], [1/Z[2]]])
        W = get_W_plane()#*modiff

        S = Hx @ self.P_m @ Hx.T + Hw @ W @ Hw.T

        # if not np.linalg.cond(S) < 1/sys.float_info.epsilon:
        #     print("S é singular RETORNANDO VALOR SEM ATUALIZAÇÃO")
        #     print(S)
        #     return self.x_m[(3+id*3):(3+(id+1)*3)]

        K = self.P_m @ Hx.T @ np.linalg.inv(S)
        #print("Kalman gain: ", K)
        # K[0, 1] = 0 
        # K[0, 2] = 0 
        # #K[0, 3] = 0 

        # K[1, 0] = 0 
        # K[1, 2] = 0 
        # #K[1, 3] = 0 

        # K[2, 2] = 0 
        # K[2, 3] = 0 
        #print("Kalman gain 2: ", K)

        #print("Feature nova: ", Z.T)
        #print("Feature antiga: ", apply_h_plane(self.x_m, self.x_m[(3+id*4):(3+(id+1)*4)]).T)

        N = apply_h_plane(self.x_m, self.x_m[(3+id*plane_feat_size):(3+(id+1)*plane_feat_size)])
        N = normalize_plane_feature(N)

        # print('SDADWAWZ[3,0] ', Z[3,0])
        # print('AWDADWN[3,0] ', N[3,0])

        #print("Plano no mundo antigo (visto do robo): ", (N).T)
        #print("Plano medido: (visto do robo)", (Z).T)

        #print("Plano no mundo antigo (visto do mundo): ", (apply_g_plane(self.x_m,N)).T)
        #print("Plano medido: (visto do mundo)", (apply_g_plane(self.x_m,Z)).T)

        v =     Z - N
        #print('xm[2]: ',self.x_m[2])
        # v[0,0] = np.cos(self.x_m[2])*v[0,0] + np.sqrt(1-np.cos(self.x_m[2])**2)*v[1,0]
        # v[1,0] = np.sin(self.x_m[2])*v[1,0] + np.sqrt(1-np.sin(self.x_m[2])**2)*v[0,0]
        #v[3,0] = -v[3,0]
        # print("Plano v: ", (apply_g_plane(self.x_m,Z)-apply_g_plane(self.x_m,N)).T)
        # print("Plano v: ", (v).T)

        # # TESTING FOREWARD
        # x_m_test = copy.deepcopy(self.x_m + K @ v)
        # P_m_test = copy.deepcopy(self.P_m - K @ Hx @ self.P_m)
        # x_m_test[(3+id*4):(3+(id+1)*4)][0:4] = normalize_plane_feature(x_m_test[(3+id*4):(3+(id+1)*4)][0:4])
        # novod = x_m_test[(3+id*4):(3+(id+1)*4)][3,0]
        # print("novo d v = N - Z: ", x_m_test[(3+id*4):(3+(id+1)*4)][3,0])

        # # TESTING BACKWARD
        # v = Z - N 
        # x_m_test = copy.deepcopy(self.x_m + K @ v)
        # P_m_test = copy.deepcopy(self.P_m - K @ Hx @ self.P_m)
        # x_m_test[(3+id*4):(3+(id+1)*4)][0:4] = normalize_plane_feature(x_m_test[(3+id*4):(3+(id+1)*4)][0:4])
        # novod = x_m_test[(3+id*4):(3+(id+1)*4)][3,0]
        # print("novo d v = Z - N : ", x_m_test[(3+id*4):(3+(id+1)*4)][3,0])


        # Z_outro = apply_g_plane(self.x_m,Z)
        # N_outro = apply_g_plane(self.x_m,N)
        # print('novod ', novod)
        # print('Z[3,0] ', Z_outro[3,0])
        # print('N[3,0] ', N_outro[3,0])

        # print("(novod <= MEDIDO[3,0] and novod >= ANTIGO[3,0]): ", (novod <= Z_outro[3,0] and novod >= N_outro[3,0]))
        # print("(novod >= MEDIDO[3,0] and novod <= ANTIGO[3,0]): ", (novod >= Z_outro[3,0] and novod <= N_outro[3,0]))

        # if not ((novod <= Z_outro[3,0] and novod >= N_outro[3,0]) or (novod >= Z_outro[3,0] and novod <= N_outro[3,0])):
        #     v = Z - N 
        # else:
        #     v = N - Z

        # x_m_test = copy.deepcopy(self.x_m + K @ v)
        # P_m_test = copy.deepcopy(self.P_m - K @ Hx @ self.P_m)
        # x_m_test[(3+id*4):(3+(id+1)*4)][0:4] = normalize_plane_feature(x_m_test[(3+id*4):(3+(id+1)*4)][0:4])

        # print("d escolhido : ", x_m_test[(3+id*4):(3+(id+1)*4)][3,0])

        if only_test:
            x_m_test = copy.deepcopy(self.x_m + K @ v)
            P_m_test = copy.deepcopy(self.P_m - K @ Hx @ self.P_m)
            x_m_test[(3+id*plane_feat_size):(3+(id+1)*plane_feat_size)][0:plane_feat_size] = normalize_plane_feature(x_m_test[(3+id*plane_feat_size):(3+(id+1)*plane_feat_size)][0:plane_feat_size])
            return x_m_test[(3+id*plane_feat_size):(3+(id+1)*plane_feat_size)]
        else:
            is_updated = True
            #print("Atualizando plano: ", Z.T)
            init = copy.deepcopy(self.x_m[0:3])
            #print("Posição inicial: ", init.T)
            #print("v: ", v)
            #print("K @ v: ", K @ v)
            mov_kv = K @ v
            # mov_kv[0,0] = np.cos(self.x_m[2])*mov_kv[0,0] + np.sqrt(1-np.cos(self.x_m[2])**2)*mov_kv[1,0]
            # mov_kv[1,0] = np.sin(self.x_m[2])*mov_kv[1,0] + np.sqrt(1-np.sin(self.x_m[2])**2)*mov_kv[0,0]
            # mov_kv[1,0] = np.sin(self.x_m[2])*mov_kv[0,0] + np.sin(self.x_m[2])*mov_kv[1,0]
            #print('self.x_m: ', self.x_m  )
            #print('mov_kv: ', mov_kv  )
            self.x_m = self.x_m + mov_kv
            #print("Posição final: ", self.x_m[0:3].T)
            #print("Movimento: ",(self.x_m[0:3] - init).T)
            self.P_m = self.P_m - K @ Hx @ self.P_m

            self.x_m[(3+id*plane_feat_size):(3+(id+1)*plane_feat_size)][0:plane_feat_size] = normalize_plane_feature(self.x_m[(3+id*plane_feat_size):(3+(id+1)*plane_feat_size)][0:plane_feat_size])
            #print("Plano no final: ", self.x_m[(3+id*plane_feat_size):(3+(id+1)*plane_feat_size)][0:plane_feat_size].T)

            # Featurenova = self.x_m[(3+id*4):(3+(id+1)*4)]
            # Featurenova_vec = Featurenova[:3,0]
            # print('Featurenova: ', Featurenova)
            # print('Featurenova vec: ', Featurenova_vec)
            # print('Modulo vec: ', np.linalg.norm(Featurenova_vec))
            # self.x_m[(3+id*4):(3+(id+1)*4)][0,0] = self.x_m[(3+id*4):(3+(id+1)*4)][0,0]/np.linalg.norm(Featurenova_vec)
            # self.x_m[(3+id*4):(3+(id+1)*4)][1,0] = self.x_m[(3+id*4):(3+(id+1)*4)][1,0]/np.linalg.norm(Featurenova_vec)
            # self.x_m[(3+id*4):(3+(id+1)*4)][2,0] = self.x_m[(3+id*4):(3+(id+1)*4)][2,0]/np.linalg.norm(Featurenova_vec)
            # self.x_m[(3+id*4):(3+(id+1)*4)][3,0] = self.x_m[(3+id*4):(3+(id+1)*4)][3,0]/np.linalg.norm(Featurenova_vec)
            # Featurenova2 = self.x_m[(3+id*4):(3+(id+1)*4)]
            # Featurenova_vec2 = Featurenova2[:3,0]
            # print('Featurenova2: ', Featurenova2)
            # print('Featurenova vec2: ', Featurenova_vec2)
            # print('Modulo vec2: ', np.linalg.norm(Featurenova_vec2))
            return copy.deepcopy(self.x_m[(3+id*plane_feat_size):(3+(id+1)*plane_feat_size)])


    def upload_point(self, Z, id, only_test=False):
        Hxv = get_Hxv_point(self.x_m, self.get_feature_from_id(id))
        Hxp = get_Hxp_point(self.x_m, self.get_feature_from_id(id))
        Hx = get_Hx(Hxv, Hxp, id, self.P_m)
        Hw = get_Hw_point()
        W = get_W_point()

        S = Hx @ self.P_m @ Hx.T + Hw @ W @ Hw.T
        K = self.P_m @ Hx.T @ np.linalg.inv(S)

        v = Z - apply_h_point(self.x_m, self.x_m[(3+id*3):(3+(id+1)*3)])

        if only_test:
            x_m_test = self.x_m + K @ v
            P_m_test = self.P_m - K @ Hx @ self.P_m
            return x_m_test[(3+id*3):(3+(id+1)*3)]
        else:
            is_updated = True
            #print("O Cilindro vai modificar o vetor de estados assim:")
            #print("K @ v: ")
            #print(K @ v)
            self.x_m = self.x_m + K @ v
            self.P_m = self.P_m - K @ Hx @ self.P_m
            return self.x_m[(3+id*3):(3+(id+1)*3)]

    def calculate_mahalanobis(self, feature):
        feature = copy.deepcopy(feature)
        if isinstance(feature,aux.plane.Plane):
            eq = feature.equation
            N = from_equation_to_feature(eq)
            distances = []
            for id in range(self.num_total_features['feature']):
                if(self.type_feature_list[id] == self.types_feat['plane']):
                    Zp = apply_h_plane(self.x_m, self.get_feature_from_id(id))
                    Hxv = get_Hxv_plane(self.x_m, self.get_feature_from_id(id))
                    Hxp = get_Hxp_plane(self.x_m, self.get_feature_from_id(id))
                    Hx = get_Hx(Hxv, Hxp, id, self.P_m)
                    Hw = get_Hw_plane()
                    W = get_W_plane()

                    S = Hx @ self.P_m @ Hx.T + Hw @ W @ Hw.T
                    #print("ZP: ", Zp)
                    #print("N: ", N)
                    y = N - Zp
                    y = np.square(y)

                    if not np.linalg.cond(S) < 1/sys.float_info.epsilon:
                        print("S é singular Impedindo associação")
                        print(S)
                        return -1
                    d = y.T @ np.linalg.inv(S) @ y
                    d2 = distance.mahalanobis(N, Zp, np.linalg.inv(S))
                    #print("PLANO: Zp: ",Zp.T, " N: ",N.T, " d: ", d[0][0], " d2: ", d2)
                    distances.append(np.sqrt(abs(d[0][0])))
                else:
                    # If the feature is from another type, we put a very high distance
                    distances.append(99999)
            if distances:
                ordered_mdistance = min(enumerate(distances), key=itemgetter(1))
                idmin = ordered_mdistance[0] 


                max_mahadistance = settings.get_setting('mahala_plane')
                print('MAHALAPLANE: ', max_mahadistance)
                if(distances[idmin] > max_mahadistance):
                    idmin = -1
                else:
                    idmin_array = [i for i, el in enumerate(distances) if el < max_mahadistance]
                    print("COMO VOU ACHAR ISSO DAQUI??")
                    
                    
                    print(distances)
                    print(idmin_array)
                    d_range = [distances[index] for index in list(idmin_array)]
                    print(d_range)
                    print(ordered_mdistance)

                    print('SORTANDO')
                    d_range, idmin_array = zip(*sorted(zip(d_range, idmin_array)))
                    print(idmin_array)
                    print(d_range)
                    idmin = list(idmin_array)

            else:
                idmin = -1
        elif(isinstance(feature,aux.cylinder.Cylinder)):
            centroid = feature.center
            C = np.asarray([[centroid[0]],[centroid[1]],[centroid[2]]])
            distances = []
            for id in range(self.num_total_features['feature']):
                if(self.type_feature_list[id] == self.types_feat['point']):
                    Zp = apply_h_point(self.x_m, self.get_feature_from_id(id))
                    Hxv = get_Hxv_point(self.x_m, self.get_feature_from_id(id))
                    Hxp = get_Hxp_point(self.x_m, self.get_feature_from_id(id))
                    Hx = get_Hx(Hxv, Hxp, id, self.P_m)
                    Hw = get_Hw_point()
                    W = get_W_point()

                    S = Hx @ self.P_m @ Hx.T + Hw @ W @ Hw.T
                    y = C - Zp
                    y = np.square(y)
                    d = y.T @ np.linalg.inv(S) @ y
                    d2 = distance.mahalanobis(C, Zp, np.linalg.inv(S))
                    #print("CILINDRO: Zp: ",Zp.T, " N: ",C.T, " d: ", d[0][0], " d2: ", d2)
                    distances.append(np.sqrt(abs(d[0][0])))
                else:
                    # If the feature is from another type, we put a very high distance
                    distances.append(99999)
            if distances:
                idmin = min(enumerate(distances), key=itemgetter(1))[0]
                # antes tava 16 
                # antes tava 10
                if(distances[idmin] > settings.get_setting('mahala_point')):
                    idmin = -1
            else:
                idmin = -1


        # if(not id == -1):
        #     feature.move(self)
        #     gfeature = Generic_feature(feature, ground_equation=self.ground_equation)
        #     older_feature = self.get_feature_from_id(id)
        #     d_maior = np.amax([older_feature.feat.width,older_feature.feat.height, gfeature.feat.width,gfeature.feat.height])
        #     if(np.linalg.norm((older_feature.feat.centroid - gfeature.feat.centroid)) < d_maior):
        #         area1 = older_feature.feat.width*older_feature.feat.height
        #         area2 = gfeature.feat.width*gfeature.feat.height
        #         if (not (area1/area2 < 0.05 or area1/area2 > 20)) or id == 0:
        #             older_feature.correspond(gfeature, self.ekf)
        #         else:
        #             id = -1
        #     else:
        #         id = -1
        #     # else:
        #     #     id = -1

        if type(idmin) != list:
            idmin = [idmin]
        print("Associação: ",distances, " id menor: ",idmin)
        
        return idmin


    def get_feature_from_id(self, id):

        return copy.deepcopy(self.x_m[(3+id*plane_feat_size):(3+(id+1)*plane_feat_size)])

    def delete_feature(self, id):
        # deleta linhas da matriz de estados
        self.x_m = np.delete(self.x_m,(np.s_[(3+id*3):(3+(id+1)*3)]), axis=0)

        # deleta linhas da matriz de covariância
        self.P_m = np.delete(self.P_m,(np.s_[(3+id*3):(3+(id+1)*3)]), axis=0)

        # deleta colunas da matriz de covariância
        self.P_m = np.delete(self.P_m,(np.s_[(3+id*3):(3+(id+1)*3)]), axis=1)

        self.num_total_features['plane'] = self.num_total_features['plane']-1

    def update_real_odom_states(self, u_real, u):
        self.x_real = apply_f(self.x_real, u_real)
        self.x_errado = apply_f(self.x_errado, u)

    def save_file(self):

        self.x_p_list.append(self.x_p.copy())
        self.P_p_list.append(self.P_p.copy())

        self.x_m_list.append(self.x_m.copy())
        self.P_m_list.append(self.P_m.copy())



        self.x_real_list.append(self.x_real.copy())
        self.x_errado_list.append(self.x_errado.copy())
        # print("f: ",self.x_p)
        # print("P: ",self.P_p)
        nsalva = {}
        nsalva['x_p_list'] =  self.x_p_list
        nsalva['P_p_list'] =  self.P_p_list

        nsalva['x_m_list'] =  self.x_m_list
        nsalva['P_m_list'] =  self.P_m_list

        nsalva['x_real_list'] =  self.x_real_list
        nsalva['x_errado_list'] =  self.x_errado_list

        f = open('map_outputs/ekf.pckl', 'wb')
        pickle.dump(nsalva, f)
        f.close()





def get_Hx(Hxv, Hxp, id, P_m):
    #print('Hxv:\n',Hxv)
    #print('Hxp:\n',Hxp)
    antes_p = np.zeros((Hxv.shape[0],id*Hxp.shape[1]))
    depois_p = np.zeros((Hxv.shape[0],(P_m.shape[1]-(Hxv.shape[1]+id*Hxp.shape[1]+Hxp.shape[1]) ) ))
    #print('antes_p:\n',antes_p)
    #print('depois_p:\n',depois_p)
    Hx = np.hstack((Hxv,antes_p,Hxp,depois_p))
    #print('Hx: \n',Hx)
    return Hx

def get_Yz(Gx, Gz, P_m):
    n = P_m.shape[0]
    In = np.eye(n)
    # print("Gx.shape[0]", Gx.shape[1])
    # print("Gx.shape[1]", Gx.shape[0])
    zero_low = np.zeros((Gx.shape[0],n-Gx.shape[1]))
    zero_up = np.zeros((n,Gz.shape[1]))
    # print('zero_low',zero_low)
    # print('zero_up',zero_up)

    low = np.hstack((Gx,zero_low))
    low = np.hstack((low,Gz))
    up = np.hstack((In,zero_up))

    Yz = np.vstack((up,low))

    # print('Yz ', Yz)
    # print('low', low)
    # print('up', up)
    # print('n', n)
    return Yz

def init_x_P(init_angle = 0):

    P = np.eye(3, dtype=float)
    P[0, 0] = 0#get_V()[0,0]
    P[1, 1] = 0#get_V()[0,0]
    P[2, 2] = 0#0get_V()[1,1]

    x = np.zeros((3, 1))
    x[2,0] = init_angle
    return x, P

def get_V():
    sigma_x = settings.get_setting('V_x')/3
    sigma_psi = ((settings.get_setting('V_psi')/3)*np.pi/180)

    V = np.asarray([[sigma_x**2, 0],
                    [0, sigma_psi**2] ])

    return V

def apply_f(x, u):
    f11 = x[0,0] + np.cos(x[2,0])*u[0,0]
    f21 = x[1, 0] + np.sin(x[2,0])*u[0,0]
    f31 = get_sum_angles(x[2, 0], u[1,0])

    new_x = np.asarray([[f11],[f21],[f31]])
    return new_x

def get_Fx(x, u):
    Fx = np.asarray([[1, 0, -np.sin(x[2,0])*u[0,0]],
                     [0, 1, np.cos(x[2,0])*u[0,0]],
                     [0, 0, 1]])
    return Fx

def get_Fv(x, u):
    Fv = np.asarray([[np.cos(x[2,0]), 0],
                     [np.sin(x[2,0]), 0],
                     [0, 1]])
    return Fv







#############################################################################################
# POINT FEATURE - CYLINDER
#############################################################################################



def apply_h_point(x, Z):

    zpx = np.cos(x[2,0])*(Z[0,0] - x[0,0]) + np.sin(x[2,0])*(Z[1,0] - x[1,0])
    zpy = -np.sin(x[2,0])*(Z[0,0] - x[0,0]) + np.cos(x[2,0])*(Z[1,0] - x[1,0])
    zpz = Z[2,0]

    zp = np.asarray( [[zpx],
                      [zpy],
                      [zpz]])
    return zp

def apply_g_point(x, C):
    zx = x[0,0] + C[0,0]*np.cos(x[2,0]) - C[1,0]*np.sin(x[2,0])
    zy = x[1,0] + C[1,0]*np.cos(x[2,0]) + C[0,0]*np.sin(x[2,0])
    zz = C[2,0]

    Z = np.asarray( [[zx],
                     [zy],
                     [zz]])
    return Z


def get_Hxv_point(x, Z):

    hx11 = -np.cos(x[2,0])
    hx12 = -np.sin(x[2,0])
    hx13 = np.sin(x[2,0])*(x[0,0] - Z[0,0]) - np.cos(x[2,0])*(x[1,0] - Z[1,0])

    hx21 = np.sin(x[2,0])
    hx22 = -np.cos(x[2,0])
    hx23 = np.cos(x[2,0])*(x[0,0] - Z[0,0]) + np.sin(x[2,0])*(x[1,0] - Z[1,0])

    hx31 = 0
    hx32 = 0
    hx33 = 0

    Hx = np.asarray([[hx11, hx12, hx13],
                     [hx21, hx22, hx23],
                     [hx31, hx32, hx33]])
    return Hx


def get_Hxp_point(x, N):

    dh1dnx =  np.cos(x[2,0])
    dh1dny =  np.sin(x[2,0])
    dh1dnz = 0

    dh2dnx = -np.sin(x[2,0])
    dh2dny = np.cos(x[2,0])
    dh2dnz = 0

    dh3dnx = 0
    dh3dny = 0
    dh3dnz = 1

    Hxp = np.asarray([[dh1dnx, dh1dny, dh1dnz],
                     [dh2dnx, dh2dny, dh2dnz],
                     [dh3dnx, dh3dny, dh3dnz]])
    return Hxp


def get_Gx_point(x, C):

    dg1dx = 1
    dg1dy = 0
    dg1dpsi = - C[1,0]*np.cos(x[2,0]) - C[0,0]*np.sin(x[2,0])

    dg2dx = 0
    dg2dy = 1
    dg2dpsi = C[0,0]*np.cos(x[2,0]) - C[1,0]*np.sin(x[2,0])

    dg3dx = 0
    dg3dy = 0
    dh3dpsi = 0

    Gxp = np.asarray([[dg1dx, dg1dy, dg1dpsi],
                     [dg2dx, dg2dy, dg2dpsi],
                     [dg3dx, dg3dy, dh3dpsi]])
    return Gxp

def get_Gz_point(x, C):


    dg1dnx = np.cos(x[2,0])
    dg1dny = -np.sin(x[2,0])
    dg1dnz = 0
    
    dg2dnx = np.sin(x[2,0])
    dg2dny = np.cos(x[2,0])
    dg2dnz = 0

    dg3dnx = 0
    dg3dny = 0
    dg3dnz = 1

    Gz = np.asarray([[dg1dnx, dg1dny, dg1dnz],
                     [dg2dnx, dg2dny, dg2dnz],
                     [dg3dnx, dg3dny, dg3dnz]])
    return Gz

def get_W_point():
    sigma_x = 0.1/3
    sigma_y = 0.1/3
    sigma_z = 0.1/3

    W = np.asarray([[sigma_x**2, 0, 0],
                    [0, sigma_y**2, 0],
                    [0, 0, sigma_z**2] ])
    return W

def get_Hw_point():
    Hw = np.asarray([[1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]])

    return Hw