import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import leastsq
import cv2
import utils
from  queue import Queue
import scipy as sp
#
# K = np.array([[ 2.1875    ,  0.        ,  0.        ],  # intrinsic matrix
#               [ 0.        ,  2.1875    ,  0.        ],
#               [ 0.        ,  0.        ,  1.        ]])

K = np.array([[ 560    ,  0.        ,  0.        ],  # intrinsic matrix
              [ 0.        ,  560    ,  0.        ],
              [ 0.        ,  0.        ,  1.        ]])
K_inv = np.array([[ 1/560    ,  0.        ,  0.        ],  # intrinsic matrix
              [ 0.        ,  1/560    ,  0.        ],
              [ 0.        ,  0.        ,  1.        ]])

RTP = np.array([[ 0   ,  1     ,  0.        ],  # intrinsic matrix
              [ -1        ,  0 ,  0.        ],
              [ 0.        ,  0.        ,1  ]])
RTN = np.array([[ 0   ,  -1     ,  0.        ],  # intrinsic matrix
              [ 1        ,  0 ,  0.        ],
              [ 0.        ,  0.        ,1  ]])

# data loading
folder = "./eight_point/"
img_arr = utils.load_scenes(folder)
# utils.plot_scenes(img_arr)

# read correspondences
corr_8 = np.load(os.path.join(folder, 'eight_pt_corr.npy'))
corr_8.shape

print("Using the image 2")
scene_id =2 # change this to view other scenes
utils.plot_correspondences(img_arr, corr_8, scene_id)


def eight_2(corr,im):
    pass

def hat2vec(T_hat):
    t = np.array([ -  T_hat[1][2],  T_hat[0][2], -T_hat[0][1] ])
    return t

def vec2hat(t):
    T_hat = np.array([[0, -t[2], t[1]],
                      [t[2], 0, -t[0]],
                      [-t[1], t[0], 0]])
    return T_hat


def findRT(U,ss,Vh,Rz):
    T_hat = ((U.dot(Rz)).dot(ss)).dot(U.T)
    T = hat2vec(T_hat)
    R = (U.dot(Rz)).dot(Vh)
    return R,T

def eight_point_algorithm(corr,im):
    # Read the corresponding 8 points
    print("------------------------ 8 Point ----------------------------")
    y1 = corr[im][0, :, 0]
    x1 = corr[im][0, :, 1]
    y2 = corr[im][1, :, 0]
    x2 = corr[im][1, :, 1]
    chi = np.mat(np.zeros((9,1)))
    for i in range(0,len(x1)):
        p_1 = np.array([x1[i], y1[i], 1]).T
        p_2 = np.array([x2[i], y2[i], 1]).T
        x_1 = K_inv.dot(p_1)
        x_2 = K_inv.dot(p_2)
        a_ = np.mat(np.kron(x_1,x_2))
        chi = np.hstack((chi,a_.T))
    chi =chi[:,1:].T

    evalue,evector = np.linalg.eig(chi.T.dot(chi))
    minVec = evector[:][8]
    b = np.zeros((9,1))
    # f = np.linalg.solve(a, b)
    f =np.array(minVec[ 0,:])
    f = f[0,:]
    F = np.mat([[f[0], f[1], f[2]],
                  [f[3], f[4], f[5]],
                  [f[6], f[7], f[8]]
                  ])
    E = (K.T).dot(F).dot(K)
    E = np.array(E, dtype='float')
    # print("The essential matrix is:\n",E)
    negE = -E
    # Find good R,T
    p_1 = np.array([x1[0], y1[0], 1]).T
    p_2 = np.array([x2[0], y2[0], 1]).T
    x_1 = K_inv.dot(p_1)
    x_2 = K_inv.dot(p_2)
    Rzs = [RTN,RTP]
    Es  = [E, negE]
    index = 0
    for Rz in Rzs:
        for E in Es:
            U, S, Vh = np.linalg.svd(E)
            ss = np.diag([1, 1, 0])
            R_, T_ = findRT(U, ss, Vh, Rz)
            if (vec2hat(T_) @  x_2 @ vec2hat(T_) @ R_ @ x_1 > 0
                and vec2hat(x_2) @ R_ @ x_1 @ vec2hat(x_2) @ T_ <= 0
                and vec2hat(R_ @ x_1) @ x_2 @ vec2hat(R_ @ x_1) @ T_ > 0):
                R = R_
                T = T_
    return R,T




def opencv8Points(corr, im):
    print("------------------------OpenCV--------------------------")
    y0 = corr[im][0, :, 0]
    x0 = corr[im][0, :, 1]
    y1 = corr[im][1, :, 0]
    x1 = corr[im][1, :, 1]
    points1 = np.mat(np.array([x0,y0])).T
    points2 = np.mat(np.array([x1, y1])).T
    E = cv2.findEssentialMat(points1=points1,points2=points2,method=cv2.RANSAC,cameraMatrix=K)
    print("The essential matrix:\n",E[0])
    U,S,Vh = np.linalg.svd(E[0])
    print("The singular value\n", S)
    R = np.zeros((3,3))
    T = np.zeros((3,1))
    # E = np.asarray(E)
    Ee= np.array(E[0], dtype='float')
    cv2.recoverPose(E=Ee,points1=points1,points2=points2,cameraMatrix=K, R=R,t=T)
    print("Rotataion:\n",R)
    print("Translation:\n",T)
    # 检验变换
    p_1 = np.array([x0[0], y0[0], 1]).T
    p_2 = np.array([x1[0], y1[0], 1]).T
    P_inspace = np.asmatrix(K_inv.dot(p_1)).T
    print(T.shape,R.shape,P_inspace.shape)
    s_2p2 = K.dot(R.dot(P_inspace) + T)
    s_2p2 = s_2p2 / s_2p2[0] * p_2[0]
    print("s2p2", s_2p2.T)
    print("p2", p_2)



# opencv8Points(corr_8,1)
R,T = eight_point_algorithm(corr_8,scene_id)
print("The rotation is:\n", R)
print("The translation is:\n", T)
#
# # ## Four-point Algorithm
#
# # data loading
folder = "./four_point/"
img_arr = utils.load_scenes(folder)
utils.plot_scenes(img_arr)

# read correspondences
corr_4 = np.load(os.path.join(folder,'four_pt_corr.npy'))

# plotting correspondences
# scene_id = 2
utils.plot_correspondences(img_arr, corr_4, scene_id)


def four_point_algorithm(corr, im):
    print("------------------------ 4 Point ----------------------------")
    y0 = corr[im][0, :, 0]
    x0 = corr[im][0, :, 1]
    y1 = corr[im][1, :, 0]
    x1 = corr[im][1, :, 1]

    a = np.mat(np.array(
        [[x0[0], y0[0], 1, 0, 0, 0, -x0[0] * x1[0], -y0[0] * x1[0]],
         [0, 0, 0, x0[0], y0[0], 1, -x0[0] * y1[0], -y0[0] * y1[0]],
         [x0[1], y0[1], 1, 0, 0, 0, -x0[1] * y1[1], -y0[1] * x1[1]],
         [0, 0, 0, x0[1], y0[1], 1, -x0[1] * y1[1], -y0[1] * y1[1]],
         [x0[2], y0[2], 1, 0, 0, 0, -x0[2] * x1[2], -y0[2] * x1[2]],
         [0, 0, 0, x0[2], y0[2], 1, -x0[2] * y1[2], -y0[2] * y1[2]],
         [x0[3], y0[3], 1, 0, 0, 0, -x0[3] * x1[3], -y0[3] * x1[3]],
         [0, 0, 0, x0[3], y0[3], 1, -x0[3] * y1[3], -y0[3] * y1[3]]
         ])
    )
    b = np.mat(np.array([x1[0], y1[0], x1[1], y1[1], x1[2], y1[2], x1[3], y1[3]])).T
    h = np.linalg.solve(a, b)
    H = np.mat(np.array([[h[0], h[1], h[2]],
                [h[3], h[4], h[5]],
                [h[6], h[7], 0]
                ]))


    HTH = (H.T).dot(H)
    HTH = np.array(HTH, dtype='float')
    V, S, VT = np.linalg.svd(HTH)
    v_1 =np.mat(VT[0][:]).T
    v_2 =np.mat(VT[1][:]).T
    v_3 =np.mat(VT[2][:]).T

    u_1 = (np.sqrt(1 - S[2] ** 2) * v_1 + np.sqrt(S[0] ** 2 - 1) * v_3) / np.sqrt(S[0] ** 2 - S[2] ** 2)
    u_2 = (np.sqrt(1 - S[2] ** 2) * v_1 - np.sqrt(S[0] ** 2 - 1) * v_3) / np.sqrt(S[0] ** 2 - S[2] ** 2)

    W1 = np.hstack((H.dot(v_2), H.dot(u_1), np.cross(H.dot(v_2).T, H.dot(u_1).T).T ))
    W2 = np.hstack((H.dot(v_2), H.dot(u_2), np.cross(H.dot(v_2).T, H.dot(u_2).T).T))
    U1 = np.hstack((v_2,u_1,np.cross(v_2.T,u_1.T).T))
    U2 = np.hstack((v_2, u_2, np.cross(v_2.T, u_2.T).T))

    e3 = np.mat(np.array([0, 0, 1])).T
    R1 = W1.dot(U1)
    N1 = np.cross(v_2.T,u_1.T).T
    T1 = (H - R1) .dot(N1)
    cnt=0
    if (N1.T.dot(e3) > 0):
        R = R1
        T = T1
        cnt+=1
    R2 = W2.dot(U2)
    N2 = np.cross(v_2.T, u_2.T).T
    T2 = (H - R2).dot(N2)
    if (N2.T.dot(e3) > 0):
        R = R2
        T = T2
        cnt += 1
    R3 = R1
    N3 = -N1
    T3 = -T1
    if (N3.T.dot(e3) > 0):
        R = R3
        T = T3
        cnt += 1
    R4 = R2
    N4 = -N2
    T4 = -T2
    if (N4.T.dot(e3) > 0):
        R = R4
        T = T4
        cnt += 1
    return R, T


R,T = four_point_algorithm(corr_4,scene_id)
R  = np.array(R)
T = np.array(T)
print("Rotation matrix R :\n", R.astype(float))
print("Translation vector T:\n", T.astype(float))

# End of Assignment
