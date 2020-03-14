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


scene_id =0 # change this to view other scenes
print("Using the image :",scene_id)
# utils.plot_correspondences(img_arr, corr_8, scene_id)


def eight_2(corr,im):
    pass

def hat2vec(T_hat):

    t = np.array([ -T_hat[1,2],  T_hat[0,2], -T_hat[0,1] ])
    return t

def vec2hat(t):
    if(t.shape[0] == 3):
        t=t.T
    t.reshape(3,1)
    t = np.mat(t)

    T_hat = np.array([[0, -t[0,2], t[0,1]],
                      [t[0,2], 0, -t[0,0]],
                      [-t[0,1], t[0,0], 0]])
    return T_hat



def show_projection(img_arr, scene_id, corr, R, T):
    x1 = corr[0]
    x2 = corr[1]
    x1 = x1 / 560
    x2 = x2 / 560
    x1[:, [0, 1]] = x1[:, [1, 0]]
    x2[:, [0, 1]] = x2[:, [1, 0]]
    X1 = np.append(x1[0], 1)
    X2 = np.append(x2[0], 1)
    l = x1.shape[0]
    X_projection = np.zeros([l, 2])

    for i in range(l):
        X1 = np.append(x1[i], 1)
        X2 = np.append(x2[i], 1)
        lambda1 = np.linalg.norm(vec2hat(X2) @ T) / np.linalg.norm(vec2hat(X2) @ R @ X1)
        lambda2 = np.linalg.norm((vec2hat(R @ X1) @ T)) / np.linalg.norm((vec2hat(R @ X1) @ X2))
        #         print(lambda1,lambda2)
        projection = (lambda1 * R.dot(X1).transpose() + T[:, 0])
        projection_xy = projection[:2] / lambda2
        X_projection[i,0] = projection_xy[0]
        X_projection[i,1] = projection_xy[1]

    im_i = scene_id  # image index 0-9 (please change this to see other scenes)
    fig, ax = plt.subplots(ncols=1, nrows=1, figsize=(7, 5), dpi=200)
    ax.imshow(img_arr[im_i, 1])
    ax.scatter(x2[:, 0] * 560, x2[:, 1] * 560, c='r', s=5)
    ax.scatter(X_projection[:, 0] * 560, X_projection[:, 1] * 560, c='b', s=5)
    ax.set_title('projection image')
    plt.show()


def eight_point_algorithm(corr):
    y1 = corr[0, :, 0]
    x1 = corr[0, :, 1]
    y2 = corr[1, :, 0]
    x2 = corr[1, :, 1]
    chi = np.mat(np.zeros((9, 1)))
    for i in range(0, len(x1)):
        p_1 = np.array([x1[i], y1[i], 1]).T
        p_2 = np.array([x2[i], y2[i], 1]).T
        x_1_ = K_inv.dot(p_1)
        x_2_ = K_inv.dot(p_2)
        a_ = np.mat(np.kron(x_1_, x_2_))
        chi = np.hstack((chi, a_.T))
    chi = chi[:, 1:].T
    evalue, evector = np.linalg.eig(chi.T@chi)
    minVec = evector[:,np.argmin( evalue)]
    E0 = (minVec.reshape(3, 3)).T
    Rzs = [RTN, RTP]
    p_1 = np.array([x1[0], y1[0], 1]).T
    p_2 = np.array([x2[0], y2[0], 1]).T
    x_1 = K_inv.dot(p_1)
    x_2 = K_inv.dot(p_2)
    U, S, Vh = np.linalg.svd(-E0)
    ss = np.diag([1, 1, 0])
    for Rz in Rzs:
        for RzT in Rzs:
            R_ = U @ Rz.T @ Vh
            T_hat = U @ RzT @ ss @ U.T
            T_ = hat2vec(T_hat)
            if (vec2hat(T_) @ x_2 @ vec2hat(T_) @ R_ @ x_1 > 0
            and vec2hat(x_2) @ R_ @ x_1 @ vec2hat(x_2) @ T_ <= 0
            and vec2hat(R_ @ x_1) @ x_2 @ vec2hat(R_ @ x_1) @ T_ > 0):
                R = np.mat(R_)
                T = np.mat(T_).T
    return R, T




def opencv8Points(corr, im):
    # print("------------------------OpenCV--------------------------")
    y0 = corr[im][0, :, 0]
    x0 = corr[im][0, :, 1]
    y1 = corr[im][1, :, 0]
    x1 = corr[im][1, :, 1]
    points1 = np.mat(np.array([x0,y0])).T
    points2 = np.mat(np.array([x1, y1])).T
    E = cv2.findEssentialMat(points1=points1,points2=points2,method=cv2.RANSAC,cameraMatrix=K)
    U,S,Vh = np.linalg.svd(E[0])
    R = np.zeros((3,3))
    T = np.zeros((3,1))
    # E = np.asarray(E)
    Ee= np.array(E[0], dtype='float')
    cv2.recoverPose(E=Ee,points1=points1,points2=points2,cameraMatrix=K, R=R,t=T)

    return R,T



#
# for i in range(0,4):
#     scene_id = i
#     R,T = eight_point_algorithm(corr_8[scene_id])
#     R,T = opencv8Points(corr_8,scene_id)
#     print("--------------scene_id = ",i,"----------------")
#     print("The rotation is:\n", R)
#     print("The translation is:\n", T)
#     show_projection(img_arr,scene_id,corr_8[scene_id],R,T)


# ## Four-point Algorithm

# data loading
folder = "./four_point/"
img_arr = utils.load_scenes(folder)
# utils.plot_scenes(img_arr)

# read correspondences
corr_4 = np.load(os.path.join(folder,'four_pt_corr.npy'))

# plotting correspondences
# scene_id = 2
# utils.plot_correspondences(img_arr, corr_4, scene_id)


def four_point_algorithm(corr, im):

    y0 = corr[im][0, :, 0]
    x0 = corr[im][0, :, 1]
    y1 = corr[im][1, :, 0]
    x1 = corr[im][1, :, 1]
    chi = np.zeros((9, 1))

    chi = np.zeros((9, 1))
    for i in range(0, len(x1)):
        p_1 = np.array([x0[i], y0[i], 1]).T
        p_2 = np.array([x1[i], y1[i], 1]).T
        x_1 = K_inv.dot(p_1)
        x_2 = K_inv.dot(p_2)
        x_2_hat = vec2hat(x_2)
        a_ = np.mat(np.kron(x_1, x_2_hat))
        chi = np.hstack((chi, a_.T))
    chi = chi[:, 1:].T
    evalue, evector = np.linalg.eig(chi.T @ chi)
    minVec = evector[:, np.argmin(evalue)]
    f = (minVec.reshape(3, 3)).T
    HL = f.reshape(3,3)
    U_, S_, V_ = np.linalg.svd(HL)
    H = HL / S_[1]
    p_1 = np.array([x0[i], y0[i], 1]).T
    p_2 = np.array([x1[i], y1[i], 1]).T
    x_1 = K_inv.dot(p_1)
    x_2 = K_inv.dot(p_2)
    if ((x_2.T @ H @ x_1)<0):
        H = -H
    V, S, VT = np.linalg.svd(H)
    v_1 = np.mat(VT[0][:]).T
    v_2 = np.mat(VT[1][:]).T
    v_3 = np.mat(VT[2][:]).T
    u_1 = (np.sqrt(1 - S[2] ** 2) * v_1 + np.sqrt(S[0] ** 2 - 1) * v_3) / np.sqrt(S[0] ** 2 - S[2] ** 2)
    u_2 = (np.sqrt(1 - S[2] ** 2) * v_1 - np.sqrt(S[0] ** 2 - 1) * v_3) / np.sqrt(S[0] ** 2 - S[2] ** 2)
    W1 = np.hstack((H.dot(v_2), H.dot(u_1), vec2hat(H.dot(v_2).T)@H.dot(u_1)))
    W2 = np.hstack((H.dot(v_2), H.dot(u_2), vec2hat(H.dot(v_2).T)@H.dot(u_2)))
    U1 = np.hstack((v_2, u_1, np.cross(v_2.T, u_1.T).T))
    U2 = np.hstack((v_2, u_2, np.cross(v_2.T, u_2.T).T))
    # CHECK RIGHT RT

    R=[]
    T=[]
    N=[]
    e3 = np.mat(np.array([0, 0, 1])).T
    R1 = W1.dot(U1.T)
    N1 = vec2hat(v_2.T).dot(u_1)
    T1 = (H - R1).dot(N1)
    cnt = 0
    if (N1.T@e3 > 0):
        R.append(R1)
        T.append(T1)
        N.append(N1)
        cnt += 1
    R2 = W2.dot(U2.T)
    N2 = vec2hat(v_2).dot(u_2)
    T2 = (H - R2).dot(N2)
    if (N2.T.dot(e3) > 0):
        R.append(R2)
        T.append(T2)
        N.append(N2)
    R3 = R1
    N3 = -N1
    T3 = -T1
    if (N3.T.dot(e3) > 0):
        R.append(R3)
        T.append(T3)
        N.append(N3)
    R4 = R2
    N4 = -N2
    T4 = -T2
    if (N4.T.dot(e3) > 0):
        R.append(R4)
        T.append(T4)
        N.append(N4)
    p_1 = np.array([x0[0], y0[0], 1]).T
    p_2 = np.array([x1[0], y1[0], 1]).T
    x_1 = K_inv.dot(p_1)
    x_2 = K_inv.dot(p_2)

    return (R[0], N[0], T[0]), (R[1], N[1], T[1])


print("------------------------ 4 Point ----------------------------")
for i in range(0,4):
    scene_id =i
    ((R1, N1, T1), (R2, N2, T2)) = four_point_algorithm(corr_4, scene_id)
    show_projection(img_arr, scene_id, corr_4[scene_id], R1, T1)
    print("--------------scene_id = ",i,"----------------")
    print("The possible rotation 1 is:\n", R1)
    print("The possible translation 1 is:\n", T1)
    print("The possible rotation 2 is:\n", R2)
    print("The possible translation 2 is:\n", T2)


# End of Assignment
