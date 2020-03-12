#!/usr/bin/env python
# coding: utf-8

# # Homework 2 - Programming

# There are two main tasks in this programming assignment are to implement the **eight-point algorithm** and **four-point algorithm**. 

# In[2]:


import os
import numpy as np
import matplotlib.pyplot as plt
import cv2
import utils
from  queue import Queue
import scipy as sp


# ## Remarks before starting
# Please take note of the following few things:
# 
# 1. In your `hw2_progrmming` folder, there are two subfolders, namely `four_point` and `eight_point`. In each folder, there are four pairs of scenes (8 images in total) and a `numpy` file, which contains the numpy array of coordinate correspondences. Each image has dimensions (512, 512). For eight-point algorithm, you have 10 points for each pair of scene. For four-point algorithm, you have 6 points for each pair of scene. 
# 2. coordinate correspondences are given to you as a numpy array, with dimensions (4, 2, 10, 2) for eight-point and (4, 2, 6, 2) for four-point. The dimensions correspond to `(num_scenes, camera angle, num_pts, pixel_coordinates)`. I highly encourage you to print out the array to get a better sense of the x, y, and z axis. 
# 3. To transform pixel coordinates to Euclidean system, you will need the intrinsic matrix. In our synthetic dataset, the intrinsic matrix `K` is the same for all scenes and camera angles. 
# 4. Some helpful functions are implemented in `utils.py`. I encourage you to take a look before starting. 

# In[3]:


K = np.array([[ 2.1875    ,  0.        ,  0.        ],  # intrinsic matrix
              [ 0.        ,  2.1875    ,  0.        ],
              [ 0.        ,  0.        ,  1.        ]])


# ## Eight Point Algorithm

# In[4]:


# data loading
folder = "./eight_point/"
img_arr = utils.load_scenes(folder)
utils.plot_scenes(img_arr)


# In[5]:


# read correspondences
corr_8 = np.load(os.path.join(folder, 'eight_pt_corr.npy'))
corr_8.shape


# In[6]:


scene_id =3 # change this to view other scenes
utils.plot_correspondences(img_arr, corr_8, scene_id)


# **TASK1**: Implement the eight-point algorithm below. 

# In[17]:

def eight_2(corr,im):
    pass

def hat2vec(T_hat):
    t = np.array([ T_hat[1][2], - T_hat[0][2], T_hat[0][1] ])
    return t

def eight_point_algorithm(corr,im):
    # Read the corresponding 8 points
    y1 = corr[im][0, :, 0]
    x1 = corr[im][0, :, 1]
    y2 = corr[im][1, :, 0]
    x2 = corr[im][1, :, 1]
    print(corr.shape)
    # Set the para matrix
    a = np.array(
             [[x1[0] * x2[0], x1[0] * y2[0], x1[0], y1[1] * x2[0], y1[0] * y2[0], y1[0], x2[0], y2[0]],
              [x1[1] * x2[1], x1[1] * y2[1], x1[1], y1[1] * x2[1], y1[1] * y2[1], y1[1], x2[1], y2[1]],
              [x1[2] * x2[2], x1[2] * y2[2], x1[2], y1[2] * x2[2], y1[2] * y2[2], y1[2], x2[2], y2[2]],
              [x1[3] * x2[3], x1[3] * y2[3], x1[3], y1[3] * x2[3], y1[3] * y2[3], y1[3], x2[3], y2[3]],
              [x1[4] * x2[4], x1[4] * y2[4], x1[4], y1[4] * x2[4], y1[4] * y2[4], y1[4], x2[4], y2[4]],
              [x1[5] * x2[5], x1[5] * y2[5], x1[5], y1[5] * x2[5], y1[5] * y2[5], y1[5], x2[5], y2[5]],
              [x1[6] * x2[6], x1[6] * y2[6], x1[6], y1[6] * x2[6], y1[6] * y2[6], y1[6], x2[6], y2[6]],
              [x1[7] * x2[7], x1[7] * y2[7], x1[7], y1[7] * x2[7], y1[7] * y2[7], y1[7], x2[7], y2[7]]
              ])
    # Assume e9 = -1
    b = np.array([-1, -1, -1, -1, -1, -1, -1, -1])
    # b = np.zeros((9,1))
    A = np.asmatrix(a)
    B = np.asmatrix(b).T
    e = np.linalg.solve(A, B)
    E = np.mat([[e[0], e[1], e[2]],
                  [e[3], e[4], e[5]],
                  [e[6], e[7], 1]
                  ])
    E = np.array(E, dtype='float')
    print("The essential matrix is:\n",E)
    negE = -E
    # The 1,2 solution
    U,S,Vh = np.linalg.svd(E)
    sigma = (S[0]+S[1])/2
    ss = np.diag([sigma,sigma,0])
    print("The singular value by myself:",S)
    # # E = U*S*Vh
    # # print(E)
    # theta = np.pi/2
    # Rz = np.array([[np.cos(theta), np.sin(theta), 0],
    #               [-np.sin(theta), np.cos(theta), 0],
    #               [0, 0, 1]])
    # T1_hat = ((U.dot(Rz)).dot(ss)).dot(U.T)
    # T1 = hat2vec(T1_hat)
    # print("!!!",T1.shape)
    # R1 = (U.dot(Rz)).dot(Vh)
    # theta = -np.pi / 2
    # Rz = np.array([[np.cos(theta), np.sin(theta), 0],
    #                [-np.sin(theta), np.cos(theta), 0],
    #                [0, 0, 1]])
    # T2_hat = ((U.dot(Rz)).dot(ss)).dot(U.T)
    # T2 = hat2vec(T2_hat)
    # R2 = (U.dot(Rz)).dot(Vh)
    # # The 3,4 solutions
    # U, S, Vh = np.linalg.svd(negE)
    # theta = np.pi / 2
    # Rz = np.array([[np.cos(theta), np.sin(theta), 0],
    #                [-np.sin(theta), np.cos(theta), 0],
    #                [0, 0, 1]])
    # T3_hat = ((U.dot(Rz)).dot(ss)).dot(U.T)
    # T3 = hat2vec(T3_hat)
    # R3 = (U.dot(Rz)).dot(Vh)
    # theta = -np.pi / 2
    # Rz = np.array([[np.cos(theta), np.sin(theta), 0],
    #                [-np.sin(theta), np.cos(theta), 0],
    #               [0, 0, 1]])
    # T4_hat = ((U.dot(Rz)).dot(ss)).dot(U.T)
    # T4 = hat2vec(T4_hat)
    # R4 = (U.dot(Rz)).dot(Vh)
    #
    # test_point = np.mat([0, 1, 0])
    # T1 = np.asmatrix(T1)
    # T2 = np.asmatrix(T2)
    # T3 = np.asmatrix(T3)
    # T4 = np.asmatrix(T4)
    # # set a point P = [1, 2, 3] p_1=[x,y,1] = 1/lambda *K*Pi*g*P
    # P = np.mat(np.array([1,2,3,1])).T
    # Pi_0 = np.mat(np.array([
    #                 [1, 0, 0, 0],
    #                 [0, 1, 0, 0],
    #                 [0, 0, 1, 0]] ))
    # g = np.hstack((R1,T1.T))
    # temp = np.mat(np.array([0,0,0,1]))
    # g = np.vstack((g,temp))
    # I4x4 = np.mat(np.eye(4,4))
    # p_1 = np.mat(np.array([x0[0],y0[0],1])).T
    # p_2 = np.mat(np.array([x1[0], y1[0], 1])).T
    # P = np. linalg.inv(K).dot(p_1)
    # print(P)
    # d2 = K.dot(R1.dot(P)+T1.T)
    # print("p2",p_2)
    # print("s2p2",d2)


def opencv8Points(corr, im):
    y0 = corr[im][0, :, 0]
    x0 = corr[im][0, :, 1]
    y1 = corr[im][1, :, 0]
    x1 = corr[im][1, :, 1]
    points1 = np.mat(np.array([x0,y0])).T
    points2 = np.mat(np.array([x1, y1])).T
    principle_points  = np.array([0,0])
    focal_len = 2.1857
    E = cv2.findEssentialMat(points1=points1,points2=points2,focal=focal_len,method=cv2.RANSAC)
    print("The essential matrix by OpenCV:\n",E[0])
    U,S,Vh = np.linalg.svd(E[0])
    print("The singular value by OpenCV\n", S)

opencv8Points(corr_8,1)
eight_point_algorithm(corr_8,1)
# print("The rotation is:\n", R)
# print("The translation is:\n", T)

# ## Four-point Algorithm

# In[18]:


# data loading
folder = "./four_point/"
img_arr = utils.load_scenes(folder)
utils.plot_scenes(img_arr)


# In[19]:


# read correspondences
corr_4 = np.load(os.path.join(folder,'four_pt_corr.npy'))


# In[23]:


# plotting correspondences
scene_id = 0
utils.plot_correspondences(img_arr, corr_4, scene_id)


# **TASK2**: Implement the four-point algorithm below

# In[10]:


def four_point_algorithm(corr, im):
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
    H = np.mat([[h[0], h[1], h[2]],
                [h[3], h[4], h[5]],
                [h[6], h[7], 0]
                ])
    print(H)
    H = np.array(H, dtype='float')
    R = None
    T = None
    return R, T

# four_point_algorithm(corr_4,1)
# End of Assignment
