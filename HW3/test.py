import numpy as np
import json
import math
import os
import os.path as osp
import random
import sys
from glob import glob

import cv2
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import numpy.linalg as LA
from HW3.utils.utils import *

N_IMAGE = 4

color = plt.rcParams["axes.prop_cycle"].by_key()["color"]
fs = glob("data/00/*_0.jpg")
fs.sort()
lines = []
juncs = []
lineids = []
juncids = []
images = []
Rt = [] # world to camera matrix (for verification purpose only. DO NOT USE THIS MATRIX IN YOUR IMPLEMENTATION)
K = None # camera to pixel matrix

for fjpg in fs[:N_IMAGE]:
    fjs = fjpg.replace(".jpg", "_label.json")
    js = json.load(open(fjs))

    line = js["line"]
    lineidx = js["lineidx"]
    junc = np.array(js["junction"])
    juncidx = js["junindex"]
    K = js["K"]
    Rt.append(np.array(js["RT"]))

    junc[:, 1] *= -1
    junc *= 256
    junc += 256

    lines.append(line)
    juncs.append(junc)
    lineids.append(lineidx)
    juncids.append(juncidx)

    I = cv2.imread(fjpg)[..., ::-1]
    plt.figure()
    plt.imshow(I)
    images.append(I)
    for ln, idx in zip(line, lineidx):
        print(ln, idx, junc[ln[0], 0], junc[ln[0], 0])
        idx += len(color)
        plt.plot(
            [junc[ln[0], 0], junc[ln[1], 0]],
            [junc[ln[0], 1], junc[ln[1], 1]],
            c=color[idx % len(color)],
        )
    for jun, idx in zip(junc, juncidx):
        idx += len(color)
        print(jun, idx)
        plt.scatter(jun[0], jun[1], c=color[idx % len(color)])

# convert projection matrix (OpenGL format) to OpenCV format to avoid confusion
# the unit of the projection matrix is pixel now (OpenCV format).
# If you are still confusing, you may want to read
#     http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/OWENS/LECT9/node2.html
#     https://github.com/vvvv/VL.OpenCV/wiki/Coordinate-system-conversions-between-OpenCV,-DirectX-and-vvvv
K = np.array([[K[0][0] * 256, 0            , 256],
              [0            , K[1][1] * 256, 256],
              [0            , 0            , 1]])

d

xs = extract_corresponding_junction(juncs, juncids)
print(xs.shape[1], "common junction extracted from", len(juncs), "images")
# visualize common junctions
color = plt.rcParams["axes.prop_cycle"].by_key()["color"]
for im_i, I in enumerate(images):
    plt.imshow(I)
    for pt_i, (y, x) in enumerate(xs[im_i]):
        plt.scatter(y, x, c=color[pt_i % len(color)])
    plt.show()





def factorization_algorithm(xs, K):
    """Factorization algorithm for multiple-view reconstruction.

    Args:
      xs: coordinate of junctions of shape [N_IMAGES, N_POINTS, 2]
      K: camera projection matrix (OpenCV format)

    Returns:
      alpha: inversion of depth of each junction in the reference view with shape [N_POINTS]
      Rs: rotation matrix w.r.t the first view of other view with shape [N_IMAGES, 3, 3]
      Ts: translation vector w.r.t the first view of other view with shape [N_IMAGES, 3]
    """
    N_IMAGES, N_POINTS = xs.shape[0], xs.shape[1]
    views = np.zeros((N_IMAGES, N_POINTS, 3))
    for i in range(N_IMAGES):
        views[i] = img2world(xs[i], K)
    view1, view2 = views[0], views[1]
    eps = 1e-4
    num_iters = 1
    iter_check = 1

    """
    Algorithm 8.1
    """
    # 1. Initialization k = 0
    k = 0
    # 1.(a) Compute [R2, T2] using the 8 point algorithm.
    R_2, T_2 = eight_point_algorithm(xs, K)
    R_2, T_2 = eight_point_algorithm2(view1, view2)
    # 1.(b) Compute alpha from (8.43)
    alpha = find_alpha(view1, view2, R_2, T_2)
    # 1.(c) normalize alpha with respect to first value
    alpha = alpha / alpha[0]

    # 2 Compute [Ri, Ti] from (8.40)
    x_proj = np.zeros_like(views)
    projection_matrices = {i: (None, None) for i in range(2, N_IMAGES + 1)}

    while np.linalg.norm(x_proj - views) ** 2 > eps and k < num_iters:
        for i in range(2, N_IMAGES + 1):
            # 2,3
            N_POINTS = views.shape[1]
            view_1, view_i = views[0], views[i - 1]

            P_i = np.zeros((N_POINTS * 3, 12))
            # prepare for batch kronecker product
            for j in range(N_POINTS):
                hat_j = hat_vec_switch(view_i[j])
                kron = np.kron(view_1[j].T, hat_j)
                P_i[j * 3:(j * 3) + 3, :] = np.concatenate((kron, alpha[j] * hat_j), axis=1)

            # 2, compute R_i_tile, T_i_tile from the singular-vector associated with the smallest singular value
            U, S, V_T = sorted_SVD(P_i, full_matrices=True)
            R_s_tilde, T_tilde = V_T[11, :9], V_T[11, 9:]
            R_tilde = R_s_tilde.reshape((3, 3)).T
            # 3, compute R_i, T_i from (8.4.1) and (8.4.2) using the SVD of R_i_tilde
            U_i, S_i, V_T_i = sorted_SVD(R_tilde, full_matrices=True)
            R_i = np.sign(np.linalg.det(U_i @ V_T_i)) * (U_i @ V_T_i)
            T_i = (np.sign(np.linalg.det(U_i @ V_T_i)) / (np.linalg.det(np.diag(S_i)) ** (1 / 3))) * T_tilde
            projection_matrices[i] = R_i, T_i
        # 4 Compute the new alpha and normalize
        alpha = update_alphas(views, projection_matrices)
        alpha_1 = alpha[0]
        # Update translation
        for _, T_i in projection_matrices.values():
            T_i *= alpha[0]
        alpha /= alpha[0]
        # Reprojection and find the reprojecion error
        x_proj = compute_reprojections(alpha, projection_matrices, views[0])
        x_proj = np.concatenate((views[0][np.newaxis, :], x_proj), axis=0)
        error = np.linalg.norm(x_proj - views)
        if k % iter_check == 0:
            print("Reprojection error:",error,". Tteration time:", k)
        k += 1
    R_s, T_s = np.zeros((N_IMAGES, 3, 3)), np.zeros((N_IMAGES, 3))
    for i in range(1, N_IMAGES + 1):
        if i == 1:
            R_s[i - 1] = np.eye(3)
        else:
            R_i, T_i = projection_matrices[i]
            R_s[i - 1] = R_i
            T_s[i - 1] = T_i.T

    return alpha, R_s, T_s

alpha, Rs, Ts = factorization_algorithm(xs, K)


# Verification
for i in range(N_IMAGE):
    # R should be a rotation matrix
    assert np.allclose(Rs[i] @ Rs[i].T, np.eye(3), atol=1e-4), i

    # compare with ground truth
    Rt_gt = Rt[i] @ np.linalg.inv(Rt[0])
    R_gt = Rt_gt[:3, :3]
    T_gt = Rt_gt[:3, 3]

def visualize_reprojection(I, x, alpha, R, T, K):
    """Verify the algorithm by reproject x using R, T, and K to I. Visualization using matplotlib.

    Args:
      I: image (new view)
      x: coordinate of junctions in reference view
      alpha: inversion of depth of each junction in the first view with shape [N_POINTS]
      R: rotation with respect to the reference
      T: translation with respect to the reference
      K: camera projection matrix
    """
    plt.imshow(I)
    # YOUR CODE to draw the reprojected junctions
    xs_world = img2world(x, K)

    pview1 = []
    for i, x0 in enumerate(xs_world):
        proj = R @ x0 / alpha[i] + T
        pview1.append(proj)

    pview1 = np.squeeze(pview1)
    pcam1 = world2img(pview1, K)
    print(pcam1.shape)
    plt.scatter(pcam1[:, 0], pcam1[:, 1], c='blue', s=6)
    plt.show()


for i in range(N_IMAGE):
    visualize_reprojection(images[i], xs[0], alpha, Rs[i], Ts[i], K)