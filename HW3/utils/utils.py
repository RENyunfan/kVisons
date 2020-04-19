import numpy as np


def find_unique_indices(juncsidx):
    unique_indices = set()
    for img_idxs in juncsidx:
        for idx in img_idxs:
            unique_indices.add(idx)
    # DO NOT INCLUDE T-JUNCTIONS
    unique_indices.remove(-1),
    return unique_indices, len(juncsidx)



RTP = np.array([[ 0   ,  1     ,  0.        ],
              [ -1        ,  0 ,  0.        ],
              [ 0.        ,  0.        ,1  ]])
RTN = np.array([[ 0   ,  -1     ,  0.        ],
              [ 1        ,  0 ,  0.        ],
              [ 0.        ,  0.        ,1  ]])
def img2world(img, K):
    x = img[:, 1]
    y = img[:, 0]
    z = np.ones_like(x)
    coor = np.stack([x, y, z]).T
    return coor @ np.linalg.inv(K).T


def world2img(X, K):
    X_ = np.divide(X, X[:, 2, np.newaxis])
    im = X_ @ K.T
    return np.array([im[:, 1], im[:, 0]]).T

def hat_vec_switch(T):
    T = np.array(T)
    if T.shape == (3, 3):
        return np.array([T[2, 1], T[0, 2], T[1, 0]])
    else:
        if(T.shape == (1,3)):
            T = T.T
        return np.array([[0, -T[2], T[1]],
                         [T[2], 0, -T[0]],
                         [-T[1], T[0], 0]])


def eight_point_algorithm(corr,K):
    y1 = corr[0, :, 0]
    x1 = corr[0, :, 1]
    y2 = corr[1, :, 0]
    x2 = corr[1, :, 1]
    chi = np.mat(np.zeros((9, 1)))
    for i in range(0, len(x1)):
        p_1 = np.array([x1[i], y1[i], 1]).T
        p_2 = np.array([x2[i], y2[i], 1]).T
        x_1_ = np.linalg.inv(K).dot(p_1)
        x_2_ = np.linalg.inv(K).dot(p_2)
        a_ = np.mat(np.kron(x_1_, x_2_))
        chi = np.hstack((chi, a_.T))
    chi = chi[:, 1:].T
    evalue, evector = np.linalg.eig(chi.T@chi)
    minVec = evector[:,np.argmin( evalue)]
    E0 = (minVec.reshape(3, 3)).T
    Rzs = [RTN, RTP]
    p_1 = np.array([x1[0], y1[0], 1]).T
    p_2 = np.array([x2[0], y2[0], 1]).T
    x_1 = np.linalg.inv(K).dot(p_1)
    x_2 = np.linalg.inv(K).dot(p_2)
    U, S, Vh = np.linalg.svd(-E0)
    ss = np.diag([1, 1, 0])
    for Rz in Rzs:
        for RzT in Rzs:
            R_ = U @ Rz.T @ Vh
            T_hat = U @ RzT @ ss @ U.T
            T_ = hat_vec_switch(T_hat)
            if (hat_vec_switch(T_) @ x_2 @ hat_vec_switch(T_) @ R_ @ x_1 > 0
            and hat_vec_switch(x_2) @ R_ @ x_1 @ hat_vec_switch(x_2) @ T_ <= 0
            and hat_vec_switch(R_ @ x_1) @ x_2 @ hat_vec_switch(R_ @ x_1) @ T_ > 0):
                R = np.mat(R_)
                T = np.mat(T_).T

    print(R,T)
    return R, T
def find_alpha(view1, view2, R, T):
    # P280 formula 8.43

    alphas = []
    for x_1, x_2 in zip(view1, view2):
        # Based of 8.4.3
        alpha = -(hat_vec_switch(x_2) @ T).T @ hat_vec_switch(x_2) @ R @ x_1
        alpha /= np.linalg.norm(hat_vec_switch(x_2) @ T) ** 2
        alphas.append(alpha)
    return np.array(alphas)

def update_alphas(points, projection_matrices):
    # P280 formula 8.44
    alphas = []
    N_IMAGES, N_POINTS = points.shape[0], points.shape[1]
    view0 = points[0]
    for j in range(1, N_POINTS + 1):
    ## for every alpha in 1,2,...N_POINTS
        term_1 = 0
        view_1 = points[0]
        for i, (R, T) in enumerate(projection_matrices.values(), start=2):
            view_i = points[i - 1]
            term_1 += (hat_vec_switch(view_i[j - 1]) @ T).T @ hat_vec_switch(view_i[j - 1]) @ R @ view_1[j - 1]
        term_2  = 0
        for i, (R, T) in enumerate(projection_matrices.values(), start=2):
            view_i = points[i - 1]
            term_2 += np.linalg.norm(hat_vec_switch(view_i[j - 1]) @ T) ** 2
        alpha_j = - term_1 / term_2
        alphas.append(alpha_j)
    return np.array(alphas)

def sorted_SVD(X, full_matrices=True):
    '''SVD that sorts singular values'''
    U, S, V = np.linalg.svd(X, full_matrices=full_matrices)
    print(U,'\n')
    indices = np.argsort(S)[::-1]
    U = U[:, indices]
    V = V[indices, :]
    print(U)
    return U, S, V
def estimate_R_T(points, idx_i, alpha):
    """
    Args:
      points: coordinate of junctions of shape [N_IMAGES, N_POINTS, 3]
      idx_i : index of corresponding second-camera view
      alpha : inverse of depth values for views 1,2,....,N_POINTS
    """
    N_POINTS = points.shape[1]
    view_1, view_i = points[0], points[idx_i - 1]

    P_i = np.zeros((N_POINTS * 3, 12))
    # prepare for batch kronecker product
    for j in range(N_POINTS):
        hat_j = hat_vec_switch(view_i[j])
        kron = np.kron(view_1[j].T, hat_j)
        P_i[j * 3:(j * 3) + 3, :] = np.concatenate((kron, alpha[j] * hat_j), axis=1)

    ### step 2, compute R_i_tile, T_i_tile from the singular-vector associated with the smallest singular value
    U, S, V_T = sorted_SVD(P_i, full_matrices=True)
    R_s_tilde, T_tilde = V_T[11, :9], V_T[11, 9:]
    R_tilde = R_s_tilde.reshape((3, 3)).T
    ## step 3, compute R_i, T_i from (8.4.1) and (8.4.2) using the SVD of R_i_tilde
    U_i, S_i, V_T_i = sorted_SVD(R_tilde, full_matrices=True)
    R_i = np.sign(np.linalg.det(U_i @ V_T_i)) * (U_i @ V_T_i)
    T_i = (np.sign(np.linalg.det(U_i @ V_T_i)) / (np.linalg.det(np.diag(S_i)) ** (1 / 3))) * T_tilde
    return R_i, T_i



def compute_reprojections(alpha, projection_matrices, first_view):
    """
    alpha: inverse of depth for every point
    projection_matrices: R,T for every image with respect to the first view
    """
    N_POINTS, N_IMAGES = alpha.shape[0], len(projection_matrices)
    x_proj = np.zeros((N_IMAGES, N_POINTS, 3))
    for v, (R, T) in enumerate(projection_matrices.values()):
        for i, x_0 in enumerate(first_view):
            proj = (1 / alpha[i]) * R @ x_0 + T
            x_proj[v, i] = proj
    return x_proj