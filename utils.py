import os

import cv2
import numpy as np
import matplotlib.pyplot as plt


## data loading functions
def load_scenes(path):
    '''Load scenes into numpy array. Return array with dimension
    (num_scenes, 2, height, width).'''

    img_files = [os.path.join(path, filename) for filename in os.listdir(path) 
                     if filename[-4:] == ".png"]
    img_files = np.sort(img_files)
    img_arr = []
    for im in range(0, len(img_files), 2):
        img_arr.append([cv2.imread(img_files[im]), 
            cv2.imread(img_files[im+1])])
    return np.stack(img_arr)


## plotting functions
def plot_scenes(img_arr, num_scenes=4):
    '''Plots pairs of scenes.'''

    fig, ax = plt.subplots(ncols=num_scenes, nrows=2, sharey=True,
                           figsize=(num_scenes, 2), dpi=300)
    k = 0
    for c in range(num_scenes):
        ax[0, c].imshow(img_arr[c, 0])
        ax[1, c].imshow(img_arr[c, 1])
        ax[0, c].set_axis_off()
        ax[1, c].set_axis_off()
    plt.show()

def plot_correspondences(img_arr, corr, scene_id):
    '''Plot one pair of scenes with respective correspondences.
    img_arr should have shape (num_scenes, 2, height, wdith),
    corr should have shape (num_scenes, 2, num_pts, 3),
    scene_id should be integer, deciding which scene to view. 

    '''
    im_i = scene_id # image index 0-9 (please change this to see other scenes)
    y0 = corr[im_i][0, :, 0]
    x0 = corr[im_i][0, :, 1]
    y1 = corr[im_i][1, :, 0]
    x1 = corr[im_i][1, :, 1]
    fig, ax = plt.subplots(ncols=2, nrows=1, figsize=(7, 5), dpi=200)
    ax[0].imshow(img_arr[im_i, 0])
    ax[0].scatter(x0, y0, c='r', s=5)
    ax[0].set_title('camera angle 0')
    ax[1].imshow(img_arr[im_i, 1])
    ax[1].scatter(x1, y1, c='r', s=5)
    ax[1].set_title('camera angle 1')
    plt.show() 