# CS294 HW2-Prog
> Name: Yunfan REN
> SID: 3035535419
>
> E-mail: renyunfan@berkeley.edu

# Description

This project is based on CS294 of Berkeley 2020 spring.

# Result

```bash
Using the image 2
------------------------ 8 Point ----------------------------
The rotation is:
 [[ 6.94105429e-04  9.96131455e-01 -8.78728811e-02]
 [ 9.99569714e-01 -3.26792122e-03 -2.91497482e-02]
 [-2.93241427e-02 -8.78148376e-02 -9.95705101e-01]]
The translation is:
 [-8.72103096e-02 -4.85287941e-04 -9.96189804e-01]
------------------------ 4 Point ----------------------------
Rotation matrix R :
 [[ 8.63234935e-01 -7.07965599e-02  3.36349274e-03]
 [-9.24358589e-02  1.08383554e-01 -1.03185577e-02]
 [-3.20926409e-05  8.67529273e-03  8.70102926e-02]]
Translation vector T:
 [[-1.17661594]
 [-9.88376738]
 [-0.08577937]]

Process finished with exit code 0
```



# Theory

See all in Jupiter notebook.



# Collaboration

The show function below is in collaboration with Anxing Xiao.

```python
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
```

