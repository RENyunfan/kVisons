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

## 8 Points

The matrix K 

<img src="README.assets/image-20200311212631513.png" alt="image-20200311212631513" style="zoom:45%;" />

From epipolar constrain

<img src="README.assets/image-20200312205708129.png" alt="image-20200312205708129" style="zoom:15%;" />

So we first find the fundamental matrix by solving

<img src="README.assets/v2-c9d2f0995aa41d60b27288c2c9719462_720w-20200311170015408.jpg" alt="img" style="zoom:50%;" />

Then we calculate the essential matrix by:

![](https://www.zhihu.com/equation?tex=E+%3D+K%5ETFK+%5C%5C )

Then we do SVD and find the transformation:

![]( https://www.zhihu.com/equation?tex=E+%3D+U%5CSigma+V%5ET+%5C%5C )

![](https://www.zhihu.com/equation?tex=%5Chat+T_1+%3D+UR_Z%28%5Cfrac%7B%5Cpi%7D%7B2%7D%29%5CSigma+U%5ET%2CR_1+%3D+UR_Z%5ET%28%5Cfrac%7B%5Cpi%7D%7B2%7D%29V%5ET%5C%5C+%5Chat+T_2+%3D+UR_Z%28-%5Cfrac%7B%5Cpi%7D%7B2%7D%29%5CSigma+U%5ET%2CR_1+%3D+UR_Z%5ET%28-%5Cfrac%7B%5Cpi%7D%7B2%7D%29V%5ET%5C%5C )

Finally we select one solution from all 4 solutions:

![]( https://www.zhihu.com/equation?tex=s_1+p_1+%3D+KP%2Cs_2p_2+%3D+K%28RP%2Bt%29+%5C%5C )

We assume s1 > 0 to calculate P, then use the equation 2 to calculate s2. If s2 > 0 the solution is resonable.

## 4 Points

![v2-912c17475b8aa3b4dc2eb8719c56037a_b](README.assets/v2-912c17475b8aa3b4dc2eb8719c56037a_b.png)

