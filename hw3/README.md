![image](https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw3/Figures/LOGO%20%E4%B8%AD%E8%8B%B1%E6%96%87%E6%A9%AB.png)
# 108 年度 機器人學：多軸旋翼機 

### HW3
Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms. \
Deadline: 4/6!!!
---

## 題目
學習基本Eigen操作，並了解旋轉矩陣和四元數與歐拉角的關係， \
更多Eigen應用可參考https://eigen.tuxfamily.org/dox/group__QuickRefPage.html

### 指令

eigen_tutorial.cpp 不須更改程式碼，看懂Eigen操作即可 \
eigen_transform.cpp 請在(Implement your code here)輸入對應程式

```
	roscore
	rosrun HW3 eigen_tutorial
	rosrun HW3 eigen_transform 
```

### eigen_tutorial.cpp
Declare Eigen 4x4 Matrix with each element as double. \
We can directly given the matrix value by <<. \
Matrix addition and subtraction can also be apply by + and -. \
transform1 represent rotation of Z-axis in -90 and translation of X-axis in 1.  \
transform2 represent rotation of Z-axis in 90 and translation of Y-axis in -1.  \
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw3/Figures/rotation.png" width="50%" height="20%">
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw3/Figures/1.png" width="80%" height="40%">

Declare Eigen 3x1 Vector with each element as double. \
Vector dot and cross operation are applied.
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw3/Figures/2.png" width="80%" height="50%">

Declare Eigen 3x3 Matrix and 3x1 Vector. \
Bying apply block operation to Eigen 4x4 Matrix, we can obtain a sub-block. \
For example, block<3,3>(0,0) means extract a Eigen 3x3 Matrix from the matrix in (0,0) element. \
Then apply two transformation to the vector. \
We should notice applying rotation first and translation first will have different result. \
These two types of transformation is oftenly used, one should use rotation first or translation first depend on the frame of transformation. \
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw3/Figures/3.png" width="80%" height="50%">

### eigen_transform.cpp
Declare Eigen 3x1 Vector of point (1,0,0) and euler angle (0,0,90) in degree.
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw3/Figures/4.png" width="80%" height="40%">

Convert euler angle to radians.  \
Then convert to quaternion. \
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw3/Figures/5.png" width="70%" height="30%">

Implement the code for conversion.

<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw3/Figures/8.png" width="70%" height="30%">

We use ZYX for Euler to quaternion.
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw3/Figures/9.png" width="80%" height="40%">

Implement the code for conversion.

<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw3/Figures/Q2E.png" width="50%" height="10%">
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw3/Figures/10.png" width="80%" height="40%">

Eigen slerp is a function to obtain a propotion rotation. \
Here we set s=0.667, so the 0.667 of 90 degrees will be 60.
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw3/Figures/6.png" width="80%" height="40%">

Apply the rotation after slerp(Q_tmp) to the point(world_point). \
(**Use Q_tmp to rotate world_point and update world_point**) \
The while loop should break after six times of rotation.
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw3/Figures/7.png" width="90%" height="50%">

Your result should look like this. \
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw3/Figures/transform_result.png" width="60%" height="40%">

---
