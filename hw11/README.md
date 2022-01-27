![image](https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw6/Figures/LOGO%20%E4%B8%AD%E8%8B%B1%E6%96%87%E6%A9%AB.png)

### Topic
---
Please complete the following two parts for this homework.

1. Please finish the update function in kalman.cpp (you can check out page 20 of the ppt. from week_12)
2. Please set parameters in the kf_test.cpp by using the following dynamic model.

Dynamic model:

<img src= "https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw11/photo/1.png" width="40%" height="20%">	
	
<img src= "https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw11/photo/11.png" width="90%" height="20%">


Suppose that the car is equipped with a position sensor that measures its output y with an additive noise v, please find the state estimate (i.e., position, velocity, acceleration) and compare it with the true state.

### Real data
The initial condition of the true state is x0=[0; 0; 0].

###  Reminder
1. Replace the word "ncrl" in produce_data1.py and kf_test.cpp with your own username.
2. The data.csv and output.cvs should be under the directory named src. 
3. Everytime before you execute kf_test, you should remove the file "data.csv".

### Instruction
```
  roscore
  rosrun hw11 kf_test
  
  chmod +x produce_data1.py
  rosrun hw11 produce_data1.py
```

### Result
<img src= "https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw11/photo/result.png" width="40%" height="20%">

