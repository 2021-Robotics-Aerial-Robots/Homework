![image](https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw1/LOGO%20%E4%B8%AD%E8%8B%B1%E6%96%87%E6%A9%AB.png)
# 110 年度 機器人學：多軸旋翼機 

### HW1
Refer to http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29 for more understanding of the code.

---
## 題目
鍵盤控制，請搜尋鍵盤對應的ASCII code，做出鍵盤控制turtlesim的功能

### 指令
```
	roscore
	roslaunch hw1 turtle_setting.launch
	rosrun hw1 hw1 
```
Ros node start from main, initialize ros node handler.
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw1/photo/week_1_1.png" width="80%" height="40%">

Declare ros publisher as massage type "geometry_msgs::Twist" and topic name "/turtle1/cmd_vel".
Define ros rate 100 Hz, duration for running the node will be 0.01 second.
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw1/photo/week_1_2.png" width="80%" height="40%">

Once the node enter this session, while loop will keep running and return unless ros is shutdown.
Function of KeyboardControl will be perform and "vel_msg" will publish through turtlesim_pub.
You may check the publish topic rate which should be 100 Hz, by command ```rostopic hz /turtle1/cmd_vel```

<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw1/photo/week_1_3.png" width="80%" height="40%">

KeyboardControl get keybroad command and you should implement switch to value of "vel_msg".
ASCII table may be find here in 可顯示字元/十進位 https://zh.wikipedia.org/wiki/ASCII.
One should notice that big and small capital have different value, e.g. small capital of "i" has the value of "105". You may press the key and check the result while printing.
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw1/photo/week_1_4.png" width="80%" height="40%">

Function of reading keybroad
Refer to getch part in https://edisonx.pixnet.net/blog/post/35585720.
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw1/photo/week_1_5.png" width="70%" height="30%">

---
