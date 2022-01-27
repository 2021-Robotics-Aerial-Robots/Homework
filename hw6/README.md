![image](https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw6/Figures/LOGO%20%E4%B8%AD%E8%8B%B1%E6%96%87%E6%A9%AB.png)
# 108 年度 機器人學：多軸旋翼機 

### HW6
---
## 題目
請實踐從continuous-time的state space轉換到discrete-time的形式 \
作法可使用數值積分的方式實現，積分得出狀態後  \
因此能繪出 位置-時間圖 (可使用rqt_plot 或 PlotJuggler)
### 指令
```
	roscore
	rosrun HW6 hw6
	rosbag record -O hw6 /Position
	-------after recording------------------
	rosrun plotjuggler PlotJuggler
	(之後點file,Load data,點選你的bag file)
```
### hw6.cpp
假設輪子與平面無磨擦力干擾  \
輸入 ``u`` 為unit step 訊號(力)
輸出 ``x(t)``(量測訊號) 為 位移  \
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw6/Figures/images.png" width="40%" height="20%">

<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw6/Figures/%E5%BE%AE%E5%88%86.PNG" width="20%" height="10%">
使用上述形式可實作到你的code  

### PlotJuggler
下載網址:https://github.com/facontidavide/PlotJuggler 
及操作

### 作業上傳
除了上傳作業的cpp檔外，先錄好bag, \
開啟PlotJuggler, 點選你的bag file \
用Kazam或其他可以錄螢幕畫面的軟體紀錄你的**動畫**, \
,**並將你的動畫影片連結附在你的Readme.md** \
**或gif檔都可以** \
<img src="https://github.com/2020-Robotics-Aerial-Robots/Homework/blob/main/hw6/Figures/plot.PNG" width="80%" height="40%">


