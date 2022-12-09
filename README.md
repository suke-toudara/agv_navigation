# agv_navigation
トマトロボットをAGV化させるべくナビゲーションを追加

センサ(追加予定）
realsense405


---

jetson　　　　　　　　　　　arduino
velocityをシリアル通信　→　各モータに指示　\n
自己位置推定に利用　　　　←　エンコーダの値から移動量推定、imuのデータも送信(シリアル) \n

jetson側のプログラム

arduino側のプログラム
- MotorController.cpp
- MotorController.h
- PID_RoboCraw.ino
