# Install Turtlebot3 di ROS1 Noetic dan Gazebo11

## 1. Buat direktori untuk menyimpan files/dependencies turtlebot3
```
mkdir -p ~/test_turtlebot/src
cd ~/test_turtlebot/src
```

## 2. Clone repository
```
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

## 3. Build packagesnya
```
cd ~/test_turtlebot
catkin build
```

## 4. Source packagesnya
```
source ~/test_turtlebot/devel/setup.bash
```

## 5. Simulasi di Gazebo
Terdapat 3 jenis robot yaitu: Burger, Waffle, dan Waffle Pi
https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview

Memilih jenis robot yang ingin digunakan di Terminal-1, disini saya memilih burger
```
export TURTLEBOT3_MODEL=burger
```

Menjalankan simulasi <br>

World Kosong:
```
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
```

World Lain:
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch 
```

Sebetulnya terdapat beberapa world yang ada pada package bawaan Turtlebot3, bisa kalian explore sendiri di direktori src

## 6. Mengendalikan TurtleBot3 (Teleoperation Mode)

Buka jendela Terminal baru
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 
```
Untuk mengendalikan, tekan WASD pada keyboard, pastikan Terminal-2 aktif

Referensi:
- https://www.youtube.com/watch?v=_RBZ-9_rz9Y
- https://emanual.robotis.com/docs/en/platform/turtlebot3/features/