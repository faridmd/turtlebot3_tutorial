# Memprogram Turtlebot3 dengan Python

Program ditulis dalam Object Oriented Programming
## 1. Buat file python baru
```
main.py
```
## 2. Import library rospy, modul dan nama message yang mau dipakai
```
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
```

## 3. Buat class baru
Init node, bebas mau nama nodenya apa. Argumen anonymous=True digunakan agar node dibuat unik dengan menambahkan angka acak ke nama yang diberikan
```
class Turtle():
        def __init__(self):
            rospy.init_node('turtle_node', anonymous=True)
```

## 4. Buat object publisher di dalam init
Object dibuat dengan isi argumen (nama topic, tipe message, dan queue size)

- nama topic dan tipe message : bisa dilihat di rqt atau bisa melalui command rostopic info /cmd_vel
```
self.cmd_vel_object = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
```

## 5. Buat object yang mau di publish
Topic /cmd_vel terdapat dua opsi yang bisa digunakan untuk mengontrol gerak dari Turtlebot3 yaitu: Linear dan Angular.

### Linear:<br>
Biasanya digunakan untuk mengatur seberapa cepat robot bergerak ke depan atau mundur. Komponen ini dapat dipecah menjadi tiga sumbu (x, y, z).

### Angular:<br>
Digunakan untuk mengatur seberapa cepat robot berputar di tempat. Komponen ini juga dapat dipecah menjadi tiga sumbu (roll, pitch, yaw), tetapi untuk Turtlebot3 ini, hanya komponen yaw (sumbu z) yang sering digunakan.

Program dibawah ini mengpublish linear x dengan nilai kecepatan 0.2. Ini berarti akan menggerakkan Turtlebot maju dengan kecepatan 0.2. Bagaimana kalau ingin mundur?. Ya tinggal diganti saja dengan -0.2.

> Note: <br> Tidak semua sumbu linear dan angular bekerja pada Turtlebot3
```
self.vel_msg = Twist()

# Linear
self.vel_msg.linear.x = 0.2
# self.vel_msg.linear.y = 0
# self.vel_msg.linear.z = 0

# Angular
# self.vel_msg.angular.x = 0
# self.vel_msg.angular.y = 0
# self.vel_msg.angular.z = 0
```

## 6. Buat object rospy rate dan perulangan
rospy.rate(30) digunakan untuk mengatur pesan yang dikirim dalam 1 detik, disini kita menggunakan 30Hz yang berarti terdapat 30 pesan yang dikirim dalam 1 detik

Agar loop berjalan terus menerus sampai node dihentikan, kita menggunakan while yang akan terus mengulang apabila rospy node tidak dihentikan

self.cmd_vel_object.publish(self.vel_msg) digunakan untuk mengirim/mengpublish pesan yang berisi kecepatan yang sudah di inisialisasi sebelumnya 

r.sleep() digunakan untuk membatasi perulangan. Jika perulangan berjalan lebih cepat dari 30 kali per detik, fungsi ini akan memperlambatnya agar sesuai dengan kecepatan yang diinginkan.
```
r = rospy.Rate(30) # 30 times per second

while not rospy.is_shutdown():
    self.cmd_vel_object.publish(self.vel_msg)

    r.sleep()
```

## 7. Buat method shutdown
self.cmd_vel_object.publish(Twist()) berarti mengirimkan Twist() tanpa parameter berarti mengatur kecepatan linear dan angular ke nol, sehingga menghentikan pergerakan Turtlebot3.

rospy.sleep(1) digunakan untuk membuat sleep selama 1 detik untuk memastikan pesan penghentian dijalankan
 ```
def shutdown(self):
    rospy.loginfo('Stopping the turtle')

    self.cmd_vel_object.publish(Twist())

    rospy.sleep(1)
 ```

 ## 8. Buat main program
 Exception `ROSIntteruptException` digunakan untuk menangkap pengecualian jika ROS dihentikan secara mendadak (ctrl + c di terminal)
 ```
if __name__ == '__main__':
    try:
        Turtle()
    except rospy.ROSInterruptException:
        pass
    except:
        rospy.loginfo('Node Terminated')
 ```

 ## 9. Full Code
 ```
 #!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class Turtle():
    def __init__(self):
        rospy.init_node('turtle_node', anonymous=True)

        rospy.loginfo('Press ctrl+c to stop')
        rospy.on_shutdown(self.shutdown)

        # rospy.spin()
        # rospy.signal_shutdown(self.shutdown)

        self.cmd_vel_object = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.vel_msg = Twist()
        # Linear
        self.vel_msg.linear.x = 0.2
        # self.vel_msg.linear.y = 0
        # self.vel_msg.linear.z = 0

        # Angular
        # self.vel_msg.angular.x = 0
        # self.vel_msg.angular.y = 0
        # self.vel_msg.angular.z = 0

        r = rospy.Rate(30) # 30 times per second

        while not rospy.is_shutdown():
            self.cmd_vel_object.publish(self.vel_msg)

            r.sleep()

    def shutdown(self):
        rospy.loginfo('Stopping the turtle')

        self.cmd_vel_object.publish(Twist())

        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Turtle()
    except rospy.ROSInterruptException:
        pass
    except:
        rospy.loginfo('Node Terminated')
 ```

Referensi:
- https://www.youtube.com/watch?v=lQqH1bFuYko
- http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber