# mobile_robot_ros_driver

Данный  ROS пакет позволяет управлять роботом через последовательный порт. Реализован на python без всяких внешних зависимостей.


serialdrive.py - драйвер, подключается к сериал и считывает данные одометрии, также слушает топики /cmd и отправляет команды на serial.

example.py - простой контроллер, отправляет управляющие команды драйверу.


Related project [here](https://github.com/industrial-robotics-lab/OmniCar-arduino).