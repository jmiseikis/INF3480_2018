# INF3480 ROS Lecture (Spring 2018)
## University of Oslo - Justinas Mišeikis

This is the tutorial for the practical coding part of the INF3480/4380 lectures in Robot Operating System (ROS) in Spring 2018 semester. The tutorial is based on ROS Kinetic running on Ubuntu 16.04. You will need some understanding of using Linux, specifically Ubuntu, and coding knowledge in Python.

If you are a student at University of Oslo and enrolled for the course, you can follow the instructions during the lecture or try it after. It is highly recommended to get hands-on experience. We have our own ROS virtual machine that you can use. The instructions on how to connect will be given separately.

## Part A - ROS Lecture 1

In this part we will learn the following:
- How to setup ROS envirnoment
- How to build packages using `catkin_make`
- How to create your own ROS package
- How to write your own ROS node in Python
- How to publish messages on ROS Topic
- How to write ROS launch file

# Let's Start!

## Connect to ROS Virtual Machine

Use the following instructions to connect to our local ROS virtual machine. It is available only for the students officially enrolled for INF3480/4380 course at University of Oslo
https://github.com/jmiseikis/INF3480_2018/blob/master/ROS_VM_Connection_Instructions.md

## Prepare ROS

### Source the ROS path to .bashrc file

In order for the system to find ROS installation, you need to source it. This can be done by editing a .bashrc file in Ubuntu and adding the path to ROS. Let's do it all from the Terminal. You can launch it by going into Applications > System Tools > Terminal.

Then we add the line to source the path of ROS to `.bashrc` file by executing the following line.

```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Create catkin workspace
Let's create a catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```
Even though the workspace is empty (there are no packages in the 'src' folder, just a single CMakeLists.txt link) you can still "build" the workspace:
```
cd ~/catkin_ws/
catkin_make
```

Source the catkin_ws so our ROS packages can be found by ROS and Linux system
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in

`echo $ROS_PACKAGE_PATH`

You can test if the system works by launching `roscore` in the terminal. If it runs successfully, the setup is complete!

### Create a new ROS package

Now we will create a new ROS package. When creating a new ROS package, we need to consider the dependencies that we will be using and add them as parameters. In this case, our work will depend on rospy and geometry_msgs. Rospy is the Python interface for ROS, so we can write scripts in Python language. Equivalent of C++ would be `roscpp`. And we will be sending coordinate messages, so we want to import geometry_msgs, which include suitable message format for us. New package can be created by using `catkin_create_pkg` followed by our package name and all the dependencies listed afterwards. Use the following command to create a package named "inf3480". Full tutorial on package creation can be found http://wiki.ros.org/ROS/Tutorials/CreatingPackage

```
cd ~/catkin_ws/src
catkin_create_pkg inf3480 rospy geometry_msgs
```

Now recompile the package using catkin_ws:

```
cd ~/catkin_ws/
catkin_make
```

Now we will see that the package was detected and successfully compiled despite having no scripts or source files.

### Write a python script to publish coordinates

Official ROS tutorial on writing a publisher can be found: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
Let's create a script to do something! Navigate to the package directory and create a new directory for scripts:

```
roscd inf3480
mkdir scripts
cd scripts
```

Now let's create a script file `coords_publisher.py`:

```
code coords_publisher.py
```

What we want is to publish coordinates on the topic `/coords` using the message format `geometry_msgs/Point` 

```
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

def talker():
    pub = rospy.Publisher('coords', Point, queue_size=10)
    rospy.init_node('coords_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    position = Point()
    position.x = 10
    position.y = 5
    position.z = 1

    while not rospy.is_shutdown():
        rospy.loginfo(position) # This is only for debugging
        pub.publish(position)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

After the script is created, we need to make it an executable in order to be able to launch it:

```
chmod +x coords_publisher.py
```

Let's test it!

In the first terminal window launch roscore
```
roscore
```

In the second terminal window run:
```
rosrun inf3480 coords_publisher.py
```

It runs! But we don't really see if it really publishes... How do we make sure that the script really publishes the correct messages?

First of all, let's list all the active topics. In the third terminal window type:
```
rostopic list
```

Cool! Our topic `/coords` is there! And now let's see what messages are being published there... In the same Terminal window type:
```
rostopic echo /coords
```

And here you see all the messages that our newly written node is sending out on this topic!

### Write a launch file

And the last thing we will look at today is writing a small launch file. As you see, we need 3 Terminal windows for running and checking this very simple program. And you can image what would happen in we need to run `roscore` as well as multiple nodes, and there can be way over 10 nodes in some systems. That's where launch files come in handy. These are XML based configuration files, which will automatically launch the needed nodes for our needs and can also pass parameters and add more advanced configuration. Today we will write a very simple one.

First, let's navigate to the correct directory and create a new launch file
```
roscd inf3480
mkdir launch
cd launch
code start_publisher.launch
```

Then write the following code

```
<launch>
  <group ns="inf3480">
    <node pkg="inf3480" name="coords_publisher_node" type="coords_publisher.py" output="screen"/>
  </group>
</launch>
```

Now save the file stop any processes running in all the terminal windows, including roscore, by using `Ctrl+C`.

Roslaunch automatically starts `roscore` and then starts any nodes defined in the launch file. Let's try to run our script using the launch file:
```
roslaunch inf3480 start_publisher.launch
```

Here you go, all the processes are running the same way as before, but now we only need one line in one Terminal window to start our program.

Well done! This is the end of this part of the tutorial and we will continue the introduction to libraries like MoveIt! next week as well as coding a more complex system.
