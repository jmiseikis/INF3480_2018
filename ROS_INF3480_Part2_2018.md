## Part B - ROS Lecture 2

We continue condsidering you have completed the first part of the tutorial and have the whole setup at the point, where we finished.

In this part we will learn the following:
- Write a custom message
- How to write a subscriber
- How to integrate new message to your node
- How to modify ROS launch to start couple of nodes
- RQT Tools
- Record and play rosbag files

## Code from the last lecture

You can clone the repository with the code from last week's lecture, so we are all at the identical starting point. Clone it to your `~/catkin_ws/src` and then use `catkin_make` to compile it.

```
git clone https://github.com/jmiseikis/inf3480
```

## Creating our own ROS message

In some cases you want to create a custom message to meet your needs. It is a good practice to put all your custom messages in a separate package. This way, if somebody wants to use your custom message without running the main package you're using, they will not need to include the whole package, and just the one containing new message format.

Ok, so let's create a new package for our messages. It will use a part of geometry_msgs, so it is added as a dependency.

```
cd ~/catkin_ws/src
catkin_create_pkg inf3480_msgs geometry_msgs
cd inf3480_msgs
```

In ROS, the messages are supposed to be in `msg` directory of the package. Let's create one.

```
mkdir msg
cd msg
```

And now inside the directory we can create a new custom message. Let's say we want to create a message defining a robot name and it's coordinates defined as `Point` from `geometry_msgs`.

```
code robotPos.msg
```

And inside the file let's define a variable to hold the name of the robot as well as a variable for position

```
string name
geometry_msgs/Point pos
```

Save the file and close it. Now we have to define parameters for compilation of the message. First, let's modify `package.xml` file.

```
cd ..
code package.xml
```

There we have to uncomment the following two lines.

```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

Save and close the file. Next is `CMakeLists.txt` file. There we have to make sure the following are made to enable message generation.

```
find_package(catkin REQUIRED COMPONENTS
   ...
   message_generation
)
```
and
```
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```

Then uncomment the lines to enable message generation
```
add_message_files(
  FILES
  robotPos.msg
)
```
and
```
generate_messages(
  DEPENDENCIES
  geometry_msgs
)
```

Great! Now we're ready to compile our new package containing the newly created message.

```
cd ../..
catkin_make
```

If compilation was successful, you can see if ROS finds your new message.

```
rosmsg info inf3480_msgs/robotPos
```

You should get the following output

```
string name
geometry_msgs/Point pos
  float64 x
  float64 y
  float64 z
```

Yes! We got it, congrats! Well... I guess if we created it, we should use it!

## Modifying our previous published to use the new message format

Let's simply take our previously written publisher node and use our new message format.

```
cd src/inf3480/scripts
code coords_publisher.py
```

There we want to import our new message format

```
from inf3480_msgs.msg import robotPos
```

Then define for the publisher that a new message type will be used

```
pub = rospy.Publisher('/coords', robotPos, queue_size=10)
```

Followed by the modifications to the creation of the message

```
position = robotPos()
position.name = "Our Awesome Robot"
position.pos.x = 10
position.pos.y = 5
position.pos.z = 1
```

Save the file and close it. Now just the last modifications to the package configuration files to add the dependency on our new messages package.

```
roscd inf3480
code package.xml
```

And add the following lines

```
<build_depend>inf3480_msgs</build_depend>
<build_export_depend>inf3480_msgs</build_export_depend>
<exec_depend>inf3480_msgs</exec_depend>
```

And in the `CMakeLists.txt` modify to have the following:

```
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  inf3480_msgs
```

Looks like we're done! Let's recompile the package to make sure the new dependencies are taken into consideration and launch the publisher.

```
cd ~/catkin_ws
catkin_make
roslaunch inf3480 start_publisher.launch
```

Here we go! :)

## Writing a subscriber

Now let's write a subscriber node to listen to our published messages. For that, we need to create a new python script.
```
roscd inf3480
cd scripts
code coords_listener.py
```

And copy the following code into the file. It simply subscribes to the defined topic and prints the result every time the message is received.

```
#!/usr/bin/env python
import rospy
from inf3480_msgs.msg import robotPos

def callback(data):
    rospy.loginfo("Heard a new message:")
    rospy.loginfo(data)

def listener():
    rospy.init_node('coords_listener', anonymous=True)
    rospy.Subscriber('/coords', robotPos, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```

Make it executable

```
chmod +x coords_listener.py
```

Let's test it! First launch the publisher and then `rosrun` the listener.

```
roslaunch inf3480 start_publisher.launch
```

and in the other Terminal window:

```
rosrun inf3480 coords_listener.py
```

Here we go, we have established communication between the two nodes.

## Modifying the launch file

As the last thing, let's include the new listener node in our launch file, so everything can be started with a single command. It can be done by simply adding one more line to the existing launch file.

```
roscd inf3480
cd launch
code start_publisher.launch
```

And add the following

```
<node pkg="inf3480" name="coords_subscriber_node" type="coords_listener.py" output="screen"/>
```

Try it now:

```
roslaunch inf3480 start_publisher.launch
```

Now you can see that it's printing double messages, one by publisher and one by listener. You can supress one of them by modifying the `output` parameter in the launch file from `screen` to `log`.

## RQT

We can also test a few visualisation tools. But first, published coordinates should have some kind of change over time. What if we put sin and cos around them to change with time. Modify the publisher file

```
roscd inf3480
cd scripts
code coords_publisher.py
```

And then import `math` module:

```
import math
```

And add the update according to the counter to x and y coordinates

```
ctr = 0
while not rospy.is_shutdown():
   position.pos.x = math.cos(ctr)
   position.pos.y = math.sin(ctr)
   ctr += 0.2
   ...
```

Now the values should be changing as the while loop iterates and increases our counter. You can verify it by launching the node. On the other hand, we can plot the vairables in realtime using rqt:

```
rqt_plot
```

Also, we can observe how nodes interact by checking `rqt_graph`

```
rqt_graph
```

These tools become very useful as the system grows with many topics and nodes interacting with each other!

## Recording and playing back the data - rosbag

The last thing we will take a look at is how to record the data and play it back for later use. This is crucial when you work with real system and want to test algorithms on exactly the same conditions. It's almost impossible to replicate identical conditions when working with real sensors. Furthermore, it allows you to share the data you are working on with your colleagues or even place them as public datasets.

ROS has a tool called `rosbag` to record the data. You can either record everything that is published on the system, or specifically select the topics that you are interested in. Now we will record our known topic `/coords`. First, launch the publisher node:

```
roslaunch inf3480 start_publisher.launch
```
And in another Terminal window go to the folder where you want to record and start recording

```
rosbag record /coords
```

Then you can stop it by simply pressing `Ctrl+C`. Now we can see the information about our bag file:

```
rosbag info filename.bag
```

When you want to playback the data, simply use

```
rosbag play filename.bag
```

You can use `rqt_plot` to see that the data is straming. There are options to loop the recording, change the playback speed, pause, etc. These options can be found here:
http://wiki.ros.org/rosbag/Commandline

# Well done and thanks! :-)

Great! We're done for today and with ROS lectures this semester! Hopefully you've learned just enough to get started with ROS and to get you excited about this system. Below are some links and additional information that will help you further understand and learn ROS.

# Additional Official ROS Tutorials

If you plan to learn ROS in depth, I recommend to go through all the official ROS Tutorials:
http://wiki.ros.org/ROS/Tutorials

## Relevant for INF3480/4380 course
These tutorials are relevant to your work in INF3480/4380 course and will give you more in-depth understanding

- Installing and Configuring ROS Environment: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
- Navigating the Filesystem: http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem
- Creating Package: http://wiki.ros.org/ROS/Tutorials/CreatingPackage
- Building Packages: http://wiki.ros.org/ROS/Tutorials/BuildingPackages
- Understanding Nodes: http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes
- Understating Topics: http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics
- Writing Publisher and Subscriber in Python: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

## Learning ROS in-depth for Future Projects and Work

If you decide to go all-in into ROS and want to get a full understanding and a lot of hands-on experience, I highly recommend buying and going through the online courses provided by The Construct Sim. They are not affiliated with University of Oslo, neither am I affiliated with them, but I found their courses to be a very effective way to get a lot of experience in ROS with a really nice web browser based interface. Unfortunately the courses are not free of charge.
http://www.theconstructsim.com/
