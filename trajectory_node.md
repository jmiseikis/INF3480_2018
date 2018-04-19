# INF3480 Trajectory Planner Walkthrough
To motivate the reason for using ROS in the course we have created a small path
planning node that you will complete and test both in simulation and on the real
robot! We will see how the same code can be used both for simulation (testing)
and operating the real-world equivalent without changing a single line of code.

## Skeleton
We will start by reviewing the skeleton code that you will fill in. The skeleton
is given below and you should add this to you package as `path_planner.py`.

```python
#!/usr/bin/env python

"""
This node is designed to take in a circle drawing description and perform
the necessary calculations and commands to draw the circle using the
Crustcrawler platform
"""

from __future__ import print_function
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
import numpy as np
import rospy


def path_length(path):
    """
    Calculate path length in centimeters

    :param path: List of points
    :returns: Length of path in centimeters
    """
    length = 0.0
    for p1, p0 in zip(path[1:], path):
        length += np.linalg.norm(p1 - p0)
    return length


def inverse_kinematic(position):
    """
    Calculate the inverse kinematic of the Crustcrawler

    :param position: Desired end-point position
    :returns: Three element vector of joint angles
    """
    x, y, z = position
    l1 = 11.0
    l2 = 22.3
    l3 = 17.1
    l4 = 8.0
    l3_mark = np.sqrt(l4 ** 2 + l3 ** 2 + np.sqrt(2) / 2.0 * l4 * l3)
    phi = np.arccos((l3 ** 2 + l3_mark ** 2 - l4 ** 2)
                    / (2.0 * l4 * l3_mark))
    s = z - l1
    r = np.sqrt(x ** 2 + y ** 2)
    d = ((x ** 2 + y ** 2 + s ** 2 - l2 ** 2 - l3_mark ** 2)
         / (2. * l2 * l3_mark))

    theta1 = np.arctan2(y, x)
    theta3 = np.arctan2(-np.sqrt(1. - d ** 2), d)
    theta2 = np.arctan2(s, r) - np.arctan2(l3_mark * np.sin(theta3),
                                           l2 + l3_mark * np.cos(theta3))
    return np.array([theta1, theta2 * -1. + np.pi / 2., theta3 * -1.])


def create_trajectory_point(position, seconds):
    """
    Create a JointTrajectoryPoint

    :param position: Joint positions
    :param seconds: Time from start in seconds
    :returns: JointTrajectoryPoint
    """
    point = JointTrajectoryPoint()
    point.positions.extend(position)
    point.time_from_start = rospy.Duration(seconds)
    return point


def rotate_path(path, angle, axis):
    """
    Rotate all elements of a path by angle-axis rotation

    :param path: List of points
    :param angle: Angle in radians to rotate by
    :param axis: Unit vector to rotate around
    :returns: List of rotated points
    """
    # TODO: Implement as part 'c)'
    return path


def generate_path(origin, radius, num, angle, axis):
    """
    Generate path in 3D space of where to draw circle

    :param origin: 3D point of circle origin
    :param radius: Radius of circle in centimeters
    :param num: Number of points in circle
    :param angle: Angle to rotate circle by
    :param axis: Unit vector to rotate circle around
    :returns: List of points to draw
    """
    pass


def generate_movement(path):
    """
    Generate Crustcrawler arm movement through a message

    :param path: List of points to draw
    :returns: FollowJointTrajectoryGoal describing the arm movement
    """
    pass


def draw_circle(origin, radius, num, angle, axis):
    """
    Draw circle using Crustcrawler

    :param origin: 3D point of circle origin
    :param radius: Radius of circle in centimeters
    :param num: Number of points in circle
    :param angle: Angle to rotate circle by
    :param axis: Unit vector to rotate circle around
    """
    pass


if __name__ == '__main__':
    import argparse
    import sys
    # Create command line parser and add options:
    parser = argparse.ArgumentParser(
            description="CrustCrawler circle drawer TM(C), patent pending!",
            version="Spring 2018")
    parser.add_argument(
            '--origin', '-o', type=float, nargs=3,
            metavar=('x', 'y', 'z'), required=True,
            help="Origin of the board")
    parser.add_argument(
            '--radius', '-r', type=float, default=5.0,
            metavar='cm', help="The radius of the circle to draw")
    parser.add_argument(
            '--num_points', '-n', type=int,
            default=4, metavar='num',
            help="Number of points to use when drawing the circle")
    parser.add_argument(
            '--orientation', '-orient', type=float, default=0.0,
            metavar='degrees',
            help="Orientation of the board along the X-axis")
    args = parser.parse_args()
    # Ensure points are NumPy arrays
    args.origin = np.array(args.origin)
    orient = np.array([0, 1., 0])
    # Ensure that arguments are within legal limits:
    if 0.0 > args.orientation or args.orientation > 90.0:
        sys.exit("Orientation must be in range [0.0, 90.0], was: {:.1f}"
                 .format(args.orientation))
    if 3 >= args.num_points <= 101:
        sys.exit("Number of points must be in range [3, 101] was: {:d}"
                 .format(args.num_points))
    max_dist = np.linalg.norm(args.origin)
    if max_dist - args.radius < 20.0:
        sys.exit("Circle to close to the robot! Minimum: 40cm, was: {:.2f}"
                 .format(max_dist - args.radius))
    # Create ROS node
    rospy.init_node('circle_drawer', anonymous=True)
    # Call function to draw circle
    try:
        sys.exit(draw_circle(args.origin, args.radius, args.num_points,
                             np.deg2rad(args.orientation), orient))
    except rospy.ROSInterruptException:
        sys.exit("Program aborted during circle drawing")
```

The skeleton above represents a ROS node that is we will invoke using `rosrun`
passing in parameters to draw a circle. As you can see the code is not complete
and we need to fill in a few blank, the `pass`-es, spots.

### Skeleton explained
Lets review the skeleton in more detail before we move on. If you understand how
this code works you can quickly skip to the next part.

```python
#!/usr/bin/env python

"""
This node is designed to take in a circle drawing description and perform
the necessary calculations and commands to draw the circle using the
Crustcrawler platform
"""

from __future__ import print_function
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
import numpy as np
import rospy
```

The first part of any ROS python node should start with the
[shebang](https://en.wikipedia.org/wiki/Shebang_(Unix)) declaring the file to be
a python file. Next we have a short description of the functionality of the
node, this is not strictly necessary, but it is good form to include.

The last part is the most important and here we import the modules that are used
later in the program. You should recognize a lot of the package dependencies
here. In addition we have a few python imports, like `importy numpy as np`,
which is a nice-to-have addition.

```python
def path_length(path):
    """
    Calculate path length in centimeters

    :param path: List of points
    :returns: Length of path in centimeters
    """
    length = 0.0
    for p1, p0 in zip(path[1:], path):
        length += np.linalg.norm(p1 - p0)
    return length
```

The `path_length` function calculates the distance between all the points in a
given path. This is a helper function which we will utilize later.

```python
def inverse_kinematic(position):
    """
    Calculate the inverse kinematic of the Crustcrawler

    :param position: Desired end-point position
    :returns: Three element vector of joint angles
    """
    x, y, z = position
    l1 = 11.0
    l2 = 22.3
    l3 = 17.1
    l4 = 8.0
    l3_mark = np.sqrt(l4 ** 2 + l3 ** 2 + np.sqrt(2) / 2.0 * l4 * l3)
    phi = np.arccos((l3 ** 2 + l3_mark ** 2 - l4 ** 2)
                    / (2.0 * l4 * l3_mark))
    s = z - l1
    r = np.sqrt(x ** 2 + y ** 2)
    d = ((x ** 2 + y ** 2 + s ** 2 - l2 ** 2 - l3_mark ** 2)
         / (2. * l2 * l3_mark))

    theta1 = np.arctan2(y, x)
    theta3 = np.arctan2(-np.sqrt(1. - d ** 2), d)
    theta2 = np.arctan2(s, r) - np.arctan2(l3_mark * np.sin(theta3),
                                           l2 + l3_mark * np.cos(theta3))
    return np.array([theta1, theta2 * -1. + np.pi / 2., theta3 * -1.])
```

You should be quite familiar with the `inverse_kinematic` function which we will
use to calculate the joint positions to reach a given point.

```python
def create_trajectory_point(position, seconds):
    """
    Create a JointTrajectoryPoint

    :param position: Joint positions
    :param seconds: Time from start in seconds
    :returns: JointTrajectoryPoint
    """
    point = JointTrajectoryPoint()
    point.positions.extend(position)
    point.time_from_start = rospy.Duration(seconds)
    return point
```

The above function, `create_trajectory_point`, is a simple helper function. Since
messages in ROS are classes in python, which can be initialized with default
values, it often makes sense to create the default and only override what you
need. Which is what we have done above.

```python
def rotate_path(path, angle, axis):
    """
    Rotate all elements of a path by angle-axis rotation

    :param path: List of points
    :param angle: Angle in radians to rotate by
    :param axis: Unit vector to rotate around
    :returns: List of rotated points
    """
    # TODO: Implement as part 'c)'
    return path
```

This is one function that we will not give out. It is part `c)` of your
assignment and enables us to draw vertical circles if so desired. The stub given here
is the identity function (it does nothing returning unchanged the input).

```python
def generate_path(origin, radius, num, angle, axis):
    """
    Generate path in 3D space of where to draw circle

    :param origin: 3D point of circle origin
    :param radius: Radius of circle in centimeters
    :param num: Number of points in circle
    :param angle: Angle to rotate circle by
    :param axis: Unit vector to rotate circle around
    :returns: List of points to draw
    """
    pass


def generate_movement(path):
    """
    Generate Crustcrawler arm movement through a message

    :param path: List of points to draw
    :returns: FollowJointTrajectoryGoal describing the arm movement
    """
    pass


def draw_circle(origin, radius, num, angle, axis):
    """
    Draw circle using Crustcrawler

    :param origin: 3D point of circle origin
    :param radius: Radius of circle in centimeters
    :param num: Number of points in circle
    :param angle: Angle to rotate circle by
    :param axis: Unit vector to rotate circle around
    """
    pass
```

The next three methods are the ones we will implement in this assignment, and
they will be explained in the next part.

```python
if __name__ == '__main__':
    import argparse
    import sys
    # Create command line parser and add options:
    parser = argparse.ArgumentParser(
            description="CrustCrawler circle drawer TM(C), patent pending!",
            version="Spring 2018")
    parser.add_argument(
            '--origin', '-o', type=float, nargs=3,
            metavar=('x', 'y', 'z'), required=True,
            help="Origin of the board")
    parser.add_argument(
            '--radius', '-r', type=float, default=5.0,
            metavar='cm', help="The radius of the circle to draw")
    parser.add_argument(
            '--num_points', '-n', type=int,
            default=4, metavar='num',
            help="Number of points to use when drawing the circle")
    parser.add_argument(
            '--orientation', '-orient', type=float, default=0.0,
            metavar='degrees',
            help="Orientation of the board along the X-axis")
    args = parser.parse_args()
    # Ensure points are NumPy arrays
    args.origin = np.array(args.origin)
    orient = np.array([0, 1., 0])
    # Ensure that arguments are within legal limits:
    if 0.0 > args.orientation or args.orientation > 90.0:
        sys.exit("Orientation must be in range [0.0, 90.0], was: {:.1f}"
                 .format(args.orientation))
    if 3 >= args.num_points <= 101:
        sys.exit("Number of points must be in range [3, 101] was: {:d}"
                 .format(args.num_points))
    max_dist = np.linalg.norm(args.origin)
    if max_dist - args.radius < 20.0:
        sys.exit("Circle to close to the robot! Minimum: 40cm, was: {:.2f}"
                 .format(max_dist - args.radius))
    # Create ROS node
    rospy.init_node('circle_drawer', anonymous=True)
    # Call function to draw circle
    try:
        sys.exit(draw_circle(args.origin, args.radius, args.num_points,
                             np.deg2rad(args.orientation), orient))
    except rospy.ROSInterruptException:
        sys.exit("Program aborted during circle drawing")
```

The last part of the skeleton code is the main function of our python program.
This is the function that will be called when our ROS node is run. You don't
need to understand all the nuances of this function, it basically parses
command-line parameters, validates them and calls the method to draw the path.
(More info on python command-line parsing is
[here](https://docs.python.org/3.5/howto/argparse.html#id1) and ROS parameter handling
is [here](https://wiki.ros.org/rospy_tutorials/Tutorials/Parameters)).

## Generating a path
Let us first start by implementing a path generation function, `generate_path`.
The goal of this function is to create a list of points, `(x, y, z)`, that
together makes up the points of a circle. There are many ways we could do that,
but we are a bit limited by the input parameters. Since we are asked to output a
fixed number of points and knowing that the robot arm can only move linearly
between points we will have to assume that linear interpolation between points
are enough (we will see the effect of this decision later).

The first thing we need is to declare a list that we will put our points into:

```python
path = []
```

The next thing we need to calculate is the distance between each point. Since we
are given only a few of them.

```python
distance_between = (2.0 * np.pi) / float(num)
```

Next we will iterate the number of points (plus one to start and stop at the
same position).

```python
for i in range(num + 1):
    index = i * distance_between
    path.append(radius * np.array([np.cos(index), np.sin(index), 0.0]))
```

`index` together with `sin` and `cos`
tells us where on the unit circle we are and we can simply multiply with
the desired `radius` to create a circle of the desired size.

As you may notice, in the above code, we did not rotate nor care about the
desired origin of the circle. We will not explain rotation here (task `3c)`),
but we must do something about the origin.

```python
# Rotate using the rotation function
path = rotate_path(path, angle, axis)
# Add origin to path:
path = [p + origin for p in path]
return path
```

As you can see we simply iterate the list adding the origin to every point. This
works because we used a unit circle with a given radius (notice that we rotate
before we add the origin, why is that?). Lastly we return the
path which marks the end of this function!

## Generating arm movement
Now that we are able to generate a path we need to translate that path into
something that the Crustcrawler can perform. The following should be filled into
the method `generate_movement`. As before we will start by declaring our return
value and then fill that in.

```python
# Generate our goal message
movement = FollowJointTrajectoryGoal()
```

The way to interact with the Crustcrawler is to use the
[`FollowJointTrajectoryAction`](https://wiki.ros.org/joint_trajectory_controller?distro=lunar#Action_interface)
later we will see how to use this interface, but for now it is enough to know
that the input to that is a `FollowJointTrajectoryGoal`.

Next we will fill in a bit of fluff that is only really needed for bookkeeping.
The following describes the trajectory tolerances and which part of the vector
belongs to which joint. For ease of use we will enumerate the joints bottom to
top.

```python
# Names describes which joint is actuated by which element in the coming
# matrices
movement.trajectory.joint_names.extend(['joint_1', 'joint_2', 'joint_3'])
# Goal tolerance describes how much we allow the movement to deviate
# from true value at the end
movement.goal_tolerance.extend([
    JointTolerance('joint_1', 0.1, 0., 0.),
    JointTolerance('joint_2', 0.1, 0., 0.),
    JointTolerance('joint_3', 0.1, 0., 0.)])
# Goal time is how many seconds we allow the movement to take beyond
# what we define in the trajectory
movement.goal_time_tolerance = rospy.Duration(0.5)  # seconds
```

With that out of the way it is time to start generating movement! We will start
by setting the position of the arm to a stable starting point giving it plenty
of time to get there.

```python
time = 4.0  # Cumulative time since start in seconds
movement.trajectory.points.append(
    create_trajectory_point([0., 0., np.pi / 2.], time))
```

In the above we use our helper function `create_trajectory_point` to generate
the correct class and fill it in with the correct position. `time` is the
cumulative time since the start of the movement that the robot is allowed to use
for the movement. Since this is the first movement it is easy to see that we
give the arm `4.0` seconds to move to the desired initial position. To get a
feel for how the value behaves try changing it to less and more time and see
what happens!

Next we will start by moving the arm to the initial point of the circle. We will
also treat this point as special to avoid quick movement from the initial
resting position.

```python
# Add initial point, also as a large time fraction to avoid jerking
time += 4.0
movement.trajectory.points.append(
    create_trajectory_point(inverse_kinematic(path[0]), time))
```

As you can see we are now using the first point of the path to calculate joint
positions. Also notice that we accumulate time, this allows the path planning
algorithms in ROS to decide how to move the joints between to trajectory
positions.

Before we start adding the rest of the positions we need to decide how much time
we will allow the robot between each point. To do this we calculate the length
of the circle (from the path we generated) and setting a fixed speed that we
would like the arm to move with.

```python
# Calculate total circle length
length = path_length(path)
# Calculate how much time we have to process each point of the circle
time_delta = (length / 2.) / len(path)
```

The `2.` ensures that the arm moves between two points on the circle by about
`2.0 cm/s`. The next is to use this `time_delta` and add the rest of the points
in the path.

```python
for point in path[1:]:
    time += time_delta
    movement.trajectory.points.append(
	create_trajectory_point(inverse_kinematic(point), time))
```

Finally we will add the back the resting position and return the message.

```python
# Once drawing is done we add the default position
time += 4.0
movement.trajectory.points.append(
    create_trajectory_point([0., 0., np.pi / 2.], time))
return movement
```

## Drawing the circle
The last part of this task is simply gluing the above parts together into a
coherent whole.

```python
def draw_circle(origin, radius, num, angle, axis):
    """
    Draw circle using Crustcrawler

    :param origin: 3D point of circle origin
    :param radius: Radius of circle in centimeters
    :param num: Number of points in circle
    :param angle: Angle to rotate circle by
    :param axis: Unit vector to rotate circle around
    """
    # First start by creating action client, this is responsible for passing
    # our parameters and monitoring the Crustcrawler during operations
    client = actionlib.SimpleActionClient(
            '/crustcrawler/controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
    # Generate circle path
    path = generate_path(origin, radius, num, angle, axis)
    # Generate arm movement path
    goal = generate_movement(path)
    # Wait for arm to respond to action client
    client.wait_for_server()
    # Send goal
    client.send_goal(goal)
    # Wait for arm to perform our movement
    client.wait_for_result()
    # Finally print status of arm, did it work or not?
    result = client.get_result()
    if not result.error_code:
        print("Crustcrawler done!")
    else:
        print("Crustcrawler failed due to: '{!s}'({!s})"
              .format(result.error_string, result.error_code))
    return result.error_code
```

Here we use
[`actionlib`](https://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29)
to call and manage the arm movement combining the above to methods to create
and draw a circle.

## Testing
To test the above code we first need to start Gazebo with the Crustcrawler
inside. Make sure you have `crustcrawler_pen` in your workspace and simply run

```bash
$ roslaunch crustcrawler_pen_gazebo controller.launch control:=trajectory
```

To test our code we can run our node through `rosrun` as such

```bash
$ rosrun trajectory_assignment path_planner.py
```

Be sure to test different command line arguments to the above command. Now is
probably a good time to test different amount of points in our circle. With the
default `4` points can we really call it a circle? What happens if you double
the amount? Or quadruple `=O`...
