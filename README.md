## <a name="_2yi04gfqht5w"></a>**ros_turtlebots_control package ReadMe**
## <a name="_b47vbfll1cm0"></a>**Overview**
This package provides four ROS nodes for controlling the movement of three turtles in a turtlesim environment. The spawn.py spawns the turtle and create three turtles in the simulator. The goal.py node allows the user to move a turtle to a specified goal position by entering the turtle name and the x, and y coordinates of the goal. The teleop.py node enables the user to control the movement of one or all turtles through the keyboard by entering the turtle name and direction. The sweep.py moves the three turtles randomly until they sweep over the whole simulator while also attempting collision avoidance.

A video demonstration can be found on <https://www.youtube.com/watch?v=hopWpQZX5a4>



## <a name="_4du40j6fknpl"></a>**Files**
The package consists of the following files:

- sweep.launch: A launch file that launches the Turtlesim simulator, spawns the turtles, and runs the sweep.py script.
- goal.launch: A launch file that launches the Turtlesim simulator, spawns the turtles, and runs the goal.py script.
- teleop.launch: A launch file that launches the Turtlesim simulator, spawns the turtles, and runs the teleop.py script.
- spawn.py: A Python script that spawns the three turtles in the Turtlesim simulator.
- goal.py: A Python script that moves the turtles to desired goal locations.
- teleop.py: A Python script that moves the turtles according to commands from the keyboard.
- sweep.py: A Python script that moves the turtles around until they clear up the whole turtlesim environment while avoiding collision with themselves and the wall.

## <a name="_uxro1auaoyfu"></a>**Requirements**
- Ubuntu 18.04 or later
- ROS
- turtlesim package

## <a name="_xbzmm5pjakai"></a>**Installation**
1. Install ROS and turtlesim package. Follow the instructions[ ](http://wiki.ros.org/ROS/Installation)[here](http://wiki.ros.org/ROS/Installation) and[ ](http://wiki.ros.org/turtlesim)[here](http://wiki.ros.org/turtlesim).
1. Clone the robot\_pkg package into your ROS workspace:

\>>bash

\>>cd <ros\_workspace>/src

\>>git clone https://github.com/soloskino1/ros_turtlebots_control.git

3. Build the package:

\>>bash

\>>cd <robot\_pkg>

\>>catkin\_make

3. Make the launch files and the nodes executable

\>> chmod +x \*

## <a name="_vqefmxv875sq"></a>**Nodes**
### <a name="_gh44rl4ffdq1"></a>**goal.py**
This is a simple ROS node that makes use of the robot\_pkg package to move a turtle to a desired location. It does so by calculating the distance to travel and the angle between the turtle's heading and the desired direction. It then sets angular and linear velocity commands to reach the goal. The turtle will move towards the goal position until it reaches within a threshold distance of 0.1 units.

## <a name="_a7rz9ap49y2s"></a>**Running the Node**
To run the node, first, make sure that your ROS workspace is set up correctly. You can then use the following command to run the launch file (goal.launch):

\>>roslaunch robot\_pkg goal.launch

The node will ask you to enter the turtle name (a, b, or c) and the coordinates of the desired location. The turtle will move to the desired location, and the node will wait for further inputs.

This node moves a turtle to a specified goal position. The node prompts the user to enter the turtle name and the x, and y coordinates of the goal. The turtle will move towards the goal position until it reaches a threshold distance of 0.1 unit.

## <a name="_aqmty3nfyrvy"></a>**Node Structure**
The goal.py node makes use of the following ROS packages and messages:

- rospy: ROS Python client library
- math: Python math library
- geometry\_msgs.msg.Twist: Message type to send velocity commands
- turtlesim.srv.SetPen: Service type to set the pen
- turtlesim.msg.Pose: Message type to get the current position of the turtle

The node has the following functions:

### <a name="_do46pplh5iu5"></a>**move\_turtle:**
This function moves the turtle to a given location. It takes the following parameters:

- turtle\_name: The name of the turtle to move (string)
- x: The x-coordinate of the desired location (float)
- y: The y-coordinate of the desired location (float)
- count: The number of times this function has been called (integer)

The function starts by setting the pen down using the SetPen service. It then creates a publisher object to send velocity commands and sets the rate of publishing commands. The current position of the turtle is obtained using the Pose message. The distance to travel is then calculated, and a Twist message object is created.

The function then enters a loop to continuously publish velocity commands until the goal is reached. Each iteration calculates the angle between the turtle's heading and the desired direction. The angular and linear velocity commands are set using this angle, and the Twist message is published. The current position of the turtle is updated, and the distance to the goal is recalculated.

The function stops the turtle when the goal is reached by setting the linear and angular velocity commands to 0.
### <a name="_uizposocydf9"></a>**main:**
This function initializes the node and calls the move\_turtle function. It takes no parameters. The function asks the user to enter the name of the turtle and the coordinates of the desired location. The move\_turtle function is called with these parameters, and the turtle moves to the desired location. The function then waits for further inputs from the user through the terminal.
#### <a name="_elcwxd9jyq07"></a>**Subscribed Topics**
- /turtle/pose ([turtlesim/Pose](http://docs.ros.org/en/api/turtlesim/html/msg/Pose.html))
- The current position of the turtle.
#### <a name="_aa7lyjn9hiaf"></a>**Published Topics**
- /turtle/cmd\_vel ([geometry_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html))
- The velocity commands that move the turtle towards the goal.
#### <a name="_1tn5zmjagyo4"></a>**Services**
- /turtle/set\_pen ([turtlesim/SetPen]( h ))
- Sets the pen to a certain color and width to draw the turtle's path.

## **Rqt graphs**

**Rqt graphs:**

![C:\Users\SOLOMON\Downloads\mobile robotics\mobile robotics\Pictures\goal.png](Aspose.Words.5c5b016e-6673-4578-8f1f-3fd708991615.001.png)**


![C:\Users\SOLOMON\Downloads\mobile robotics\mobile robotics\goaooall.png](Aspose.Words.5c5b016e-6673-4578-8f1f-3fd708991615.002.png)



### <a name="_o2u0mzlki8pr"></a>**teleop.py**
This node allows the user to control the movement of a turtle through the keyboard. The node prompts the user to enter the turtle's name. Once the turtle is selected, the node prints the instructions for controlling the turtle through the keyboard. 

## <a name="_evm4lm9qbnxj"></a>**Running the Node**
To run the node, first, make sure that your ROS workspace is set up correctly. You can then use the following command to run the launch file (teleop.launch):

\>>roslaunch robot\_pkg teleop.launch

The node will ask you to enter the specific direction it wants the specified turtle to move. The turtle will move to the desired location, and the node will wait for further inputs.

## <a name="_m081ceh7i5y"></a>**Controls**
The following keys control the turtle:

- t: Move up
- v: Move down
- f: Move left
- g: Move right
- y: Move clockwise
- r: Move anti-clockwise
- i: Switch to turtle1
- o: Switch to turtle2
- p: Switch to  turtle3
- u: Control all turtles
- c: Stop

## <a name="_c09cjjl3vzf8"></a>**Node Structure**
The teleop.py node makes use of the same ROS packages and messages as goal.py

The node has the following functions:

**get\_key():** This function is used to get keyboard input from the user. It changes the settings of the terminal to raw mode and returns the first character read from the standard input.

**get\_position(turtle\_num):** This function is used to get the current position of the turtle specified by turtle\_num. It waits for a Pose message from the corresponding topic and returns the x and y coordinates of the turtle.

**keyboard\_input(turtle\_num):** This function is used to process the keyboard input and set the velocity of the turtle. It prints instructions for the user and waits for a key press. Based on the key pressed, it sets the linear and angular velocity of the turtle. It also sets the value of turtle\_num if the user wants to control a different turtle or all turtles.

**move\_turtle(turtle\_num, velocity\_msg):** This function is used to move the turtle specified by turtle\_num with the velocity specified by velocity\_msg. It checks the current position of the turtle and changes the direction of the velocity if the turtle has reached the edge of the screen.

**main():** This function is the main function of the script. It initializes the node and creates publishers for each turtle. It also sets the rate at which the loop runs. In each iteration of the loop, it calls the keyboard\_input and move\_turtle functions to process the user input and move the turtle.

## <a name="_t52sw3d29fix"></a>**Notes**
- By default, the node controls turtle1.
- The node can control up to three turtles simultaneously (turtle1, turtle2, turtle3).
- When controlling all turtles, the same commands are sent to each turtle.
- The turtles will bounce off the walls of the Turtlesim window.
- To exit the node, press any invalid key or use Ctrl-C.
- This node uses the termios and tty modules to capture keyboard input, and may not work correctly on all systems. If you experience issues with keyboard input, try running the node in a different terminal window or on another system.
#### <a name="_ae85nbutjzf5"></a>**Subscribed Topics**
- /turtle/pose ([turtlesim/Pose](http://docs.ros.org/en/api/turtlesim/html/msg/Pose.html))
- The current position of the turtle.
#### <a name="_5ig9zxdmxzvx"></a>**Published Topics**
- /turtle/cmd\_vel ([geometry\_msgs/Twist](<http://docs.ros.org/en/api/>





## **Rqt graphs**

![C:\Users\SOLOMON\Downloads\mobile robotics\mobile robotics\Pictures\teleop.png](Aspose.Words.5c5b016e-6673-4578-8f1f-3fd708991615.003.png)

![C:\Users\SOLOMON\Downloads\mobile robotics\mobile robotics\Pictures\teleop2.png](Aspose.Words.5c5b016e-6673-4578-8f1f-3fd708991615.004.png)

### <a name="_4lgs9sj451si"></a>**sweep.py**
This node controls three turtles in the simulated environment. The turtles move randomly and try to sweep around the simulator environment while performing the vacuum operation and avoiding collisions with other turtles and walls.

## <a name="_4559kisybg8n"></a>**Running the Node**
To run the node, first, make sure that your ROS workspace is set up correctly. You can then use the following command to run the launch file (sweep.launch):

\>>roslaunch robot\_pkg sweep.launch

The node will start moving randomly according to the random\_motion\_with\_wall\_avoidance function until the terminal is closed.


## <a name="_hatb4rg5b5ew"></a>**Node Structure**
The node has the following functions:
### <a name="_429zc857b5vy"></a>**turtle\_pose\_callback(pose, turtle\_id)**
This function is called every time a turtle's pose (position and orientation) is published. It updates the global x, y, and theta arrays with the turtle's current position and orientation.
### <a name="_wx0u4780qpsy"></a>**random\_motion\_with\_wall\_avoidance(turtle\_id)**
This is the main function that controls the random motion of a turtle. It takes a turtle ID as input and uses it to identify the turtle's topic and service names. The function subscribes to the turtle's pose topic and publishes to its command velocity topic. The turtle's pen is also set to a thickness of 30.

The function then enters a loop where it checks for collisions with other turtles and walls. If a collision is detected with another turtle, the turtle turns away from the other turtle. If the turtle is close to a wall, it turns away from the wall. Otherwise, the turtle moves randomly with a combination of linear and angular velocities.

The function runs continuously until the node is shut down. The image below show an example of the code being ran after about 3-4 minutes

![C:\Users\SOLOMON\Downloads\mobile robotics\mobile robotics\Pictures\sweep6.png](Aspose.Words.5c5b016e-6673-4578-8f1f-3fd708991615.005.png)
## <a name="_ae7t9t8ienti"></a>**Notes**

- By default, the node controls turtle1.
- The node can control up to three turtles simultaneously (turtle1, turtle2, turtle3).
- When controlling all turtles, the same commands are sent to each turtle.
- The turtles will bounce off the walls of the Turtlesim window.
- The function checks if the turtle is close to any of the walls or other turtles, and adjusts its movement accordingly to avoid collisions.
- The code uses lambda functions to pass additional arguments to the random\_motion\_with\_wall\_avoidance function so that it can distinguish between the three turtles.
- The code also defines a rospy.on\_shutdown function that is called when the node is shut down and prints a message to the console to indicate that the node has been shut down.
## **Rqt graphs**

![C:\Users\SOLOMON\Downloads\mobile robotics\mobile robotics\Pictures\sweep.png](Aspose.Words.5c5b016e-6673-4578-8f1f-3fd708991615.006.png)

![C:\Users\SOLOMON\Downloads\mobile robotics\mobile robotics\Pictures\Screenshot from 2023-04-28 05-40-48.png](Aspose.Words.5c5b016e-6673-4578-8f1f-3fd708991615.007.png)
