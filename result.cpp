/**
* Hello reader! This code has been made by four students:
* Johhni Nielsen, Hugo Karl Markoff and Jevgenijs Galaktionovs.
* Descriptive comments are placed to be ALWAYS on top of the described comment.
* Code lines described in "calculator.cpp" are not commented in this code.
* Code lines related to ROS were described using "ROS WIKI" as the source.
*/

/**
 * "ros/ros.h" is required to use ROS commands.
 * "std_msgs/String.h" is a primitive data type used in ROS Messages.
 * <iostream> activates standard input/output streams, such as cin and cout.
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

/**
 * This function is doing a return call . Function will be called each time new messages
 * are  published on a subscribed topic.
 * %s is a format specifier for string variable input or output.
 * Function saves messages into the boost_shared_ptr, which allows user
 * to store messages.
 * The 'ROS_INFO' function prints the string from the topic to the terminal
 * so the user can see the calculated results.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("%s", msg->data.c_str());
}

int main(int argc, char **argv)
{
	/**
	* The 'system("clear")' command allows us to run the terminal
	* command 'clear' which clears the screen of all text.
	* It has been made with purpose to always bring the calcutaled
	* results on top of the terminal.
	*/
	system("clear");

  	/**
  	 * Here we print the text 'The calculated results:' to the user,
  	 */
  	std::cout << "The calculated results:" << std::endl << std::endl;

	/**
	 * The next code line initializes our node. By using (argc,argv,...) we allow
	 * ROS to remap it's command-line arguments.
	 * The third argument of init() is declaration of our new node name.
	 * You must call one of ros::init() versions before using any other part of the ROS system.
	 */
	ros::init(argc, argv, "results");

	/**
	 * NodeHandle provides startup and shutdown of the internal node in roscpp program.
	 */
	ros::NodeHandle n;

	/**
	 * The subscribe() tells ROS that we want to receive messages from "chatter" topic.
	 * Same as in publisher(), it lets ROS Master to know current situation.
	 * Messages are passed to a callback function "chatterCallback".
	 */
	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

	/**
	 * ros::spin() goes into loop, updating callback function messages.
	 */
	ros::spin();

	return 0;
}

