/**
* Hello reader! This code has been made by three students:
* Johhni Nielsen, Hugo Karl Markoff and Jevgenijs Galaktionovs.
* Descriptive comments are placed to be ALWAYS on top of the described comment.
* Repeating code actions are left uncommented,
* such as similar functions and 'switch' cases after 1 .
* Code lines related to ROS were described using "ROS WIKI" as the source.
*/

/**
 * "ros/ros.h" is required to use ROS commands.
 * "std_msgs/String.h" is a primitive data type used in ROS Messages.
 * <sstream> enables usage of stringstreams.
 * <iostream> activates standard input/output streams, such as cin and cout.
 * <limits> is used to include numeric limits for the
 * cin.ignore( numeric_limits<std::streamsize>::max(), '\n') line.
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <limits>

using namespace std;

/**
 * The next four represented functions only differ with mathematical signs.
 * Functions import the 'a' and 'b' integers, calculate them and return
 * the result.
 */
double add (double a, double b)
{
	double r;
	r = a + b;
	return r;
}

double subtract (double a, double b)
{
	double r;
	r = a - b;
	return r;
}

double multiply (double a, double b)
{
	double r;
	r = a * b;
	return r;
}

double divide (double a, double b)
{
	double r;
	r = a / b;
	return r;
}

int main(int argc, char **argv)
{
	/**
	 * The next code line initializes our node. By using (argc,argv,...) we allow
	 * ROS to remap it's command-line arguments.
	 * The third argument of init() is declaration of our new node name.
	 * You must call one of ros::init() versions before using any other part of the ROS system.
	 */
	ros::init(argc, argv, "calculator");

	/**
	 * NodeHandle provides startup and shutdown of the internal node in roscpp program.
	 */
	ros::NodeHandle n;

	/**
     * Advertise function shows, which message type we want to publish.
     * Also, this function calls ROS Master and notifies it about who
     * is publisher and who is a subscriber. After advertise() is completed, ROS
     * Master notifies other nodes that are trying to connect to our "topic".
     * First parameter in advertise() declares a name for a new topic.
     * Second parameter is the amount of messages that are saved in buffer memory.
	 */
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 10);

	double a = 0;
	double b = 0;
	int inputInt = 0;

	char tryagain;

	/**
	 * Next two lines create 'std_msgs::String' called 'msg' and
	 * 'stringstream' called 'ss', which are used for publishing to
	 * the topic.
	 */
	std_msgs::String msg;
    std::stringstream ss;


	/**
	 * This 'do-while' statement contains most of the code which is visible
	 * to the user. It loops as long as the 'tryagain' string is 'Y' or 'y'.
	 */
	do
	{
		/**
		 * This 'for' statement creates an integer 'count' and sets
		 * it's value to '1'. Checks that 'count' value is larger
		 * than zero in order to run. Then, pre-increment '++cout' increases
		 * value by one after each run of our "for" loop.
		 */
		for (int count = 1; count > 0; ++count)
		{
			/**
			 * The 'system("clear")' command allows us to run the terminal
			 * command 'clear' which clears the screen of all text.
			 * It has been made with purpose to always bring menu on top of the terminal.
			 */
			system("clear");

			/**
			 * Here we print the menu, so the user has a
			 * easy to understand GUI.
			 */
			cout << endl;
			cout << "<<<<<<<<<<<<<<<<<<<<<< MENU >>>>>>>>>>>>>>>>>>>>>>" << endl;
			cout << "<                                                >" << endl;
			cout << "<   Press 1 and ENTER to add two numbers.        >" << endl;
			cout << "<                                                >" << endl;
			cout << "<   Press 2 and ENTER to subtract two numbers.   >" << endl;
			cout << "<                                                >" << endl;
			cout << "<   Press 3 and ENTER to multiply two numbers.   >" << endl;
			cout << "<                                                >" << endl;
			cout << "<   Press 4 and ENTER to divide two numbers.     >" << endl;
			cout << "<                                                >" << endl;
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>" << endl;
			cout << "<                                                >" << endl;
			cout << "<   Press 5 and ENTER to quit.                   >" << endl;
			cout << "<                                                >" << endl;
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>" << endl;
			cout << endl;

			/**
			 * This while loop runs as long as the 'cin' input to the
			 * integer 'inputInt' is a valid input.
			 * 'inputInt' is an integer and will only accept numbers. Loop will
			 * keep running if the user inputs another type than number, (i.e. characters).
			 */
			while(!(cin >> inputInt))
			{
				/**
				 * The cin.clear() clears the error flag on cin (so that future
				 * I/O operations will work correctly).
				 */
				cin.clear();

				/**
				 * The 'cin.ignore( numeric_limits<std::streamsize>::max(), '\n')'
				 * line ignores the 'cin' input if the user inputs anything other
				 * than a number. It prints the last 'cout' and sends the user back
				 * to the 'cin'.
				 */
				cin.ignore( numeric_limits<std::streamsize>::max(), '\n');
				cout << "That was not a number. Please choose a number between 1-5: " << endl;
			}

			/**
			 * Here we start the 'switch' statement which uses
			 * 'inputInt' integer to select which case to run.
			 */
			switch(inputInt)
			{
				/**
				 * The first 4 cases only differ by mathematical
				 * operators (+, -, *, /).
				 */
				case 1:

				/**
				 * Here we ask the user to type two numbers they
				 * want to calculate, put these numbers into the
				 * integers 'a' and 'b'.
				 */
				cout << "You chose addition:" << endl;
				cout << endl;
				cout << "Type the first number:  ";
				cin >> a;
				cout << "                        +" << endl;
				cout << "Type the second number: ";
				cin >> b;

				/**
				 * ss.str(""); line clears the content of the stringstream 'ss'.
				 */
				ss.str("");

				/**
				 * The next line combines content of the integer 'a' with a
				 * '+' sign, then 'b' followed by a '=' sign, then the
				 * value returned from the function 'add ()' and puts all
				 * that data into the stringstream 'ss' in correct order.
				 */
				ss << a << " + " << b << " = " << add (a,b);

				/**
				 * This is a message object. Here, we specifying which message data will be sent to the topic.
				 */
				msg.data = ss.str();

				/**
				 * The publish() function is a method to send a message. Parameter is the object
                 * declared in previous code line. Object type must match the one which was
                 * stated in advertise<>() function.
				 */
				chatter_pub.publish(msg);

				/**
				 * Next code lines are again clearing the stringstream 'ss' content, puts
				 * the amount of "calculations" tries that user already did into the
				 * stringstream 'ss' and again sends the data to the topic.
				 */
				ss.str("");
				ss << "You have made " << count << " calculations." << endl << endl;
				msg.data = ss.str();
				chatter_pub.publish(msg);

				/**
				 * Here we notify the user that calculation process has been completed.
				 * Result has been published to the "chatter' topic. Next, program prompts user
				 * whether they want to try again or quit.
				 * Accordingly, it puts user's answer into the char 'tryagain'.
				 */
				cout << endl;
				cout << "The resuĺt of " << a << " + " << b << " has been calculated." << endl;
				cout << endl;
				cout << "Do you want to try again? [Y / N]" << endl;
				cin >> tryagain;

				/**
				 * 'break' is needed to end the case. Otherwise, the program will run through
				 * until next 'break' or 'default case' is found.
				 */
				break;

				case 2:
				cout << "You chose subtraction:" << endl;
				cout << endl;
				cout << "Type the first number:  ";
				cin >> a;
				cout << "                        -" << endl;
				cout << "Type the second number: ";
				cin >> b;

				ss.str("");
				ss << a << " - " << b << " = " << subtract (a,b);
				msg.data = ss.str();
				chatter_pub.publish(msg);

				ss.str("");
				ss << "You have made " << count << " calculations." << endl << endl;
				msg.data = ss.str();
				chatter_pub.publish(msg);

				cout << endl;
				cout << "The resuĺt of " << a << " - " << b << " has been calculated." << endl;
				cout << endl;
				cout << "Do you want to try again? [Y / N]" << endl;
				cin >> tryagain;
				break;

				case 3:
				cout << "You chose multiplication:" << endl;
				cout << endl;
				cout << "Type the first number:  ";
				cin >> a;
				cout << "                        x" << endl;
				cout << "Type the second number: ";
				cin >> b;

				ss.str("");
				ss << a << " x " << b << " = " << multiply (a,b);
				msg.data = ss.str();
				chatter_pub.publish(msg);

				ss.str("");
				ss << "You have made " << count << " calculations." << endl << endl;
				msg.data = ss.str();
				chatter_pub.publish(msg);

				cout << endl;
				cout << "The resuĺt of " << a << " x " << b << " has been calculated." << endl;
				cout << endl;
				cout << "Do you want to try again? [Y / N]" << endl;
				cin >> tryagain;
				break;

				case 4:
				cout << "You chose division:" << endl;
				cout << endl;
				cout << "Type the first number:  ";
				cin >> a;
				cout << "                        /" << endl;
				cout << "Type the second number: ";
				cin >> b;

				ss.str("");
				ss << a << " / " << b << " = " << divide (a,b);
				msg.data = ss.str();
				chatter_pub.publish(msg);

				ss.str("");
				ss << "You have made " << count << " calculations." << endl << endl;
				msg.data = ss.str();
				chatter_pub.publish(msg);

				cout << endl;
				cout << "The resuĺt of " << a << " / " << b << " has been calculated." << endl;
				cout << endl;
				cout << "Do you want to try again? [Y / N]" << endl;
				cin >> tryagain;
				break;

				/**
				 * Case '5' ends the program.
				 */
				case 5:
				cout << endl << "Ending program." << endl << endl;
				return 0;
				break;

				/**
				 * Here we set up the default case. The switch runs this case if
				 * the integer 'inputInt' is not 1, 2, 3, 4 or 5.
				 */
				default:

				/**
				 * We ask the user if they want to try again or end the program.
				 */
				cout << endl << "Please choose a number between 1-5. " << endl << endl;
				cout << "Do you want to try again? [Y / N]" << endl;
				cin >> tryagain;
			}

			/**
			 * This last if-statement checks if the user typed 'N' or 'n' in the
			 * 'tryagain' string. If so, it ends the program.
			 */
			if (tryagain == 'N' || tryagain == 'n')
			{
				return 0;
			}
		}
	}while( tryagain == 'Y'|| tryagain == 'y' );

return 0;
}
