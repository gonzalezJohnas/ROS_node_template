#include "node_example_core.h"

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "node_example");
  ros::NodeHandle n("~");

  // Create a new NodeExample object.
  NodeExample *node_example = new NodeExample(n);

  ros::spin();

  return 0;
} // end main()