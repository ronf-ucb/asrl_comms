#include <serial/SerialNode.hpp>


int main(int argc, char* argv[])
{

  ros::init(argc, argv, "serial_node");
  ros::NodeHandle nh;

  try
    {
      asrl::serial::SerialNode s;
    
      s.spin();

    }
  catch (std::exception& e)
    {
      ROS_ERROR_STREAM("Exception: " << e.what());
    }

  return 0;
}
