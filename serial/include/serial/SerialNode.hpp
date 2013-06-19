#ifndef ASRL_UDP_HPP
#define ASRL_UDP_HPP

//////////////////////////////////
// ROS headers
#include <ros/ros.h>

// Message types
#include <asrl_sensor_msgs/SerialData.h>

// Exception macros
#include <asrl/assert_macros.hpp>

// ASIO headers for UDP communication with RobuROC6
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

// C++ headers
#include <sstream>

//services
#include <serial/Baud.h>

namespace asrl { namespace serial {

    ASRL_DEFINE_EXCEPTION(Exception,std::runtime_error);

  class SerialNode
  {
  public:
    SerialNode();
    ~SerialNode();

    void spin();

  private:
    void startSerialReceive();
    void processIncomingSerialMessages(const boost::system::error_code& error, size_t bytes_recvd);
    void handleSendErrors( const boost::system::error_code& error, size_t bytes_sent);
    void processOutgoingSerialMessages(const asrl_sensor_msgs::SerialData::ConstPtr & packet);
    bool changeBaudRate(::serial::Baud::Request& req, ::serial::Baud::Response& resp);

    boost::asio::io_service ioService_;
    std::auto_ptr<boost::asio::serial_port> serialPort_;
    std::vector<boost::uint8_t> data_;

    ros::Publisher  publisher_;
    ros::Subscriber subscriber_;
    
    ros::ServiceServer changeBaudRate_;

    bool echo_;
  };

  }} // namespace asrl::serial

#endif

