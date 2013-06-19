#include <tcp/TcpNode.hpp>
#include <asrl/rosutil/param.hpp>

#define MAX_PACKET_SIZE 65535

typedef boost::asio::ip::tcp asio_tcp;

namespace asrl {

  TcpNode::TcpNode(const ros::NodeHandle & nh) :
      nodeHandle_(nh),
      numSent_(0),
      numReceived_(0),
      connected_(false)
  {
    // Parameters
    rosutil::param<std::string>(nodeHandle_, "ip_address", ipAddressStr_, "192.168.0.75");
    rosutil::param<int>        (nodeHandle_, "port",       port_,         4000);

    // Set up socket
    endpoint_ = asio_tcp::endpoint(boost::asio::ip::address_v4::from_string(ipAddressStr_), port_);
    tcpSocket_.reset(new asio_tcp::socket(ioService_));

    ROS_INFO_STREAM("Connecting to " << ipAddressStr_ << ":" << port_ << "...");
    tcpSocket_->async_connect(endpoint_, boost::bind(&TcpNode::connect_handler,
                                                     this,
                                                     boost::asio::placeholders::error));

    /////////////////////////////////
    // boost::asio socket options:
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference.html

    // Socket option for the receive buffer size of a socket. 
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/socket_base/receive_buffer_size.html
    int receiveBufferSize = MAX_PACKET_SIZE;
    asrl::rosutil::param<int>(nodeHandle_, "receive_buffer_size", receiveBufferSize, MAX_PACKET_SIZE);
    boost::asio::socket_base::receive_buffer_size receiveBufferOption(receiveBufferSize);
    tcpSocket_->set_option(receiveBufferOption);

    // Socket option for the send buffer size of a socket. 
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/socket_base/send_buffer_size.html
    int sendBufferSize = MAX_PACKET_SIZE;
    asrl::rosutil::param<int>(nodeHandle_, "send_buffer_size", sendBufferSize, MAX_PACKET_SIZE);
    boost::asio::socket_base::send_buffer_size sendBufferOption(sendBufferSize);
    tcpSocket_->set_option(sendBufferOption);

    /*
    // Socket option to permit sending of broadcast messages.
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/socket_base/broadcast.html
    bool broadcast = false;
    asrl::rosutil::param<bool>(nodeHandle_, "broadcast", broadcast, false);
    boost::asio::socket_base::broadcast broadcastOption(broadcast);
    tcpSocket_->set_option(broadcastOption);
    
    // Socket option to report aborted connections on accept. 
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/socket_base/enable_connection_aborted.html
    bool enableAborted = false;
    asrl::rosutil::param<bool>(nodeHandle_, "enable_connection_aborted", enableAborted, false);
    boost::asio::socket_base::enable_connection_aborted enableAbortedOption(enableAborted);
    tcpSocket_->set_option(enableAbortedOption);

    // Socket option for the receive low watermark. 
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/socket_base/receive_low_watermark.html
    int receiveLowWatermark = 1;
    asrl::rosutil::param<int>(nodeHandle_, "receive_low_watermark", receiveLowWatermark, 1);
    boost::asio::socket_base::receive_low_watermark receiveLowWatermarkOption(receiveLowWatermark);
    tcpSocket_->set_option(receiveLowWatermarkOption);

    // Socket option to allow the socket to be bound to an address that is already in use. 
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/socket_base/reuse_address.html
    bool reuseAddress = false;
    asrl::rosutil::param<bool>(nodeHandle_, "reuse_address", reuseAddress, false);
    boost::asio::socket_base::reuse_address reuseAddressOption(reuseAddress);
    tcpSocket_->set_option(reuseAddressOption);

    // Socket option for the send low watermark.
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/socket_base/send_low_watermark.html
    int sendLowWatermark = 1;
    asrl::rosutil::param<int>(nodeHandle_, "send_low_watermark", sendLowWatermark, 1);
    boost::asio::socket_base::send_low_watermark sendLowWatermarkOption(sendLowWatermark);
    tcpSocket_->set_option(sendLowWatermarkOption);

    // Socket option to enable socket-level debugging. 
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/socket_base/debug.html
    bool socketDebug = false;
    asrl::rosutil::param<bool>(nodeHandle_, "socket_debug", socketDebug, false);
    boost::asio::socket_base::debug socketDebugOption(socketDebug);
    tcpSocket_->set_option(socketDebugOption);
    */

    int pubQueueDepth = 0;
    int subQueueDepth = 0;
    asrl::rosutil::param<int>(nodeHandle_, "in_queue_depth",  subQueueDepth, 100);
    asrl::rosutil::param<int>(nodeHandle_, "out_queue_depth", pubQueueDepth, 100);
    subscriber_ = nodeHandle_.subscribe("in/tcp", subQueueDepth, &TcpNode::packetMessageCallback, this);
    publisher_  = nodeHandle_.advertise<asrl_sensor_msgs::TcpPacket>("out/tcp", pubQueueDepth);

    // Initialize the data buffer. This is the theoretical maximum data size for a single tcp packet
    data_.resize(MAX_PACKET_SIZE);
  }


  TcpNode::~TcpNode()
  {
    // do nothing
  }


  void TcpNode::spin()
  {
    startTcpReceive();
    boost::thread tcpThread(boost::bind(&boost::asio::io_service::run, &ioService_));
    ///////////////////////////////////////////
    // Run ROS until shutdown.
    ros::spin();
    
    ///////////////////////////////////////////
    // Try a clean shutdown.
    // Stop the asio IO service.
    ROS_INFO("Stopping the IO service");
    ioService_.stop();
    
    // Join with the IO service thread
    ROS_INFO("Joining with the ASIO thread");
    tcpThread.join();
    
    // Ah...success.
    ROS_INFO("Join successful");
  }


  void TcpNode::startTcpReceive()
  {
    // Asynchronously receive tcp messages into data_, and handle them in processTcpBroadcastMessages()
    tcpSocket_->async_receive(boost::asio::buffer(&data_[0],data_.size()),
                              boost::bind(&TcpNode::processIncomingTcpMessages,
                                          this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred));
  }


  void TcpNode::processIncomingTcpMessages(const boost::system::error_code& error,
					   size_t bytes_recvd)
  {
    ros::Time stamp = ros::Time::now();
    numReceived_++;
    
    // TODO: Error handling. Diagnostics?
    if (!error)
    {
      boost::shared_ptr<asrl_sensor_msgs::TcpPacket> packet(new asrl_sensor_msgs::TcpPacket());

      // Fill in the timestamp
      packet->stamp = stamp;

      // Fill in the sequence number (i.e., the number received)
      packet->seqNumber = numReceived_;

      // Copy the data.
      packet->data.resize(bytes_recvd);
      if(bytes_recvd > 0)
        {
          memcpy(&packet->data[0],&data_[0],bytes_recvd);
        }
      // Publish the message
      publisher_.publish(packet);
    }
    else
    {
      ROS_ERROR("Error receiving TCP packet (%d): %s", error.value(), error.message().c_str());
    }

    // Re-registers the asynchronous receive.
    startTcpReceive();
  }


  void TcpNode::packetMessageCallback(const asrl_sensor_msgs::TcpPacket::ConstPtr & packet)
  {
    if (connected_)
    {
      processOutgoingTcpMessages(packet->data);
    }
    else
    {
      ROS_WARN("TCP node not yet connected. Please resend packet later.");
    }
  }


  void TcpNode::processOutgoingTcpMessages(std::vector<boost::uint8_t> const & data)
  {
    // Pass the message on to boost::asio
    tcpSocket_->async_send(boost::asio::buffer(&data[0],data.size()),
                           boost::bind(&TcpNode::handleSendErrors,
                                       this,
                                       boost::asio::placeholders::error,
                                       boost::asio::placeholders::bytes_transferred));
  }


  void TcpNode::handleSendErrors( const boost::system::error_code& error, size_t bytes_sent)
  {
    if (error)
    {
      ROS_ERROR("Error sending TCP packet (%d): %s", error.value(), error.message().c_str());
    }
  }

  void TcpNode::connect_handler(const boost::system::error_code& error)
  {
    if (!error)
    {
      // Connect succeeded
      ROS_INFO_STREAM("TCP node connected to " << ipAddressStr_ << ":" << port_ << "!");
      connected_ = true;
    }
    else
    {
      ROS_ERROR_STREAM("Error connecting to " << ipAddressStr_ << ":" << port_ << " - (" << error.value() << ") " << error.message());
    }
  }
} // namespace asrl
