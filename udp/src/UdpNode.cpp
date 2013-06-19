#include <udp/UdpNode.hpp>
#include <asrl/rosutil/param.hpp>

typedef boost::asio::ip::udp asio_udp;

namespace asrl {

  UdpNode::UdpNode(const ros::NodeHandle & nh) :
    nodeHandle_(nh), numSent_(0), numReceived_(0)
  {

    asrl::rosutil::param<int>(nodeHandle_, "listen_port", listenPort_, 6101);
    destinationEndpoint_ = asio_udp::endpoint(asio_udp::v4(), listenPort_);
    udpSocket_.reset( new asio_udp::socket(ioService_, destinationEndpoint_));      
    /////////////////////////////////
    // boost::asio socket options:
    //http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference.html

    // Socket option for the receive buffer size of a socket. 
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/socket_base/receive_buffer_size.html
    int receiveBufferSize = 65535;
    asrl::rosutil::param<int>(nodeHandle_, "receive_buffer_size", receiveBufferSize, 65535);
    boost::asio::socket_base::receive_buffer_size receiveBufferOption(receiveBufferSize);
    udpSocket_->set_option(receiveBufferOption);

    // Socket option for the send buffer size of a socket. 
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/socket_base/send_buffer_size.html
    int sendBufferSize = 65535;
    asrl::rosutil::param<int>(nodeHandle_, "send_buffer_size", sendBufferSize, 65535);
    boost::asio::socket_base::send_buffer_size sendBufferOption(sendBufferSize);
    udpSocket_->set_option(sendBufferOption);

    // Socket option to permit sending of broadcast messages. 
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/socket_base/broadcast.html
    bool broadcast = false;
    asrl::rosutil::param<bool>(nodeHandle_, "broadcast", broadcast, false);
    boost::asio::socket_base::broadcast broadcastOption(broadcast);
    udpSocket_->set_option(broadcastOption);
    
    // Socket option to report aborted connections on accept. 
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/socket_base/enable_connection_aborted.html
    bool enableAborted = false;
    asrl::rosutil::param<bool>(nodeHandle_, "enable_connection_aborted", enableAborted, false);
    boost::asio::socket_base::enable_connection_aborted enableAbortedOption(enableAborted);
    udpSocket_->set_option(enableAbortedOption);

    // Socket option for the receive low watermark. 
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/socket_base/receive_low_watermark.html
    int receiveLowWatermark = 1;
    asrl::rosutil::param<int>(nodeHandle_, "receive_low_watermark", receiveLowWatermark, 1);
    boost::asio::socket_base::receive_low_watermark receiveLowWatermarkOption(receiveLowWatermark);
    udpSocket_->set_option(receiveLowWatermarkOption);

    

    // Socket option to allow the socket to be bound to an address that is already in use. 
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/socket_base/reuse_address.html
    bool reuseAddress = false;
    asrl::rosutil::param<bool>(nodeHandle_, "reuse_address", reuseAddress, false);
    boost::asio::socket_base::reuse_address reuseAddressOption(reuseAddress);
    udpSocket_->set_option(reuseAddressOption);


	// ****************API not availble for setting sendLowWatermark for udp******************
	// UDP send buffer never changes (UDP does not keep a copy of datagram sent by application), reference:
	// http://books.google.com/books?id=KDJMFW-hVxsC&lpg=SA3-PA9&ots=OvaGDe1P5v&pg=SA3-PA9#v=onepage
	// ***************************************************************************    

	// Socket option for the send low watermark. 
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/socket_base/send_low_watermark.html
    
	// *****disabled the following next section (specifically set_option(sendLowWatermarkOption)) for udp socket, 
	///they are only kept as reference as the cmds are valid for tcp
	// enabling them will cause error: [FATAL] Protocol not available

	//int sendLowWatermark = 1;
    //asrl::rosutil::param<int>(nodeHandle_, "send_low_watermark", sendLowWatermark, 1);
    //boost::asio::socket_base::send_low_watermark sendLowWatermarkOption(sendLowWatermark);
    //udpSocket_->set_option(sendLowWatermarkOption);


    // Socket option to enable socket-level debugging. 
    // http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/socket_base/debug.html
    bool socketDebug = false;
    asrl::rosutil::param<bool>(nodeHandle_, "socket_debug", socketDebug, false);
    boost::asio::socket_base::debug socketDebugOption(socketDebug);
    udpSocket_->set_option(socketDebugOption);

    // do multicast?
    // ip::multicast::enable_loopback
    // ip::multicast::
    // ip::multicast::join_group
    // ip::multicast::outbound_interface

    // else
    // ip::unicast::hops

    int pubQueueDepth = 0;
    int subQueueDepth = 0;
    asrl::rosutil::param<int>(nodeHandle_, "in_queue_depth", subQueueDepth, 100);
    asrl::rosutil::param<int>(nodeHandle_, "out_queue_depth", pubQueueDepth, 100);
    subscriber_ = nodeHandle_.subscribe("in/udp",subQueueDepth,&UdpNode::packetMessageCallback,this); 
    publisher_ = nodeHandle_.advertise<asrl_sensor_msgs::Packet>("out/udp", pubQueueDepth);

    // Initialize the data buffer. This is the theoretical maximum data size
    // for a single udp packet
    data_.resize(65535);
  }

  UdpNode::~UdpNode()
  {
    
  }

  boost::asio::ip::address_v4 UdpNode::rosIpToBoostIp(asrl_sensor_msgs::IpAddress const & ip)
  {
    return boost::asio::ip::address_v4(ip.address);
  }

  asrl_sensor_msgs::IpAddress UdpNode::boostIpToRosIp(boost::asio::ip::address_v4 const & ip)
  {
    asrl_sensor_msgs::IpAddress retVal;
    retVal.address = ip.to_bytes();
    return retVal;
  }


  void UdpNode::spin()
  {

    startUdpReceive();
    boost::thread udpThread(boost::bind(&boost::asio::io_service::run, &ioService_));
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
    udpThread.join();
    
    // Ah...success.
    ROS_INFO("Join successful");


  }

  void UdpNode::startUdpReceive()
    {
      // Asynchronously receive udp messages into data_
      // Handle them in processUdpBroadcastMessages()
      udpSocket_->async_receive_from( boost::asio::buffer(&data_[0],data_.size()), sourceEndpoint_,
				      boost::bind(&UdpNode::processIncomingUdpMessages, this,
						  boost::asio::placeholders::error,
						  boost::asio::placeholders::bytes_transferred));    
    }

  void UdpNode::processIncomingUdpMessages(const boost::system::error_code& error,
					   size_t bytes_recvd)
  {
    ros::Time stamp = ros::Time::now();
    numReceived_++;
    
    // TODO: Error handling. Diagnostics?
    if (!error)
      {
	boost::shared_ptr<asrl_sensor_msgs::Packet> packet(new asrl_sensor_msgs::Packet());
	// Fill in the source address and port
	packet->source = boostIpToRosIp(sourceEndpoint_.address().to_v4());
	packet->sourcePort = sourceEndpoint_.port();
        
        // Fill in the sequence number (i.e., the number received)
        packet->seqNumber = numReceived_;

        if (packet->seqNumber % 2000 == 0)
          ROS_INFO("UDP node - sent %d packets",(int)packet->seqNumber);

	// Fill in the destination address and port
	packet->destination = boostIpToRosIp(destinationEndpoint_.address().to_v4());
	packet->destinationPort = destinationEndpoint_.port();

	// Copy the data.
	packet->data.resize(bytes_recvd);
	if(bytes_recvd > 0)
	  {
	    memcpy(&packet->data[0],&data_[0],bytes_recvd);
	  }
	// Publish the message
	publisher_.publish(packet);

        // Keep track of the number of sent packets and report once in a while
        //numSent_++;
        //if (numSent_ % 2000 == 0)
        //  ROS_INFO("Udp node statistics - num packets sent = %d, num packets received = %d",numSent_,numReceived_);
      }
    else
      {
	ROS_ERROR("Error receiving UDP packet (%d): %s", error.value(), error.message().c_str());
      }

    // Re-registers the asynchronous receive.
    startUdpReceive();
  }

  void UdpNode::packetMessageCallback(const asrl_sensor_msgs::Packet::ConstPtr & packet)
  {
    // TODO: warn if the source address and port don't match this
    //       source endpoint and port.
    asio_udp::endpoint outEndpoint(boost::asio::ip::address_v4(packet->destination.address),packet->destinationPort);
    processOutgoingUdpMessages(packet->data, outEndpoint);
  }


  void UdpNode::processOutgoingUdpMessages(std::vector<boost::uint8_t> const & data, asio_udp::endpoint sendEndpoint)
  {

    // Pass the message on to boost::asio
    udpSocket_->async_send_to(boost::asio::buffer(&data[0],data.size()), sendEndpoint,
				  boost::bind(&UdpNode::handleSendErrors, this,
					      boost::asio::placeholders::error,
					      boost::asio::placeholders::bytes_transferred));
  }

  void UdpNode::handleSendErrors( const boost::system::error_code& error, size_t bytes_sent)
  {
    
    if(error)
      {
	ROS_ERROR("Error sending UDP packet (%d): %s", error.value(), error.message().c_str());
      }
  }

} // namespace asrl
