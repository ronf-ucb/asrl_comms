#include <serial/SerialNode.hpp>
#include <asrl/assert_macros.hpp>
#include <algorithm>
#include <iterator>

namespace asrl { namespace serial {
  using boost::asio::serial_port;
  typedef boost::asio::serial_port_base spbase;
    using asrl_sensor_msgs::SerialData;

  SerialNode::SerialNode()
  {
    ros::NodeHandle nh_param("~");
    serialPort_.reset( new serial_port(ioService_));      
    
    std::string serialPortName;
    nh_param.param<std::string>("serial_port", serialPortName, "/dev/ttyUSB0");

    boost::system::error_code err;
    err = serialPort_->open( serialPortName, err );   // or whatever the device might be
    ASRL_ASSERT(Exception,!err,"Error opening serial port [" << serialPortName << "]: (" << err.value() << ") " << err.message());
    ASRL_ASSERT(Exception,serialPort_->is_open(), "Unable to open serial port [" << serialPortName << "]");
      
    // Options defined here http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference/serial_port_base.html
    // General ASIO reference defined here: http://live.boost.org/doc/libs/1_44_0/doc/html/boost_asio/reference.html
    int baudRate = 9600;
    nh_param.param<int>("baud_rate",baudRate,9600);
 
    // TODO: Add these parameters.
    serialPort_->set_option( spbase::baud_rate( baudRate ) );
    serialPort_->set_option( spbase::flow_control( spbase::flow_control::none ) );
    serialPort_->set_option( spbase::parity( spbase::parity::none ) );
    serialPort_->set_option( spbase::stop_bits( spbase::stop_bits::one ) );
    serialPort_->set_option( spbase::character_size( 8 ) ); 

    subscriber_ = nh_param.subscribe("serial_send",10,&SerialNode::processOutgoingSerialMessages,this); 
    publisher_ = nh_param.advertise< SerialData >("serial_receive", 10);

    changeBaudRate_ = nh_param.advertiseService("baudrate_service",&SerialNode::changeBaudRate,this);

    int bufferSize = 2048;
    nh_param.param<int>("buffer_size",bufferSize,2048);
    data_.resize(bufferSize);

    nh_param.param<bool>("echo",echo_,false);
    ROS_INFO_STREAM("Echo: " << echo_);
  }

  SerialNode::~SerialNode()
  {
    
  }

  void SerialNode::spin()
  {
    ///////////////////////////////////////////
    // Start the ASIO asynchronous serial thread
    startSerialReceive();
    boost::thread serialThread(boost::bind(&boost::asio::io_service::run, &ioService_));

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
    serialThread.join();
    
    // Ah...success.
    ROS_INFO("Join successful");


  }

  void SerialNode::startSerialReceive()
    {
      // Asynchronously receive serial messages 
      // Handle them in processSerialBroadcastMessages()
      serialPort_->async_read_some( boost::asio::buffer(&data_[0],data_.size()), 
				      boost::bind(&SerialNode::processIncomingSerialMessages, this,
						  boost::asio::placeholders::error,
						  boost::asio::placeholders::bytes_transferred));    
    }

  void SerialNode::processIncomingSerialMessages(const boost::system::error_code& error,
						 size_t bytes_recvd)
  {
    ros::Time stamp = ros::Time::now();
    
    // TODO: Error handling. Diagnostics?
    if (!error)
      {
	boost::shared_ptr<serial::SerialData> packet(new serial::SerialData());
	
	packet->stamp = stamp;
	
	// Copy the data.
	packet->data.resize(bytes_recvd);
	if(bytes_recvd > 0)
	  {
            if(echo_) 
              {
                //std::copy(packet->data.begin(), packet->data.end(), std::ostream_iterator<boost::uint8_t>(std::cout,""));
                for(int i = 0; i < bytes_recvd; i++)
                  {
                    std::cout << (char)packet->data[i];
                   }
                
              }
	    memcpy(&packet->data[0],&data_[0],bytes_recvd);
	  }
	// Publish the message
	publisher_.publish(packet);
      }
    else
      {
	ROS_ERROR("Error receiving serial data (%d): %s", error.value(), error.message().c_str());
      }

    // Re-registers the asynchronous receive.
    startSerialReceive();
  }

    void SerialNode::processOutgoingSerialMessages(const SerialData::ConstPtr & packet)
  {
    serialPort_->async_write_some(boost::asio::buffer(&packet->data[0],packet->data.size()),
				  boost::bind(&SerialNode::handleSendErrors, this,
					      boost::asio::placeholders::error,
					      boost::asio::placeholders::bytes_transferred));
  }

  void SerialNode::handleSendErrors( const boost::system::error_code& error, size_t bytes_sent)
  {
    
    if(error)
      {
	ROS_ERROR("Error sending serial data (%d): %s", error.value(), error.message().c_str());
      }
  }

  bool SerialNode::changeBaudRate(::serial::Baud::Request& req, ::serial::Baud::Response& resp)
  {
	  serialPort_->set_option( spbase::baud_rate( req.rate ) );
	  spbase::baud_rate option;
	  serialPort_->get_option(option);

	  if( option.value() == req.rate)
		  resp.isBaudCorrect = 1;
	  else
		  resp.isBaudCorrect = 0;
	  return resp.isBaudCorrect;
  }

  }} // namespace asrl::serial
