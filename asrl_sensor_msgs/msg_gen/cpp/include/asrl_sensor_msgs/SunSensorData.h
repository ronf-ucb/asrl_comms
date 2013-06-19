/* Auto-generated by genmsg_cpp for file /home/linaro/fearing_rosbuild_overlay/asrl_comms/asrl_sensor_msgs/msg/SunSensorData.msg */
#ifndef ASRL_SENSOR_MSGS_MESSAGE_SUNSENSORDATA_H
#define ASRL_SENSOR_MSGS_MESSAGE_SUNSENSORDATA_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"

namespace asrl_sensor_msgs
{
template <class ContainerAllocator>
struct SunSensorData_ {
  typedef SunSensorData_<ContainerAllocator> Type;

  SunSensorData_()
  : header()
  , image()
  , tempCelcius(0.0)
  , sunVector()
  , errorCode1(0)
  , errorCode2(0)
  {
  }

  SunSensorData_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , image(_alloc)
  , tempCelcius(0.0)
  , sunVector(_alloc)
  , errorCode1(0)
  , errorCode2(0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector<uint16_t, typename ContainerAllocator::template rebind<uint16_t>::other >  _image_type;
  std::vector<uint16_t, typename ContainerAllocator::template rebind<uint16_t>::other >  image;

  typedef float _tempCelcius_type;
  float tempCelcius;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _sunVector_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  sunVector;

  typedef int32_t _errorCode1_type;
  int32_t errorCode1;

  typedef int32_t _errorCode2_type;
  int32_t errorCode2;


  typedef boost::shared_ptr< ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SunSensorData
typedef  ::asrl_sensor_msgs::SunSensorData_<std::allocator<void> > SunSensorData;

typedef boost::shared_ptr< ::asrl_sensor_msgs::SunSensorData> SunSensorDataPtr;
typedef boost::shared_ptr< ::asrl_sensor_msgs::SunSensorData const> SunSensorDataConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace asrl_sensor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "06161ef49dc9ea3e276d16646600287d";
  }

  static const char* value(const  ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x06161ef49dc9ea3eULL;
  static const uint64_t static_value2 = 0x276d16646600287dULL;
};

template<class ContainerAllocator>
struct DataType< ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "asrl_sensor_msgs/SunSensorData";
  }

  static const char* value(const  ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
\n\
# Inclinometer data\n\
uint16[] image\n\
float32 tempCelcius\n\
\n\
# Sun Vector \n\
float32[] sunVector\n\
\n\
# Error code 1 from the sun vector computation algorithm\n\
# The value of the error code should be interpreted as follows:\n\
# >= 0: Successful image.\n\
#   -1: Error returns during NLSQ fit, solution may be valid to nearest pixel.\n\
#   -2: Too many peaks, solution not assigned.\n\
#   -3: Too few peaks, solution not assigned \n\
#   -4: Appropriate number of peaks found, but fit is of poor quality.\n\
int32 errorCode1\n\
\n\
# Error code 2 from the sun vector computation algorithm\n\
# The value of the error code should be interpreted as follows:\n\
#    0: Good geometry.\n\
#   -5: Imaginary solution.\n\
int32 errorCode2\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.image);
    stream.next(m.tempCelcius);
    stream.next(m.sunVector);
    stream.next(m.errorCode1);
    stream.next(m.errorCode2);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SunSensorData_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::asrl_sensor_msgs::SunSensorData_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "image[]" << std::endl;
    for (size_t i = 0; i < v.image.size(); ++i)
    {
      s << indent << "  image[" << i << "]: ";
      Printer<uint16_t>::stream(s, indent + "  ", v.image[i]);
    }
    s << indent << "tempCelcius: ";
    Printer<float>::stream(s, indent + "  ", v.tempCelcius);
    s << indent << "sunVector[]" << std::endl;
    for (size_t i = 0; i < v.sunVector.size(); ++i)
    {
      s << indent << "  sunVector[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.sunVector[i]);
    }
    s << indent << "errorCode1: ";
    Printer<int32_t>::stream(s, indent + "  ", v.errorCode1);
    s << indent << "errorCode2: ";
    Printer<int32_t>::stream(s, indent + "  ", v.errorCode2);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ASRL_SENSOR_MSGS_MESSAGE_SUNSENSORDATA_H
