/* Auto-generated by genmsg_cpp for file /home/linaro/fearing_rosbuild_overlay/asrl_comms/asrl_sensor_msgs/msg/InclinometerData.msg */
#ifndef ASRL_SENSOR_MSGS_MESSAGE_INCLINOMETERDATA_H
#define ASRL_SENSOR_MSGS_MESSAGE_INCLINOMETERDATA_H
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
struct InclinometerData_ {
  typedef InclinometerData_<ContainerAllocator> Type;

  InclinometerData_()
  : header()
  , xDegrees(0.0)
  , yDegrees(0.0)
  , tempCelcius(0.0)
  {
  }

  InclinometerData_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , xDegrees(0.0)
  , yDegrees(0.0)
  , tempCelcius(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef float _xDegrees_type;
  float xDegrees;

  typedef float _yDegrees_type;
  float yDegrees;

  typedef float _tempCelcius_type;
  float tempCelcius;


  typedef boost::shared_ptr< ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct InclinometerData
typedef  ::asrl_sensor_msgs::InclinometerData_<std::allocator<void> > InclinometerData;

typedef boost::shared_ptr< ::asrl_sensor_msgs::InclinometerData> InclinometerDataPtr;
typedef boost::shared_ptr< ::asrl_sensor_msgs::InclinometerData const> InclinometerDataConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace asrl_sensor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "74137248fe2cc755280d4d4dd7b4237c";
  }

  static const char* value(const  ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x74137248fe2cc755ULL;
  static const uint64_t static_value2 = 0x280d4d4dd7b4237cULL;
};

template<class ContainerAllocator>
struct DataType< ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "asrl_sensor_msgs/InclinometerData";
  }

  static const char* value(const  ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
\n\
# Inclinometer data\n\
float32 xDegrees\n\
float32 yDegrees\n\
float32 tempCelcius\n\
\n\
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

  static const char* value(const  ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.xDegrees);
    stream.next(m.yDegrees);
    stream.next(m.tempCelcius);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct InclinometerData_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::asrl_sensor_msgs::InclinometerData_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "xDegrees: ";
    Printer<float>::stream(s, indent + "  ", v.xDegrees);
    s << indent << "yDegrees: ";
    Printer<float>::stream(s, indent + "  ", v.yDegrees);
    s << indent << "tempCelcius: ";
    Printer<float>::stream(s, indent + "  ", v.tempCelcius);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ASRL_SENSOR_MSGS_MESSAGE_INCLINOMETERDATA_H

