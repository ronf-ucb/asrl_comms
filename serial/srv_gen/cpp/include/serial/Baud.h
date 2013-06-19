/* Auto-generated by genmsg_cpp for file /home/linaro/fearing_rosbuild_overlay/asrl_comms/serial/srv/Baud.srv */
#ifndef SERIAL_SERVICE_BAUD_H
#define SERIAL_SERVICE_BAUD_H
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

#include "ros/service_traits.h"




namespace serial
{
template <class ContainerAllocator>
struct BaudRequest_ {
  typedef BaudRequest_<ContainerAllocator> Type;

  BaudRequest_()
  : rate(0)
  {
  }

  BaudRequest_(const ContainerAllocator& _alloc)
  : rate(0)
  {
  }

  typedef uint32_t _rate_type;
  uint32_t rate;


  typedef boost::shared_ptr< ::serial::BaudRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::serial::BaudRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct BaudRequest
typedef  ::serial::BaudRequest_<std::allocator<void> > BaudRequest;

typedef boost::shared_ptr< ::serial::BaudRequest> BaudRequestPtr;
typedef boost::shared_ptr< ::serial::BaudRequest const> BaudRequestConstPtr;


template <class ContainerAllocator>
struct BaudResponse_ {
  typedef BaudResponse_<ContainerAllocator> Type;

  BaudResponse_()
  : isBaudCorrect(false)
  {
  }

  BaudResponse_(const ContainerAllocator& _alloc)
  : isBaudCorrect(false)
  {
  }

  typedef uint8_t _isBaudCorrect_type;
  uint8_t isBaudCorrect;


  typedef boost::shared_ptr< ::serial::BaudResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::serial::BaudResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct BaudResponse
typedef  ::serial::BaudResponse_<std::allocator<void> > BaudResponse;

typedef boost::shared_ptr< ::serial::BaudResponse> BaudResponsePtr;
typedef boost::shared_ptr< ::serial::BaudResponse const> BaudResponseConstPtr;

struct Baud
{

typedef BaudRequest Request;
typedef BaudResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct Baud
} // namespace serial

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::serial::BaudRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::serial::BaudRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::serial::BaudRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e2079609cc0d84210a175117b44fa8b1";
  }

  static const char* value(const  ::serial::BaudRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe2079609cc0d8421ULL;
  static const uint64_t static_value2 = 0x0a175117b44fa8b1ULL;
};

template<class ContainerAllocator>
struct DataType< ::serial::BaudRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "serial/BaudRequest";
  }

  static const char* value(const  ::serial::BaudRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::serial::BaudRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint32 rate\n\
\n\
";
  }

  static const char* value(const  ::serial::BaudRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::serial::BaudRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::serial::BaudResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::serial::BaudResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::serial::BaudResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "23ed9579cf57d53cb774dc14e7bcc3ae";
  }

  static const char* value(const  ::serial::BaudResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x23ed9579cf57d53cULL;
  static const uint64_t static_value2 = 0xb774dc14e7bcc3aeULL;
};

template<class ContainerAllocator>
struct DataType< ::serial::BaudResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "serial/BaudResponse";
  }

  static const char* value(const  ::serial::BaudResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::serial::BaudResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool isBaudCorrect\n\
\n\
\n\
";
  }

  static const char* value(const  ::serial::BaudResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::serial::BaudResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::serial::BaudRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.rate);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct BaudRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::serial::BaudResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.isBaudCorrect);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct BaudResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<serial::Baud> {
  static const char* value() 
  {
    return "12a3269bd96cfc4f075d690eb501992f";
  }

  static const char* value(const serial::Baud&) { return value(); } 
};

template<>
struct DataType<serial::Baud> {
  static const char* value() 
  {
    return "serial/Baud";
  }

  static const char* value(const serial::Baud&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<serial::BaudRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "12a3269bd96cfc4f075d690eb501992f";
  }

  static const char* value(const serial::BaudRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<serial::BaudRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "serial/Baud";
  }

  static const char* value(const serial::BaudRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<serial::BaudResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "12a3269bd96cfc4f075d690eb501992f";
  }

  static const char* value(const serial::BaudResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<serial::BaudResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "serial/Baud";
  }

  static const char* value(const serial::BaudResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // SERIAL_SERVICE_BAUD_H
