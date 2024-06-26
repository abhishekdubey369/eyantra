// Generated by gencpp from file whycon/Projection.msg
// DO NOT EDIT!


#ifndef WHYCON_MESSAGE_PROJECTION_H
#define WHYCON_MESSAGE_PROJECTION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace whycon
{
template <class ContainerAllocator>
struct Projection_
{
  typedef Projection_<ContainerAllocator> Type;

  Projection_()
    : header()
    , projection()  {
      projection.assign(0.0);
  }
  Projection_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , projection()  {
  (void)_alloc;
      projection.assign(0.0);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef boost::array<double, 9>  _projection_type;
  _projection_type projection;





  typedef boost::shared_ptr< ::whycon::Projection_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::whycon::Projection_<ContainerAllocator> const> ConstPtr;

}; // struct Projection_

typedef ::whycon::Projection_<std::allocator<void> > Projection;

typedef boost::shared_ptr< ::whycon::Projection > ProjectionPtr;
typedef boost::shared_ptr< ::whycon::Projection const> ProjectionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::whycon::Projection_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::whycon::Projection_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::whycon::Projection_<ContainerAllocator1> & lhs, const ::whycon::Projection_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.projection == rhs.projection;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::whycon::Projection_<ContainerAllocator1> & lhs, const ::whycon::Projection_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace whycon

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::whycon::Projection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::whycon::Projection_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::whycon::Projection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::whycon::Projection_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::whycon::Projection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::whycon::Projection_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::whycon::Projection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f8f0ec7b4ddd16195597a2d3ca529821";
  }

  static const char* value(const ::whycon::Projection_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf8f0ec7b4ddd1619ULL;
  static const uint64_t static_value2 = 0x5597a2d3ca529821ULL;
};

template<class ContainerAllocator>
struct DataType< ::whycon::Projection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "whycon/Projection";
  }

  static const char* value(const ::whycon::Projection_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::whycon::Projection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"float64[9] projection\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::whycon::Projection_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::whycon::Projection_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.projection);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Projection_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::whycon::Projection_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::whycon::Projection_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "projection[]" << std::endl;
    for (size_t i = 0; i < v.projection.size(); ++i)
    {
      s << indent << "  projection[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.projection[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // WHYCON_MESSAGE_PROJECTION_H
