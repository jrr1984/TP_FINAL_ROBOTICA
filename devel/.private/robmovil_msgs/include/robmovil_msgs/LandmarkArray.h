// Generated by gencpp from file robmovil_msgs/LandmarkArray.msg
// DO NOT EDIT!


#ifndef ROBMOVIL_MSGS_MESSAGE_LANDMARKARRAY_H
#define ROBMOVIL_MSGS_MESSAGE_LANDMARKARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <robmovil_msgs/Landmark.h>

namespace robmovil_msgs
{
template <class ContainerAllocator>
struct LandmarkArray_
{
  typedef LandmarkArray_<ContainerAllocator> Type;

  LandmarkArray_()
    : header()
    , landmarks()  {
    }
  LandmarkArray_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , landmarks(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::robmovil_msgs::Landmark_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::robmovil_msgs::Landmark_<ContainerAllocator> >::other >  _landmarks_type;
  _landmarks_type landmarks;





  typedef boost::shared_ptr< ::robmovil_msgs::LandmarkArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robmovil_msgs::LandmarkArray_<ContainerAllocator> const> ConstPtr;

}; // struct LandmarkArray_

typedef ::robmovil_msgs::LandmarkArray_<std::allocator<void> > LandmarkArray;

typedef boost::shared_ptr< ::robmovil_msgs::LandmarkArray > LandmarkArrayPtr;
typedef boost::shared_ptr< ::robmovil_msgs::LandmarkArray const> LandmarkArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robmovil_msgs::LandmarkArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robmovil_msgs::LandmarkArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robmovil_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'robmovil_msgs': ['/home/jrr/catkin_ws/src/robmovil_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robmovil_msgs::LandmarkArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robmovil_msgs::LandmarkArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robmovil_msgs::LandmarkArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robmovil_msgs::LandmarkArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robmovil_msgs::LandmarkArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robmovil_msgs::LandmarkArray_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robmovil_msgs::LandmarkArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7542da8c7dee411bb2ec76a7d4d48c04";
  }

  static const char* value(const ::robmovil_msgs::LandmarkArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7542da8c7dee411bULL;
  static const uint64_t static_value2 = 0xb2ec76a7d4d48c04ULL;
};

template<class ContainerAllocator>
struct DataType< ::robmovil_msgs::LandmarkArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robmovil_msgs/LandmarkArray";
  }

  static const char* value(const ::robmovil_msgs::LandmarkArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robmovil_msgs::LandmarkArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
Landmark[] landmarks\n\
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
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: robmovil_msgs/Landmark\n\
float32 range\n\
float32 bearing\n\
";
  }

  static const char* value(const ::robmovil_msgs::LandmarkArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robmovil_msgs::LandmarkArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.landmarks);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LandmarkArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robmovil_msgs::LandmarkArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robmovil_msgs::LandmarkArray_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "landmarks[]" << std::endl;
    for (size_t i = 0; i < v.landmarks.size(); ++i)
    {
      s << indent << "  landmarks[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::robmovil_msgs::Landmark_<ContainerAllocator> >::stream(s, indent + "    ", v.landmarks[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBMOVIL_MSGS_MESSAGE_LANDMARKARRAY_H