// Generated by gencpp from file robmovil_msgs/Landmark.msg
// DO NOT EDIT!


#ifndef ROBMOVIL_MSGS_MESSAGE_LANDMARK_H
#define ROBMOVIL_MSGS_MESSAGE_LANDMARK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robmovil_msgs
{
template <class ContainerAllocator>
struct Landmark_
{
  typedef Landmark_<ContainerAllocator> Type;

  Landmark_()
    : range(0.0)
    , bearing(0.0)  {
    }
  Landmark_(const ContainerAllocator& _alloc)
    : range(0.0)
    , bearing(0.0)  {
  (void)_alloc;
    }



   typedef float _range_type;
  _range_type range;

   typedef float _bearing_type;
  _bearing_type bearing;





  typedef boost::shared_ptr< ::robmovil_msgs::Landmark_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robmovil_msgs::Landmark_<ContainerAllocator> const> ConstPtr;

}; // struct Landmark_

typedef ::robmovil_msgs::Landmark_<std::allocator<void> > Landmark;

typedef boost::shared_ptr< ::robmovil_msgs::Landmark > LandmarkPtr;
typedef boost::shared_ptr< ::robmovil_msgs::Landmark const> LandmarkConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robmovil_msgs::Landmark_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robmovil_msgs::Landmark_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robmovil_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'robmovil_msgs': ['/home/jrr/catkin_ws/src/robmovil_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robmovil_msgs::Landmark_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robmovil_msgs::Landmark_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robmovil_msgs::Landmark_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robmovil_msgs::Landmark_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robmovil_msgs::Landmark_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robmovil_msgs::Landmark_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robmovil_msgs::Landmark_<ContainerAllocator> >
{
  static const char* value()
  {
    return "90f01577d6cf664c77376bc73ab5c487";
  }

  static const char* value(const ::robmovil_msgs::Landmark_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x90f01577d6cf664cULL;
  static const uint64_t static_value2 = 0x77376bc73ab5c487ULL;
};

template<class ContainerAllocator>
struct DataType< ::robmovil_msgs::Landmark_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robmovil_msgs/Landmark";
  }

  static const char* value(const ::robmovil_msgs::Landmark_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robmovil_msgs::Landmark_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 range\n\
float32 bearing\n\
";
  }

  static const char* value(const ::robmovil_msgs::Landmark_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robmovil_msgs::Landmark_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.range);
      stream.next(m.bearing);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Landmark_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robmovil_msgs::Landmark_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robmovil_msgs::Landmark_<ContainerAllocator>& v)
  {
    s << indent << "range: ";
    Printer<float>::stream(s, indent + "  ", v.range);
    s << indent << "bearing: ";
    Printer<float>::stream(s, indent + "  ", v.bearing);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBMOVIL_MSGS_MESSAGE_LANDMARK_H
