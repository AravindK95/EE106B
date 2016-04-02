// Generated by gencpp from file kalman_zumy/ImuSrvResponse.msg
// DO NOT EDIT!


#ifndef KALMAN_ZUMY_MESSAGE_IMUSRVRESPONSE_H
#define KALMAN_ZUMY_MESSAGE_IMUSRVRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>

namespace kalman_zumy
{
template <class ContainerAllocator>
struct ImuSrvResponse_
{
  typedef ImuSrvResponse_<ContainerAllocator> Type;

  ImuSrvResponse_()
    : linear_acceleration()
    , angular_velocity()
    , linear_acceleration_filtered()
    , angular_velocity_filtered()  {
    }
  ImuSrvResponse_(const ContainerAllocator& _alloc)
    : linear_acceleration(_alloc)
    , angular_velocity(_alloc)
    , linear_acceleration_filtered(_alloc)
    , angular_velocity_filtered(_alloc)  {
    }



   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _linear_acceleration_type;
  _linear_acceleration_type linear_acceleration;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _angular_velocity_type;
  _angular_velocity_type angular_velocity;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _linear_acceleration_filtered_type;
  _linear_acceleration_filtered_type linear_acceleration_filtered;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _angular_velocity_filtered_type;
  _angular_velocity_filtered_type angular_velocity_filtered;




  typedef boost::shared_ptr< ::kalman_zumy::ImuSrvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kalman_zumy::ImuSrvResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ImuSrvResponse_

typedef ::kalman_zumy::ImuSrvResponse_<std::allocator<void> > ImuSrvResponse;

typedef boost::shared_ptr< ::kalman_zumy::ImuSrvResponse > ImuSrvResponsePtr;
typedef boost::shared_ptr< ::kalman_zumy::ImuSrvResponse const> ImuSrvResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kalman_zumy::ImuSrvResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kalman_zumy::ImuSrvResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace kalman_zumy

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::kalman_zumy::ImuSrvResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kalman_zumy::ImuSrvResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kalman_zumy::ImuSrvResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kalman_zumy::ImuSrvResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kalman_zumy::ImuSrvResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kalman_zumy::ImuSrvResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kalman_zumy::ImuSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41085fd9e23edf9efb8c5d896ef9228";
  }

  static const char* value(const ::kalman_zumy::ImuSrvResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41085fd9e23edf9ULL;
  static const uint64_t static_value2 = 0xefb8c5d896ef9228ULL;
};

template<class ContainerAllocator>
struct DataType< ::kalman_zumy::ImuSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kalman_zumy/ImuSrvResponse";
  }

  static const char* value(const ::kalman_zumy::ImuSrvResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kalman_zumy::ImuSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Vector3 linear_acceleration\n\
geometry_msgs/Vector3 angular_velocity\n\
geometry_msgs/Vector3 linear_acceleration_filtered\n\
geometry_msgs/Vector3 angular_velocity_filtered\n\
\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::kalman_zumy::ImuSrvResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kalman_zumy::ImuSrvResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.linear_acceleration);
      stream.next(m.angular_velocity);
      stream.next(m.linear_acceleration_filtered);
      stream.next(m.angular_velocity_filtered);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct ImuSrvResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kalman_zumy::ImuSrvResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kalman_zumy::ImuSrvResponse_<ContainerAllocator>& v)
  {
    s << indent << "linear_acceleration: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.linear_acceleration);
    s << indent << "angular_velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.angular_velocity);
    s << indent << "linear_acceleration_filtered: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.linear_acceleration_filtered);
    s << indent << "angular_velocity_filtered: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.angular_velocity_filtered);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KALMAN_ZUMY_MESSAGE_IMUSRVRESPONSE_H