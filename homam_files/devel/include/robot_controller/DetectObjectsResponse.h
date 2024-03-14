// Generated by gencpp from file robot_controller/DetectObjectsResponse.msg
// DO NOT EDIT!


#ifndef ROBOT_CONTROLLER_MESSAGE_DETECTOBJECTSRESPONSE_H
#define ROBOT_CONTROLLER_MESSAGE_DETECTOBJECTSRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <robot_controller/DetectedObject.h>

namespace robot_controller
{
template <class ContainerAllocator>
struct DetectObjectsResponse_
{
  typedef DetectObjectsResponse_<ContainerAllocator> Type;

  DetectObjectsResponse_()
    : object()  {
    }
  DetectObjectsResponse_(const ContainerAllocator& _alloc)
    : object(_alloc)  {
  (void)_alloc;
    }



   typedef  ::robot_controller::DetectedObject_<ContainerAllocator>  _object_type;
  _object_type object;





  typedef boost::shared_ptr< ::robot_controller::DetectObjectsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_controller::DetectObjectsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct DetectObjectsResponse_

typedef ::robot_controller::DetectObjectsResponse_<std::allocator<void> > DetectObjectsResponse;

typedef boost::shared_ptr< ::robot_controller::DetectObjectsResponse > DetectObjectsResponsePtr;
typedef boost::shared_ptr< ::robot_controller::DetectObjectsResponse const> DetectObjectsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_controller::DetectObjectsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_controller::DetectObjectsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robot_controller::DetectObjectsResponse_<ContainerAllocator1> & lhs, const ::robot_controller::DetectObjectsResponse_<ContainerAllocator2> & rhs)
{
  return lhs.object == rhs.object;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robot_controller::DetectObjectsResponse_<ContainerAllocator1> & lhs, const ::robot_controller::DetectObjectsResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robot_controller

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::robot_controller::DetectObjectsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_controller::DetectObjectsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_controller::DetectObjectsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_controller::DetectObjectsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_controller::DetectObjectsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_controller::DetectObjectsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_controller::DetectObjectsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "832bf2ae172cf259fafc45005d594984";
  }

  static const char* value(const ::robot_controller::DetectObjectsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x832bf2ae172cf259ULL;
  static const uint64_t static_value2 = 0xfafc45005d594984ULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_controller::DetectObjectsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_controller/DetectObjectsResponse";
  }

  static const char* value(const ::robot_controller::DetectObjectsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_controller::DetectObjectsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "DetectedObject object\n"
"\n"
"\n"
"================================================================================\n"
"MSG: robot_controller/DetectedObject\n"
"int32 x1\n"
"int32 y1\n"
"int32 x2\n"
"int32 y2\n"
"string class_name\n"
;
  }

  static const char* value(const ::robot_controller::DetectObjectsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_controller::DetectObjectsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.object);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DetectObjectsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_controller::DetectObjectsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_controller::DetectObjectsResponse_<ContainerAllocator>& v)
  {
    s << indent << "object: ";
    s << std::endl;
    Printer< ::robot_controller::DetectedObject_<ContainerAllocator> >::stream(s, indent + "  ", v.object);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_CONTROLLER_MESSAGE_DETECTOBJECTSRESPONSE_H