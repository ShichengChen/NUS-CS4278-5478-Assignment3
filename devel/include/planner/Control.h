// Generated by gencpp from file planner/Control.msg
// DO NOT EDIT!


#ifndef PLANNER_MESSAGE_CONTROL_H
#define PLANNER_MESSAGE_CONTROL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace planner
{
template <class ContainerAllocator>
struct Control_
{
  typedef Control_<ContainerAllocator> Type;

  Control_()
    : v(0.0)
    , w(0.0)  {
    }
  Control_(const ContainerAllocator& _alloc)
    : v(0.0)
    , w(0.0)  {
  (void)_alloc;
    }



   typedef double _v_type;
  _v_type v;

   typedef double _w_type;
  _w_type w;





  typedef boost::shared_ptr< ::planner::Control_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::planner::Control_<ContainerAllocator> const> ConstPtr;

}; // struct Control_

typedef ::planner::Control_<std::allocator<void> > Control;

typedef boost::shared_ptr< ::planner::Control > ControlPtr;
typedef boost::shared_ptr< ::planner::Control const> ControlConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::planner::Control_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::planner::Control_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace planner

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'planner': ['/home/csc/CS4278-5478-Assignment3/src/planner/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::planner::Control_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::planner::Control_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::planner::Control_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::planner::Control_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::planner::Control_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::planner::Control_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::planner::Control_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e63966e769215a305e1aebe679281346";
  }

  static const char* value(const ::planner::Control_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe63966e769215a30ULL;
  static const uint64_t static_value2 = 0x5e1aebe679281346ULL;
};

template<class ContainerAllocator>
struct DataType< ::planner::Control_<ContainerAllocator> >
{
  static const char* value()
  {
    return "planner/Control";
  }

  static const char* value(const ::planner::Control_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::planner::Control_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 v\n\
float64 w\n\
";
  }

  static const char* value(const ::planner::Control_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::planner::Control_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.v);
      stream.next(m.w);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Control_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::planner::Control_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::planner::Control_<ContainerAllocator>& v)
  {
    s << indent << "v: ";
    Printer<double>::stream(s, indent + "  ", v.v);
    s << indent << "w: ";
    Printer<double>::stream(s, indent + "  ", v.w);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PLANNER_MESSAGE_CONTROL_H
