/* Auto-generated by genmsg_cpp for file /home/sergey/MyLsdSlamProject/ROS/package_dir/lsd_slam/myTestViewer/msg/keyframeMsg.msg */
#ifndef MYTESTVIEWER_MESSAGE_KEYFRAMEMSG_H
#define MYTESTVIEWER_MESSAGE_KEYFRAMEMSG_H
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


namespace myTestViewer
{
template <class ContainerAllocator>
struct keyframeMsg_ {
  typedef keyframeMsg_<ContainerAllocator> Type;

  keyframeMsg_()
  : id(0)
  , time(0.0)
  , isKeyframe(false)
  , camToWorld()
  , fx(0.0)
  , fy(0.0)
  , cx(0.0)
  , cy(0.0)
  , height(0)
  , width(0)
  , pointcloud()
  {
    camToWorld.assign(0.0);
  }

  keyframeMsg_(const ContainerAllocator& _alloc)
  : id(0)
  , time(0.0)
  , isKeyframe(false)
  , camToWorld()
  , fx(0.0)
  , fy(0.0)
  , cx(0.0)
  , cy(0.0)
  , height(0)
  , width(0)
  , pointcloud(_alloc)
  {
    camToWorld.assign(0.0);
  }

  typedef int32_t _id_type;
  int32_t id;

  typedef double _time_type;
  double time;

  typedef uint8_t _isKeyframe_type;
  uint8_t isKeyframe;

  typedef boost::array<float, 7>  _camToWorld_type;
  boost::array<float, 7>  camToWorld;

  typedef float _fx_type;
  float fx;

  typedef float _fy_type;
  float fy;

  typedef float _cx_type;
  float cx;

  typedef float _cy_type;
  float cy;

  typedef uint32_t _height_type;
  uint32_t height;

  typedef uint32_t _width_type;
  uint32_t width;

  typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _pointcloud_type;
  std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  pointcloud;


  typedef boost::shared_ptr< ::myTestViewer::keyframeMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::myTestViewer::keyframeMsg_<ContainerAllocator>  const> ConstPtr;
}; // struct keyframeMsg
typedef  ::myTestViewer::keyframeMsg_<std::allocator<void> > keyframeMsg;

typedef boost::shared_ptr< ::myTestViewer::keyframeMsg> keyframeMsgPtr;
typedef boost::shared_ptr< ::myTestViewer::keyframeMsg const> keyframeMsgConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::myTestViewer::keyframeMsg_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::myTestViewer::keyframeMsg_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace myTestViewer

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::myTestViewer::keyframeMsg_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::myTestViewer::keyframeMsg_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::myTestViewer::keyframeMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "42d4108dbb7d0e5d166eb68dd4054826";
  }

  static const char* value(const  ::myTestViewer::keyframeMsg_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x42d4108dbb7d0e5dULL;
  static const uint64_t static_value2 = 0x166eb68dd4054826ULL;
};

template<class ContainerAllocator>
struct DataType< ::myTestViewer::keyframeMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "myTestViewer/keyframeMsg";
  }

  static const char* value(const  ::myTestViewer::keyframeMsg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::myTestViewer::keyframeMsg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 id\n\
float64 time\n\
bool isKeyframe\n\
\n\
# camToWorld as serialization of sophus sim(3).\n\
# may change with keyframeGraph-updates.\n\
float32[7] camToWorld \n\
\n\
\n\
# camera parameter (fx fy cx cy), width, height\n\
# will never change, but required for display.\n\
float32 fx\n\
float32 fy\n\
float32 cx\n\
float32 cy\n\
uint32 height\n\
uint32 width\n\
\n\
\n\
# data as InputPointDense (float idepth, float idepth_var, uchar color[4]), width x height\n\
# may be empty, in that case no associated pointcloud is ever shown.\n\
uint8[] pointcloud\n\
\n\
";
  }

  static const char* value(const  ::myTestViewer::keyframeMsg_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::myTestViewer::keyframeMsg_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.id);
    stream.next(m.time);
    stream.next(m.isKeyframe);
    stream.next(m.camToWorld);
    stream.next(m.fx);
    stream.next(m.fy);
    stream.next(m.cx);
    stream.next(m.cy);
    stream.next(m.height);
    stream.next(m.width);
    stream.next(m.pointcloud);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct keyframeMsg_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::myTestViewer::keyframeMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::myTestViewer::keyframeMsg_<ContainerAllocator> & v) 
  {
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "time: ";
    Printer<double>::stream(s, indent + "  ", v.time);
    s << indent << "isKeyframe: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.isKeyframe);
    s << indent << "camToWorld[]" << std::endl;
    for (size_t i = 0; i < v.camToWorld.size(); ++i)
    {
      s << indent << "  camToWorld[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.camToWorld[i]);
    }
    s << indent << "fx: ";
    Printer<float>::stream(s, indent + "  ", v.fx);
    s << indent << "fy: ";
    Printer<float>::stream(s, indent + "  ", v.fy);
    s << indent << "cx: ";
    Printer<float>::stream(s, indent + "  ", v.cx);
    s << indent << "cy: ";
    Printer<float>::stream(s, indent + "  ", v.cy);
    s << indent << "height: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.height);
    s << indent << "width: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.width);
    s << indent << "pointcloud[]" << std::endl;
    for (size_t i = 0; i < v.pointcloud.size(); ++i)
    {
      s << indent << "  pointcloud[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.pointcloud[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // MYTESTVIEWER_MESSAGE_KEYFRAMEMSG_H

