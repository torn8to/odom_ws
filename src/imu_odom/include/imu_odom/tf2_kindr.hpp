#ifndef TF2_KINDR_HPP_
#define TF2_KINDR_HPP_

#include <tf2/convert.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <kindr/Core>

namespace tf2
{

/**template<>
inline void fromMsg(const geometry_msgs::msg::Vector3& msg, kindr::Vector<kindr::PhysicalType::Linear, double, 3>& vec)
{
  vec[0] = msg.x;
  vec[1] = msg.y;
  vec[2] = msg.z;
}**/

template<>
inline void fromMsg(const geometry_msgs::msg::Vector3& msg, kindr::Vector<kindr::PhysicalType::Typeless, double, 3>& vec)
{
  vec.x() = msg.x;
  vec.y() = msg.y;
  vec.z() = msg.z;
}

template<>
inline void fromMsg(const geometry_msgs::msg::Vector3& msg, kindr::Vector<kindr::PhysicalType::AngularVelocity, double, 3>& vec)
{
  vec.x() = msg.x;
  vec.y() = msg.y;
  vec.z() = msg.z;
}

} // namespace tf2

#endif // TF2_KINDR_HPP_ 