#ifndef _MATH_H_
#define _MATH_H_

#include <gazebo/math/Quaternion.hh>

gazebo::math::Quaternion deriv( gazebo::math::Quaternion q, gazebo::math::Vector3 w ) {
  gazebo::math::Quaternion qd;

  qd.w = .5 * (-q.x * w.x - q.y * w.y - q.z * w.z);
  qd.x = .5 * (+q.w * w.x + q.z * w.y - q.y * w.z);
  qd.y = .5 * (-q.z * w.x + q.w * w.y + q.x * w.z);
  qd.z = .5 * (+q.y * w.x - q.x * w.y + q.w * w.z);

  return qd;
}

gazebo::math::Vector3 to_omega( gazebo::math::Quaternion q, gazebo::math::Quaternion qd )
{
  gazebo::math::Vector3 omega;
  omega.x = 2 * (-q.x * qd.w + q.w * qd.x - q.z * qd.y + q.y * qd.z);
  omega.y = 2 * (-q.y * qd.w + q.z * qd.x + q.w * qd.y - q.x * qd.z);
  omega.z = 2 * (-q.z * qd.w - q.y * qd.x + q.x * qd.y + q.w * qd.z);
  return omega;
}


#endif // _MATH_H_
