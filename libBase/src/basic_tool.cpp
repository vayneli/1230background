#include "basic_tool.h"
#include "iostream"
#include <cmath>

struct timeval BasicTool::start=startInitGet();

struct timeval BasicTool::startInitGet() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv;
}


int BasicTool::currentTimeMsGet() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return (int)((tv.tv_sec-start.tv_sec)*1000+(tv.tv_usec-start.tv_usec)/1000);
}
Telemetry::Vector3f BasicTool::toEulerAngle(Quaternion quaternionData)
{
  Telemetry::Vector3f    ans;
  Telemetry::Quaternion quaternion = quaternionData;

  double q2sqr = quaternion.q2 * quaternion.q2;
  double t0    = -2.0 * (q2sqr + quaternion.q3 * quaternion.q3) + 1.0;
  double t1 =
    +2.0 * (quaternion.q1 * quaternion.q2 + quaternion.q0 * quaternion.q3);
  double t2 =
    -2.0 * (quaternion.q1 * quaternion.q3 - quaternion.q0 * quaternion.q2);
  double t3 =
    +2.0 * (quaternion.q2 * quaternion.q3 + quaternion.q0 * quaternion.q1);
  double t4 = -2.0 * (quaternion.q1 * quaternion.q1 + q2sqr) + 1.0;

  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;

  ans.x = asin(t2);
  ans.y = atan2(t3, t4);
  ans.z = atan2(t1, t0);

  return ans;
}