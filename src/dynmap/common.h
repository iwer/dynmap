/**
 *  @file common.h
 *
 *  @date 02.10.2012
 *      @author Iwer Petersen
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <pcl/point_types.h>

#define FPS_CALC_BEGIN                          \
    static double duration = 0;                 \
    double start_time = pcl::getTime ();        \

#define FPS_CALC_END(_WHAT_)                    \
  {                                             \
    double end_time = pcl::getTime ();          \
    static unsigned count = 0;                  \
    if (++count == 10)                          \
    {                                           \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(duration) << " Hz" <<  std::endl; \
      count = 0;                                                        \
      duration = 0.0;                                                   \
    }                                           \
    else                                        \
    {                                           \
      duration += end_time - start_time;        \
    }                                           \
  }

typedef typename pcl::PointXYZRGBA DefaultPoint;
typedef typename pcl::PointCloud<DefaultPoint> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

#endif /* COMMON_H_ */
