#ifndef VIS4EARTH_MATH_H
#define VIS4EARTH_MATH_H

#include <array>

#include <osg/Math>

namespace VIS4Earth {
namespace Math {

template <typename FloatTy> FloatTy Deg2Rad(FloatTy deg) { return deg * osg::PI / 180.; };

template <typename FloatTy>
std::array<FloatTy, 3> BLHToEarth(FloatTy longtitude, FloatTy latitude, FloatTy height) {
    std::array<FloatTy, 3> ret;
    ret[2] = height * std::sin(latitude);
    height *= std::cos(latitude);
    ret[1] = height * std::sin(longtitude);
    ret[0] = height * std::cos(longtitude);
    return ret;
}

} // namespace Math
} // namespace VIS4Earth

#endif // !VIS4EARTH_MATH_H
