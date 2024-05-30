#ifndef VIS4EARTH_QT_UTIL_H
#define VIS4EARTH_QT_UTIL_H

#include <array>

#include <QtGui/QColor>

namespace VIS4Earth {

inline QColor RGBAToQColor(const std::array<float, 4> &rgba) {
    return QColor::fromRgba(qRgba(std::max(std::min(rgba[0] * 255.f, 255.f), 0.f),
                                  std::max(std::min(rgba[1] * 255.f, 255.f), 0.f),
                                  std::max(std::min(rgba[2] * 255.f, 255.f), 0.f),
                                  std::max(std::min(rgba[3] * 255.f, 255.f), 0.f)));
}

inline std::array<float, 4> QColorToRGBA(const QColor &color) {
    return std::array<float, 4>{
        static_cast<float>(color.redF()), static_cast<float>(color.greenF()),
        static_cast<float>(color.blueF()), static_cast<float>(color.alphaF())};
}

} // namespace VIS4Earth

#endif // !VIS4EARTH_QT_UTIL_H
