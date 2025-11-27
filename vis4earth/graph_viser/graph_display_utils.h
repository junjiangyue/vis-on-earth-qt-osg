#ifndef VIS4EARTH_GRAPH_VISER_GRAPH_DISPLAY_UTILS_H
#define VIS4EARTH_GRAPH_VISER_GRAPH_DISPLAY_UTILS_H

#include <osg/BoundingBox>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/Math>
#include <osg/CoordinateSystemNode>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <sstream>
#include <iomanip>

namespace VIS4Earth {

// 前向声明
class Graph;

namespace GraphUtils {

// ============================================================================
// 坐标范围结构
// ============================================================================

/**
 * @brief 坐标范围结构
 */
struct CoordRange {
    float minX;
    float maxX;
    float minY;
    float maxY;
};

// ============================================================================
// 坐标转换函数
// ============================================================================

/**
 * @brief 将经纬度坐标转换为球面3D坐标
 * @param v3 输入的经纬度坐标 (x=纬度, y=经度, z=高度)
 * @return 球面3D坐标
 */
inline osg::Vec3 vec3ToSphere(const osg::Vec3& v3) {
    // v3.x() 是纬度，v3.y() 是经度
    float lat = osg::DegreesToRadians(v3.x()); // 纬度转换为弧度
    float lon = osg::DegreesToRadians(v3.y()); // 经度转换为弧度

    float h = osg::WGS_84_RADIUS_POLAR + v3.z(); // 固定为地球半径，单位为米

    osg::Vec3 ret;
    ret.z() = h * sinf(lat); // 根据纬度计算 Z 坐标

    h = h * cosf(lat); // 根据纬度调整水平投影的半径

    ret.y() = h * sinf(lon); // 根据经度计算 Y 坐标
    ret.x() = h * cosf(lon); // 根据经度计算 X 坐标

    return ret;
}

/**
 * @brief 将球面3D坐标转换为经纬度坐标
 * @param sphere 球面3D坐标
 * @return 经纬度坐标 (x=纬度, y=经度, z=高度)
 */
inline osg::Vec3 sphereToVec3(const osg::Vec3& sphere) {
    // 固定的地球半径
    float earthRadius = osg::WGS_84_RADIUS_POLAR;

    // 计算出地心到球面点的实际半径 h
    float h = sqrtf(sphere.x() * sphere.x() + sphere.y() * sphere.y() + sphere.z() * sphere.z());

    // 根据高度计算地球表面的距离
    float altitude = h - earthRadius;

    // 计算纬度 lat = asin(z / h)
    float lat = asinf(sphere.z() / h);

    // 计算经度 lon = atan2(y, x)
    float lon = atan2f(sphere.y(), sphere.x());

    // 将纬度和经度从弧度转换为角度
    lat = osg::RadiansToDegrees(lat);
    lon = osg::RadiansToDegrees(lon);

    // 返回值为纬度、经度和高度
    return osg::Vec3(lat, lon, altitude); // z 分量为高度
}

/**
 * @brief 将球面3D坐标转换为经纬度 (角度制)
 * @param point 球面3D坐标
 * @param lon 输出经度 (度)
 * @param lat 输出纬度 (度)
 * @param offset 高度偏移量 (默认60000.0)
 */
inline void sphereToLatLon(const osg::Vec3& point, double& lon, double& lat, double offset = 60000.0) {
    double radius = osg::WGS_84_RADIUS_POLAR + offset;
    double x = point.x();
    double y = point.y();
    double z = point.z();

    // 计算纬度
    lat = std::asin(z / radius);
    // 计算经度
    lon = std::atan2(y, x);

    // 将弧度转换为角度
    lat = osg::RadiansToDegrees(lat);
    lon = osg::RadiansToDegrees(lon);
}

/**
 * @brief 将经纬度坐标转换为世界坐标
 * @param lat 纬度 (度)
 * @param lon 经度 (度)
 * @return 世界坐标
 */
inline osg::Vec3 latLonToWorldPos(double lat, double lon) {
    // 将经纬度转换为世界坐标（球面坐标）
    float latRad = osg::DegreesToRadians(lat);
    float lonRad = osg::DegreesToRadians(lon);
    
    float h = osg::WGS_84_RADIUS_POLAR; // 地球半径
    
    osg::Vec3 ret;
    ret.z() = h * sinf(latRad);
    h = h * cosf(latRad);
    ret.y() = h * sinf(lonRad);
    ret.x() = h * cosf(lonRad);
    
    return ret;
}

// ============================================================================
// 颜色转换函数
// ============================================================================

/**
 * @brief 将十六进制颜色字符串转换为RGB浮点值
 * @param hex 十六进制颜色字符串 (格式: "#RRGGBB" 或 "RRGGBB")
 * @param r 输出红色分量 [0.0, 1.0]
 * @param g 输出绿色分量 [0.0, 1.0]
 * @param b 输出蓝色分量 [0.0, 1.0]
 */
inline void hexToRGBf(const std::string& hex, float& r, float& g, float& b) {
    std::string hexCode = (hex[0] == '#') ? hex.substr(1) : hex;

    // 假设 hexCode 长度始终为 6，且合法
    int ri = std::stoi(hexCode.substr(0, 2), nullptr, 16);
    int gi = std::stoi(hexCode.substr(2, 2), nullptr, 16);
    int bi = std::stoi(hexCode.substr(4, 2), nullptr, 16);

    r = ri / 255.0f;
    g = gi / 255.0f;
    b = bi / 255.0f;
}

/**
 * @brief 将RGB浮点值转换为十六进制颜色字符串
 * @param r 红色分量 [0.0, 1.0]
 * @param g 绿色分量 [0.0, 1.0]
 * @param b 蓝色分量 [0.0, 1.0]
 * @return 十六进制颜色字符串 (格式: "#RRGGBB")
 */
inline std::string rgbToHex(float r, float g, float b) {
    // 将 [0.0, 1.0] 范围的 r, g, b 转换为 [0, 255] 范围
    int ri = static_cast<int>(r * 255);
    int gi = static_cast<int>(g * 255);
    int bi = static_cast<int>(b * 255);

    // 使用 ostringstream 拼接每个分量的 16 进制字符串
    std::ostringstream oss;
    oss << "#" 
        << (ri < 16 ? "0" : "") << std::hex << ri 
        << (gi < 16 ? "0" : "") << std::hex << gi
        << (bi < 16 ? "0" : "") << std::hex << bi;

    return oss.str();
}

/**
 * @brief 根据索引生成预定义颜色
 * @param index 颜色索引
 * @return RGBA颜色向量
 */
inline osg::Vec4 generateColor(float index) {
    std::vector<osg::Vec4> predefinedColors = {
        osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), // 红色
        osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f), // 绿色
        osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f), // 蓝色
        osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f), // 黄色
        osg::Vec4(0.0f, 1.0f, 1.0f, 1.0f), // 青色
        osg::Vec4(1.0f, 0.0f, 1.0f, 1.0f), // 品红色
        osg::Vec4(0.5f, 0.5f, 0.5f, 1.0f), // 灰色
        osg::Vec4(1.0f, 0.5f, 0.0f, 1.0f), // 橙色
        osg::Vec4(0.5f, 0.0f, 0.5f, 1.0f), // 紫色
        osg::Vec4(0.0f, 0.5f, 0.5f, 1.0f), // 深青色
        osg::Vec4(0.3f, 0.3f, 0.7f, 1.0f)  // 其他颜色
    };

    if (index < 0)
        index = 0;
    if (index >= predefinedColors.size())
        index = predefinedColors.size() - 1;
    
    return predefinedColors[static_cast<size_t>(index)];
}

// ============================================================================
// 几何计算函数
// ============================================================================

/**
 * @brief 计算两个包围盒的重叠距离
 * @param bb1 第一个包围盒
 * @param bb2 第二个包围盒
 * @return 重叠距离向量
 */
inline osg::Vec3 calculateOverlapDistance(const osg::BoundingBox& bb1, const osg::BoundingBox& bb2) {
    float overlapY = std::min(bb1.yMax(), bb2.yMax()) - std::max(bb1.yMin(), bb2.yMin());
    float overlapZ = std::min(bb1.zMax(), bb2.zMax()) - std::max(bb1.zMin(), bb2.zMin());
    return osg::Vec3(0.0f, overlapY, overlapZ);
}

/**
 * @brief 检查两个包围盒是否重叠
 * @param bb1 第一个包围盒
 * @param bb2 第二个包围盒
 * @return 如果重叠返回true，否则返回false
 */
inline bool checkOverlap(const osg::BoundingBox& bb1, const osg::BoundingBox& bb2) {
    return !(bb1.zMax() < bb2.zMin() || bb1.zMin() > bb2.zMax() || 
             bb1.yMax() < bb2.yMin() || bb1.yMin() > bb2.yMax());
}

/**
 * @brief 找到满足条件的最优高度
 * @param p 参数p
 * @param max 最大值
 * @param heightArray 高度数组
 * @param maxHeightInArray 数组中的最大高度
 * @return 最优高度值
 */
inline float findOptimalHeight(float p, float max, const std::vector<float>& heightArray,
                       float maxHeightInArray) {
    float h_max = maxHeightInArray * 1.1f; // 最大顶点高度

    // 检查是否满足条件
    while (true) {
        // 计算系数 a1 和 a2
        float a1 = (heightArray[0] - h_max) / (p * p);
        float a2 = (heightArray.back() - h_max) / ((max - p) * (max - p));

        // 检查是否满足上界条件
        bool valid = true;
        for (size_t i = 0; i < heightArray.size(); ++i) {
            float x = static_cast<float>(i);
            float f_x;
            if (x <= p) {
                f_x = a1 * (x - p) * (x - p) + h_max;
            } else {
                f_x = a2 * (x - p) * (x - p) + h_max;
            }
            if (f_x < heightArray[i]) {
                valid = false;
                break;
            }
        }

        // 如果满足条件，返回 h'
        if (valid) {
            return h_max;
        }

        // 否则增加 h'
        h_max *= 1.1f; // 增加 10%
    }

    return h_max;
}

// ============================================================================
// 范围计算函数
// ============================================================================

/**
 * @brief 获取图的坐标范围
 * @param graph 图数据指针
 * @return 坐标范围
 */
inline CoordRange getCoordRange(const std::shared_ptr<Graph> graph) {
    CoordRange range = {
        std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()
    };

    for (const auto& node : graph->getNodes()) {
        if (node.second.pos.x < range.minX)
            range.minX = node.second.pos.x;
        if (node.second.pos.x > range.maxX)
            range.maxX = node.second.pos.x;
        if (node.second.pos.y < range.minY)
            range.minY = node.second.pos.y;
        if (node.second.pos.y > range.maxY)
            range.maxY = node.second.pos.y;
    }

    return range;
}
// 调整文字位置以避免重叠
inline void adjustTextPosition(std::vector<osg::ref_ptr<osgText::Text>> &texts, float nodeGeomSize,
                        osg::ref_ptr<osg::Camera> camera) {
    // 定义网格大小和网格数量
    const int GRID_SIZE = 10;
    const int GRID_COLS = 100;
    const int GRID_ROWS = 100;

    // 创建空间网格
    std::vector<std::vector<std::vector<int>>> spatialGrid(
        GRID_ROWS, std::vector<std::vector<int>>(GRID_COLS));

    // 检查相机
    if (!camera || !camera->getViewport())
        return;

    // 获取必要的矩阵
    osg::Matrix viewMatrix = camera->getViewMatrix();
    osg::Matrix projectionMatrix = camera->getProjectionMatrix();
    osg::Viewport *viewport = camera->getViewport();

    // 辅助函数：将世界坐标转换为屏幕坐标
    auto worldToScreen = [&](const osg::Vec3 &worldPos) -> osg::Vec3 {
        // 视图变换
        osg::Vec4 viewPos = osg::Vec4(worldPos, 1.0f) * viewMatrix;

        // 投影变换
        osg::Vec4 clipPos = viewPos * projectionMatrix;

        // 透视除法
        if (clipPos.w() != 0.0) {
            clipPos.x() /= clipPos.w();
            clipPos.y() /= clipPos.w();
            clipPos.z() /= clipPos.w();
        }

        // 视口变换
        return osg::Vec3((clipPos.x() * 0.5f + 0.5f) * viewport->width() + viewport->x(),
                         (clipPos.y() * 0.5f + 0.5f) * viewport->height() + viewport->y(),
                         clipPos.z());
    };
    auto getTextScreenBoundingBox = [&](osgText::Text *text,
                                        osg::Camera *camera) -> osg::BoundingBox {
        if (!text || !camera || !camera->getViewport())
            return osg::BoundingBox();
        // 获取世界位置和字符高度
        osg::Vec3 worldPos = text->getPosition();
        float charHeight = text->getCharacterHeight();

        // 估算屏幕高度（从当前位置向上一个字符高度）
        osg::Vec3 worldTop = worldPos + osg::Vec3(0.0f, 0.0f, charHeight);
        float pixelHeight = fabs(worldToScreen(worldTop).y() - worldToScreen(worldPos).y());

        // 获取文字长度
        std::string content = text->getText().createUTF8EncodedString();
        size_t charCount = content.length();

        // 估算宽高比（英文字符为0.5~0.6，中文字符更接近1.0）
        float aspectRatio = 0.6f;
        float pixelWidth = pixelHeight * aspectRatio * charCount;

        // 左下角屏幕坐标
        osg::Vec3 screenOrigin = worldToScreen(worldPos);

        // 构造屏幕空间包围盒
        osg::BoundingBox screenBB;
        screenBB.set(screenOrigin.x(), screenOrigin.y(), 0.0f, screenOrigin.x() + pixelWidth,
                     screenOrigin.y() + pixelHeight, 0.0f);
        // 增加边界的偏移量，单位为像素
        float extraPadding = 0.0f; // 根据需要调整这个值
        // 创建一个新的包围盒，在原包围盒的基础上加上额外的边界
        screenBB._min -= osg::Vec3(extraPadding, extraPadding, 0.0f); // 左下角扩展
        screenBB._max += osg::Vec3(extraPadding, extraPadding, 0.0f); // 右上角扩展
        return screenBB;
    };

    struct TextInfo {
        osg::BoundingBox screenBB;
        osg::Vec3 originalPos;
        osg::Vec3 screenPos;
        int gridRow;
        int gridCol;
        bool isVisible;
    };
    std::vector<TextInfo> textInfos(texts.size());

    // 第一步：转换所有文字到屏幕坐标并计算网格位置
    for (size_t i = 0; i < texts.size(); ++i) {
        auto &text = texts[i];
        auto &info = textInfos[i];
        info.originalPos = text->getPosition();
        info.screenPos = worldToScreen(info.originalPos);
        info.screenBB = getTextScreenBoundingBox(text.get(), camera.get());

        // 计算网格位置
        info.gridRow = static_cast<int>(info.screenPos.y() / GRID_SIZE);
        info.gridCol = static_cast<int>(info.screenPos.x() / GRID_SIZE);

        // 确保网格索引在有效范围内
        info.gridRow = osg::clampBetween(info.gridRow, 0, GRID_ROWS - 1);
        info.gridCol = osg::clampBetween(info.gridCol, 0, GRID_COLS - 1);

        // 将文字索引添加到对应的网格中
        spatialGrid[info.gridRow][info.gridCol].push_back(i);

        info.isVisible = true;
    }
    // 反投影
    osg::Matrix VPInv =
        osg::Matrix::inverse(camera->getViewMatrix() * camera->getProjectionMatrix());
    auto screenToWorld = [&](const osg::Vec3 &screen) -> osg::Vec3 {
        float x = (screen.x() - viewport->x()) / viewport->width() * 2.0f - 1.0f;
        float y = (screen.y() - viewport->y()) / viewport->height() * 2.0f - 1.0f;
        float z = screen.z();

        osg::Vec4 ndc(x, y, z, 1.0f);
        osg::Vec4 world = ndc * VPInv;
        if (world.w() != 0.0f)
            world /= world.w();
        return osg::Vec3(world.x(), world.y(), world.z());
    };

    // 第二步：处理碰撞
    for (size_t i = 0; i < texts.size(); ++i) {
        auto &info = textInfos[i];
        if (!info.isVisible)
            continue;
        // 获取文字的屏幕空间包围盒
        float pixelWidth = info.screenBB._max.x() - info.screenBB._min.x(); // 计算文字宽度（像素）

        const float MAX_OFFSET = (pixelWidth) / 1000.0f; // 最大偏移距离
        const float STEP = MAX_OFFSET / 2.0f;            // 每次尝试偏移的步长
        // 检查相邻网格中的文字
        bool foundValidPosition = false;
        float currentOffset = 0.0f;
        osg::Vec3 bestScreenPos = info.screenPos; // 保存找到的最佳屏幕位置

        // 首先检查原始位置是否有碰撞
        bool hasInitialCollision = false;
        for (int dr = -1; dr <= 1 && !hasInitialCollision; ++dr) {
            for (int dc = -1; dc <= 1 && !hasInitialCollision; ++dc) {
                int checkRow = info.gridRow + dr;
                int checkCol = info.gridCol + dc;

                if (checkRow < 0 || checkRow >= GRID_ROWS || checkCol < 0 || checkCol >= GRID_COLS)
                    continue;

                for (int idx : spatialGrid[checkRow][checkCol]) {
                    if (idx == i)
                        continue;

                    if (info.screenBB.intersects(textInfos[idx].screenBB)) {
                        hasInitialCollision = true;
                        std::cout << "Checking text " << i << " with screenBB: " << idx
                                  << std::endl;
                        break;
                    }
                }
            }
        }

        // 如果原始位置没有碰撞，直接使用
        if (!hasInitialCollision) {
            foundValidPosition = true;
        } else {
            // 尝试不同的偏移位置
            while (currentOffset <= MAX_OFFSET && !foundValidPosition) {
                // 尝试8个方向的偏移
                for (int angle = 0; angle < 8; ++angle) {
                    float theta = angle * osg::PI_4;
                    osg::Vec3 screenOffset(cos(theta) * currentOffset, sin(theta) * currentOffset,
                                           0.0f);

                    osg::Vec3 testPos = info.screenPos + screenOffset;
                    osg::BoundingBox testBB = info.screenBB;
                    testBB._min += screenOffset;
                    testBB._max += screenOffset;

                    bool hasCollision = false;

                    // 检查周围网格
                    int testGridRow = static_cast<int>(testPos.y() / GRID_SIZE);
                    int testGridCol = static_cast<int>(testPos.x() / GRID_SIZE);

                    for (int dr = -1; dr <= 1 && !hasCollision; ++dr) {
                        for (int dc = -1; dc <= 1 && !hasCollision; ++dc) {
                            int checkRow = testGridRow + dr;
                            int checkCol = testGridCol + dc;

                            if (checkRow < 0 || checkRow >= GRID_ROWS || checkCol < 0 ||
                                checkCol >= GRID_COLS)
                                continue;

                            for (int idx : spatialGrid[checkRow][checkCol]) {
                                if (idx == i)
                                    continue;

                                if (testBB.intersects(textInfos[idx].screenBB)) {
                                    hasCollision = true;
                                    break;
                                }
                            }
                        }
                    }

                    if (!hasCollision) {
                        bestScreenPos = testPos;
                        foundValidPosition = true;
                        break;
                    }
                }

                currentOffset += STEP;
            }
        }

        if (foundValidPosition) {
            // 计算屏幕空间的偏移量
            // osg::Vec3 totalScreenOffset = bestScreenPos - info.screenPos;
            //// 根据屏幕偏移量计算世界空间的偏移
            // float scale = info.originalPos.length() * 0.001f;
            // osg::Vec3 worldOffset(totalScreenOffset.x() * scale, totalScreenOffset.y() * scale,
            //                       0.0f);

            //// 设置新位置
            // texts[i]->setPosition(info.originalPos + worldOffset);

            osg::Vec3 screenFrom = info.screenPos;
            osg::Vec3 screenTo = bestScreenPos;

            osg::Vec3 worldFrom = screenToWorld(screenFrom);
            osg::Vec3 worldTo = screenToWorld(screenTo);

            osg::Vec3 worldOffset = worldTo - worldFrom;
            texts[i]->setPosition(info.originalPos + worldOffset);

            // 更新网格
            spatialGrid[info.gridRow][info.gridCol].erase(
                std::remove(spatialGrid[info.gridRow][info.gridCol].begin(),
                            spatialGrid[info.gridRow][info.gridCol].end(), i),
                spatialGrid[info.gridRow][info.gridCol].end());
            // 重新计算网格位置
            info.screenPos = worldToScreen(info.originalPos + worldOffset); // 更新新的屏幕坐标
            info.screenBB =
                getTextScreenBoundingBox(texts[i].get(), camera.get()); // 重新计算包围盒

            info.gridRow = static_cast<int>(info.screenPos.y() / GRID_SIZE);
            info.gridCol = static_cast<int>(info.screenPos.x() / GRID_SIZE);

            // 确保网格索引在有效范围内
            info.gridRow = osg::clampBetween(info.gridRow, 0, GRID_ROWS - 1);
            info.gridCol = osg::clampBetween(info.gridCol, 0, GRID_COLS - 1);

            // 更新网格
            spatialGrid[info.gridRow][info.gridCol].push_back(i);
        } else {
            // 如果没找到合适的位置，隐藏文字
            texts[i]->setColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
            info.isVisible = false;
            // 从原来的网格中删除它
            spatialGrid[info.gridRow][info.gridCol].erase(
                std::remove(spatialGrid[info.gridRow][info.gridCol].begin(),
                            spatialGrid[info.gridRow][info.gridCol].end(), i),
                spatialGrid[info.gridRow][info.gridCol].end());
        }
    }
}

} // namespace GraphUtils
} // namespace VIS4Earth

#endif // VIS4EARTH_GRAPH_VISER_GRAPH_DISPLAY_UTILS_H
