#ifndef VIS4EARTH_GRAPH_VISER_GRAPH_DRAW_H
#define VIS4EARTH_GRAPH_VISER_GRAPH_DRAW_H

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/LineWidth>
#include <osg/MatrixTransform>
#include <osg/Node>
#include <osg/ShapeDrawable>
#include <osg/Vec3>

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
namespace VIS4Earth {
class CityLoader {
  public:
    CityLoader() = default;

    // 读取CSV文件中的建筑物 OBB 数据
    bool loadBuildingsFromCSV(const std::string &filename) {
        std::ifstream file(filename);
        std::string line;

        while (std::getline(file, line)) {
            std::vector<osg::Vec3> obb_points;
            std::stringstream ss(line);
            float x, y, z;

            for (int i = 0; i < 8; ++i) {
                // 读取每三个值，表示一个坐标
                ss >> x;
                ss.ignore(); // 忽略分隔符（逗号）
                ss >> y;
                ss.ignore(); // 忽略分隔符（逗号）
                ss >> z;
                if (i < 7)
                    ss.ignore();
                obb_points.push_back(osg::Vec3(x, y, z));
            }

            buildings.push_back(obb_points);
        }
        return true;
    }

    // 给定经纬度，返回该位置的高度
    float getHeightAtLatitudeLongitude(float lat, float lon) {}

    // 根据经纬度范围和缩放比例绘制建筑物
    void drawBuildings(osg::Group *root, const std::vector<std::pair<float, float>> &latLonBounds,
                       float scale) {

        for (const auto &obb : buildings) {
            std::vector<osg::Vec3> lonlatBuild;
            for (const auto &point : obb) {
                osg::Vec3 lonlatpoint = convertToLatLon(point, latLonBounds, scale);
                lonlatBuild.push_back(lonlatpoint);
            }
            coords.push_back(lonlatBuild);
            osg::ref_ptr<osg::Geode> geode = new osg::Geode();
            // 创建建筑物的 OBB 边框
            geode = createOBBBox(obb, latLonBounds, scale);

            // 将建筑物添加到场景的根节点中
            root->addChild(geode);
        }
        calculateHeightMap(coords, latLonBounds);
    }
    std::vector<std::vector<float>> getHeightMap() { return heightMap; }

  private:
    std::vector<std::vector<osg::Vec3>> buildings;
    std::vector<std::vector<osg::Vec3>> coords;
    std::vector<std::vector<float>> heightMap;
    void calculateHeightMap(std::vector<std::vector<osg::Vec3>> coords,
                            const std::vector<std::pair<float, float>> &latLonBounds) {
        // 经纬度范围
        float latMin = latLonBounds[0].first;
        float lonMin = latLonBounds[0].second;
        float latMax = latLonBounds[1].first;
        float lonMax = latLonBounds[1].second;
        int size = 100;
        std::vector<std::vector<float>> myheightMap(size, std::vector<float>(size, 0.0f));
        // 遍历 coords 中的每个建筑物（假设建筑物有 8 个顶点）
        for (const auto &row : coords) {
            // 获取每个建筑物的后四个顶点
            const osg::Vec3 &vertex4 = row[4];
            const osg::Vec3 &vertex5 = row[5];
            const osg::Vec3 &vertex6 = row[6];
            float height = vertex4.z();
            int rowIndexMin =
                static_cast<int>((vertex4.x() - latMin) / (latMax - latMin) * (size - 1));
            int colIndexMin =
                static_cast<int>((vertex4.y() - lonMin) / (lonMax - lonMin) * (size - 1));
            int rowIndexMax =
                static_cast<int>((vertex6.x() - latMin) / (latMax - latMin) * (size - 1));
            int colIndexMax =
                static_cast<int>((vertex5.y() - lonMin) / (lonMax - lonMin) * (size - 1));
            for (int i = rowIndexMin; i <= rowIndexMax; i++) {
                for (int j = colIndexMin; j <= colIndexMax; j++) {
                    if (i >= 0 && i < size && j >= 0 && j < size)
                        myheightMap[i][j] = std::max(myheightMap[i][j], height);
                }
            }
            // for (int i = 4; i < 8; ++i) {
            //     const osg::Vec3 &vertex = row[i]; // 顶部四个顶点
            //     float lat = vertex.x();           // 经度
            //     float lon = vertex.y();           // 纬度
            //     float height = vertex.z();        // 高度

            //    // 将经纬度映射到高度图的行列索引
            //    int rowIndex = static_cast<int>((lat - latMin) / (latMax - latMin) * (size - 1));
            //    int colIndex = static_cast<int>((lon - lonMin) / (lonMax - lonMin) * (size - 1));

            //    // 更新高度图
            //    if (rowIndex >= 0 && rowIndex < size && colIndex >= 0 && colIndex < size) {
            //        // 更新对应位置的最大高度
            //        myheightMap[rowIndex][colIndex] =
            //            std::max(myheightMap[rowIndex][colIndex], height);
            //    }
            //}
        }
        heightMap = myheightMap;
    }
    // 计算 OBB 的中心点
    osg::Vec3 calculateCenter(const std::vector<osg::Vec3> &obb) {
        osg::Vec3 center(0.0, 0.0, 0.0);
        for (const auto &point : obb) {
            center += point;
        }
        center /= obb.size();
        return center;
    }

    // 将笛卡尔坐标系下的 x, y 坐标转换为经纬度坐标
    osg::Vec3 convertToLatLon(const osg::Vec3 &cartesian,
                              const std::vector<std::pair<float, float>> &latLonBounds,
                              float scale) {
        // 获取经纬度范围的最小值和最大值
        float lat_min = latLonBounds[0].first;
        float lon_min = latLonBounds[0].second;
        float lat_max = latLonBounds[1].first;
        float lon_max = latLonBounds[1].second;

        // 获取笛卡尔坐标的 x, y, z
        float x = cartesian.x();
        float y = cartesian.y();
        float z = cartesian.z(); // 高度，不影响经纬度计算

        // 纬度和经度的比例因子（计算的经纬度范围和比例）
        float latRange = lat_max - lat_min;
        float lonRange = lon_max - lon_min;

        // 映射 x, y 到经纬度范围
        // 先将笛卡尔坐标 (x, y) 转换为相对于原点的偏移量
        float latConverted = lat_min + (y / scale); // 将y坐标映射到纬度
        float lonConverted = lon_min + (x / scale); // 将x坐标映射到经度

        // 返回转换后的经纬度（x = lat, y = lon, z = 原来的z高度）
        return osg::Vec3(latConverted, lonConverted, z);
    }

    // 创建建筑物的 OBB 边框
    osg::ref_ptr<osg::Geode> createOBBBox(const std::vector<osg::Vec3> &obb,
                                          const std::vector<std::pair<float, float>> &latLonBounds,
                                          float scale) {

        auto vec3ToSphere = [&](const osg::Vec3 &v3) -> osg::Vec3 {
            // v3.x() 是纬度，v3.y() 是经度
            float lat = osg::DegreesToRadians(v3.x()); // 纬度转换为弧度
            float lon = osg::DegreesToRadians(v3.y()); // 经度转换为弧度

            float h = 6371000.0f + v3.z(); // 固定为地球半径，单位为米

            osg::Vec3 ret;
            ret.z() = h * sinf(lat); // 根据纬度计算 Z 坐标

            h = h * cosf(lat); // 根据纬度调整水平投影的半径

            ret.y() = h * sinf(lon); // 根据经度计算 Y 坐标
            ret.x() = h * cosf(lon); // 根据经度计算 X 坐标

            return ret;
        };
        auto geom = new osg::Geometry;
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();

        for (const auto &point : obb) {
            vertices->push_back(vec3ToSphere(convertToLatLon(point, latLonBounds, scale)));
        }
        geom->setVertexArray(vertices);

        // 创建连接各顶点的线条
        osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_LINES, 24);
        for (size_t i = 0; i < 4; ++i) {
            (*indices)[i * 2 + 0] = i;
            (*indices)[i * 2 + 1] = (i + 1) % 4;
        }
        for (size_t i = 4; i < 8; ++i) {
            (*indices)[(i - 4) * 2 + 0] = i;
            (*indices)[(i - 4) * 2 + 1] = (i + 1) % 4 + 4;
        }
        for (size_t i = 0; i < 4; ++i) {
            (*indices)[8 + i * 2 + 0] = i;
            (*indices)[8 + i * 2 + 1] = i + 4;
        }

        geom->addPrimitiveSet(indices);
        // 创建一个新的 Geode 并添加几何体
        osg::ref_ptr<osg::Geode> geode = new osg::Geode();
        geode->addDrawable(geom);

        // 设置线条的颜色为红色
        osg::ref_ptr<osg::StateSet> stateSet = geode->getOrCreateStateSet();
        stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        stateSet->setMode(GL_BLEND, osg::StateAttribute::OFF); // 开启混合模式
        // 设置线条宽度
        osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth(2.0f); // 线条宽度为2
        stateSet->setAttribute(lineWidth.get());

        return geode;
    }
};

} // namespace VIS4Earth
#endif