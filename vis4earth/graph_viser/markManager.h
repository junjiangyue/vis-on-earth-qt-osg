#ifndef MARKMANAGER_H
#define MARKMANAGER_H

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <osg/Camera>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Point>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osgText/Text>
#include <sstream>
#include <thread>
#include <vector>

struct Marker {
    int id;
    std::string label;
    double lat;
    double lon;
    osg::Vec3 position;   // 用于存储球面坐标
    bool visible = true; // 用于标记是否显示标签
};
struct ViewExtent {
    double minLon, maxLon;
    double minLat, maxLat;
};

class EarthMarkerManager {
  public:
    EarthMarkerManager(osg::Group *root, osg::Camera *camera, double clusterDistance = 100.0)
        : _root(root), _camera(camera), _clusterDistance(clusterDistance) {
        // 只加载一次字体
        _font = osgText::readFontFile("fonts/arial.ttf");
    }

    ~EarthMarkerManager() { stopBackgroundThread(); }

    void loadMarkers(const std::string &filePath) {
        std::ifstream file(filePath);

        if (!file.is_open()) {
            throw std::runtime_error("Unable to open CSV file: " + filePath);
        }

        std::string line;
        bool isHeader = true; // 跳过表头

        while (std::getline(file, line)) {
            if (isHeader) {
                isHeader = false;
                continue;
            }

            std::istringstream ss(line);
            std::string token;

            Marker marker;

            // 读取 id
            std::getline(ss, token, ',');
            marker.id = std::stoi(token);

            // 读取 name
            std::getline(ss, token, ',');
            marker.label = token;

            // 读取 lon
            std::getline(ss, token, ',');
            marker.lat = std::stod(token);

            // 读取 lat
            std::getline(ss, token, ',');
            marker.lon = std::stod(token);

            marker.position = latLonToSphere(marker.lon, marker.lat);
            _markers.push_back(marker);
        }
    }

    // 启动后台线程
    void startBackgroundThread() {
        _stop = false;
        // 点的后台线程
        _markerThread = std::thread([this]() {
            while (!_stop) {
                updateMarkersInBackground();
                std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 定时更新
            }
        });
    }

    // 停止后台线程
    void stopBackgroundThread() {
        _stop = true;
        if (_markerThread.joinable())
            _markerThread.join();
    }

    // 在主线程更新标记
    void updateMarkers() {
        {
            std::lock_guard<std::mutex> lock(_markerMutex);
            _visibleMarkers = _backgroundMarkers;
        }

        // 更新点和标签
        updateLabelVisibility();
        _root->removeChildren(1, _root->getNumChildren() - 1);
        _root->addChild(createMarkerGeometry(_visibleMarkers));
        
    }

  private:
    float fontSize;
    osg::Group *_root;
    osg::Camera *_camera;
    std::vector<Marker> _markers;                               // 所有标记
    std::vector<Marker> _backgroundMarkers;                     // 后台缓冲区
    std::vector<osg::ref_ptr<osgText::Text>> _backgroundLabels; // 后台标签缓冲区
    std::vector<Marker> _visibleMarkers;                        // 前台缓冲区
    std::vector<osg::ref_ptr<osgText::Text>> _visibleLabels;    // 前台标签缓冲区
    double _clusterDistance;
    std::thread _markerThread; // 后台点更新线程
    std::mutex _markerMutex;   // 点的线程锁
    std::atomic<bool> _stop;

    osg::ref_ptr<osgText::Font> _font; // 缓存字体对象

    constexpr static double WGS_84_RADIUS_POLAR = 6356752.3142;

    // 经纬度转球面坐标
    osg::Vec3 latLonToSphere(double lon, double lat, double offset = 50000.0) {
        double radLat = osg::DegreesToRadians(lat);
        double radLon = osg::DegreesToRadians(lon);

        double radius = WGS_84_RADIUS_POLAR + offset;
        double x = radius * cos(radLat) * cos(radLon);
        double y = radius * cos(radLat) * sin(radLon);
        double z = radius * sin(radLat);
        return osg::Vec3(x, y, z);
    }

    // 获取视图范围（假设全球范围）
    ViewExtent getViewExtent() {
        //osg::Viewport *viewport = _camera->getViewport();

        //// 获取相机的投影矩阵和视图矩阵
        //osg::Matrixd projectionMatrix = _camera->getProjectionMatrix();
        //osg::Matrixd viewMatrix = _camera->getViewMatrix();

        //// 合成视图矩阵和投影矩阵
        //osg::Matrixd projectionViewMatrix = projectionMatrix * viewMatrix;

        //// 计算投影视图矩阵的逆矩阵
        //osg::Matrixd invProjectionView = projectionViewMatrix.inverse(invProjectionView);

        //// 定义屏幕的四个角的 NDC 坐标
        //osg::Vec3d lowerLeft(-1.0, -1.0, -1.0); // 左下角
        //osg::Vec3d lowerRight(1.0, -1.0, -1.0); // 右下角
        //osg::Vec3d upperLeft(-1.0, 1.0, -1.0);  // 左上角
        //osg::Vec3d upperRight(1.0, 1.0, -1.0);  // 右上角

        //// 将 NDC 坐标转换为世界坐标
        //osg::Vec3d worldLowerLeft = lowerLeft * invProjectionView;
        //osg::Vec3d worldLowerRight = lowerRight * invProjectionView;
        //osg::Vec3d worldUpperLeft = upperLeft * invProjectionView;
        //osg::Vec3d worldUpperRight = upperRight * invProjectionView;

        //// 经纬度转换函数，假设是球面坐标
        //auto latLonFromWorldCoords = [](const osg::Vec3d &worldPos) -> std::pair<double, double> {
        //    double lat = osg::RadiansToDegrees(asin(worldPos.z() / WGS_84_RADIUS_POLAR));
        //    double lon = osg::RadiansToDegrees(atan2(worldPos.y(), worldPos.x()));
        //    return {lat, lon};
        //};

        //// 将世界坐标转换为经纬度
        //std::pair<double, double> lowerLeftLatLon = latLonFromWorldCoords(worldLowerLeft);
        //double minLat = lowerLeftLatLon.first;
        //double minLon = lowerLeftLatLon.second;

        //std::pair<double, double> upperRightLatLon = latLonFromWorldCoords(worldUpperRight);
        //double maxLat = upperRightLatLon.first;
        //double maxLon = upperRightLatLon.second;

        //// 为了确保结果不出错，我们应该保证经度范围在 [-180, 180] 范围内
        //if (maxLon < minLon) {
        //    std::swap(minLon, maxLon);
        //}

        // return {minLon, maxLon, minLat, maxLat};
        return {-60.0, 60.0, -50.0, 50.0};
    }

    // 筛选视图范围内的标记
    std::vector<Marker> filterMarkersInView(const ViewExtent &extent) {
        std::vector<Marker> visibleMarkers;
        for (const auto &marker : _markers) {
            if (marker.lon >= extent.minLon && marker.lon <= extent.maxLon &&
                marker.lat >= extent.minLat && marker.lat <= extent.maxLat) {
                visibleMarkers.push_back(marker);
            }
        }
        return visibleMarkers;
    }

    // 在后台更新可见标记
    void updateMarkersInBackground() {
        auto extent = getViewExtent();
        auto filteredMarkers = filterMarkersInView(extent);

        std::lock_guard<std::mutex> lock(_markerMutex);
        _backgroundMarkers = aggregateMarkers(filteredMarkers); // 聚合标记
    }

    // 聚合标记
    std::vector<Marker> aggregateMarkers(const std::vector<Marker> &markers) {
        std::vector<Marker> aggregated;
        std::vector<bool> processed(markers.size(), false);

        for (size_t i = 0; i < markers.size(); ++i) {
            if (processed[i])
                continue;

            Marker clusterCenter = markers[i];
            bool foundCluster = false;

            // 聚合附近的标记
            for (size_t j = i + 1; j < markers.size(); ++j) {
                if (processed[j])
                    continue;

                double distance =
                    haversine(clusterCenter.lat, clusterCenter.lon, markers[j].lat, markers[j].lon);
                if (distance < _clusterDistance) {
                    clusterCenter.lat = (clusterCenter.lat + markers[j].lat) / 2.0;
                    clusterCenter.lon = (clusterCenter.lon + markers[j].lon) / 2.0;
                    processed[j] = true;
                    foundCluster = true;
                }
            }

            aggregated.push_back(clusterCenter); // 将聚合后的代表点加入结果
        }

        return aggregated;
    }

    // 计算两点间距离（单位：公里）
    double haversine(double lat1, double lon1, double lat2, double lon2) {
        const double R = 6371.0; // 地球半径
        double dLat = osg::DegreesToRadians(lat2 - lat1);
        double dLon = osg::DegreesToRadians(lon2 - lon1);

        double a = sin(dLat / 2) * sin(dLat / 2) + cos(osg::DegreesToRadians(lat1)) *
                                                       cos(osg::DegreesToRadians(lat2)) *
                                                       sin(dLon / 2) * sin(dLon / 2);

        return R * 2 * atan2(sqrt(a), sqrt(1 - a)); // 返回两个点之间的距离（公里）
    }
    
    // 计算相机与标签的距离
    float calculateDistanceToCamera(const osg::Vec3 &cameraPos, const osg::Vec3 &labelPos) {
        return (cameraPos - labelPos).length();
    }

    // 根据距离计算字体大小
    float calculateFontSize(const osg::Vec3 &cameraPos, const osg::Vec3 &labelPos,
                            float minFontSize = 5.0f, float maxFontSize = 30.0f) {
        // 计算相机与标签之间的距离
        float distance = calculateDistanceToCamera(cameraPos, labelPos);

        // 基于距离的线性插值调整字体大小
        float fontSize = maxFontSize * (distance / 9000.0f); // 距离大，字体大
        //fontSize =
        //    std::max(minFontSize, std::min(maxFontSize, fontSize)); // 确保字体大小在合理范围内

        return fontSize;
    }

    // 创建标记几何体并创建标签
    osg::ref_ptr<osg::Geode> createMarkerGeometry(const std::vector<Marker> &markers) {
        osg::ref_ptr<osg::Geode> geode = new osg::Geode();

        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
       
        // 创建标记点
        for (const auto &marker : markers) {
            osg::Vec3 position = latLonToSphere(marker.lon, marker.lat);
            vertices->push_back(position);
            colors->push_back(osg::Vec4(1.0, 0.0, 0.0, 1.0)); // 红色点
            // 计算与相机的距离并动态调整字体大小
            osg::Vec3 labelPosition = latLonToSphere(marker.lon, marker.lat, 60000.0);
            // 创建文本标签
            if (marker.visible) {
                osg::ref_ptr<osgText::Text> label = new osgText::Text();
                label->setFont(_font);
                label->setCharacterSize(fontSize); // 标签的大小
                label->setText(marker.label);
                // 设置标签的对齐方式为屏幕对齐
                label->setAxisAlignment(osgText::Text::SCREEN); // 屏幕对齐，始终面向相机
                label->setPosition(labelPosition +
                                   osg::Vec3(0.0f, 0.0f, 1000.0f)); // 设置标签位置（稍微偏移）
                label->setColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)); // 设置标签颜色为白色

                geode->addDrawable(label.get());
            }
        }

        geometry->setVertexArray(vertices);
        geometry->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
        geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, vertices->size()));
        geometry->setUseVertexBufferObjects(true);

        // 设置点的大小和抗锯齿
        osg::ref_ptr<osg::Point> pointSize = new osg::Point();
        pointSize->setSize(5.0f); // 调整点大小
        geometry->getOrCreateStateSet()->setAttributeAndModes(pointSize, osg::StateAttribute::ON);
        geometry->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        geometry->getOrCreateStateSet()->setMode(GL_POINT_SMOOTH, osg::StateAttribute::ON);

        geode->addDrawable(geometry);
        return geode;
    }

    // 更新标签的可见性并判断是否被遮挡
    void updateLabelVisibility() {
        osg::Vec3 cameraPos = _camera->getViewMatrix().getTrans(); // 获取相机位置
        fontSize = calculateFontSize(cameraPos, _visibleMarkers[0].position);
        osg::Viewport *viewport = _camera->getViewport();
        std::vector<osg::Vec3> screenPositions;

        for (auto &marker : _visibleMarkers) {
            if (!marker.visible)
                continue; // 跳过不显示标签的点

            osg::Vec3 worldPos = marker.position;
            // 计算世界坐标到屏幕坐标的转换
            osg::Matrixd MVP = _camera->getViewMatrix() * _camera->getProjectionMatrix();
            osg::Vec3 screenPos = worldPos * MVP;

            // 将标准化设备坐标(NDC)转换为屏幕坐标
            screenPos.x() = (screenPos.x() * 0.5 + 0.5) * viewport->width();
            screenPos.y() = (screenPos.y() * 0.5 + 0.5) * viewport->height();

            // 判断标签是否在屏幕范围内
            bool visible = (screenPos.x() >= 0.0 && screenPos.x() <= viewport->width()) &&
                           (screenPos.y() >= 0.0 && screenPos.y() <= viewport->height());

            if (visible) {
                // 检查标签是否与其他标签重叠
                for (const auto &otherPos : screenPositions) {
                    if (fabs(screenPos.x() - otherPos.x()) < 50.0 &&
                        fabs(screenPos.y() - otherPos.y()) < 50.0) {
                        visible = false;
                        break;
                    }
                }
            }

            // 设置标签的可见性
            marker.visible = visible;

            if (visible) {
                screenPositions.push_back(screenPos);
            }
        }
    }
};

#endif