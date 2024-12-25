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
    bool visible = true;  // 用于标记是否显示标签
    bool isHover = false; // 用于标记当前标签是否被悬浮
};
struct ViewExtent {
    double minLon, maxLon;
    double minLat, maxLat;
};

struct ScreenGrid {
    int gridWidth;               // 网格的列数
    int gridHeight;              // 网格的行数
    float cellWidth;             // 单元格宽度（像素）
    float cellHeight;            // 单元格高度（像素）
    std::vector<bool> gridCells; // 每个单元格是否已被占用

    // 初始化屏幕网格
    ScreenGrid(int screenWidth, int screenHeight, float cellWidth, float cellHeight)
        : cellWidth(cellWidth), cellHeight(cellHeight) {
        gridWidth = std::ceil(screenWidth / cellWidth);
        gridHeight = std::ceil(screenHeight / cellHeight);
        gridCells.resize(gridWidth * gridHeight, false);
    }

    // 检查某个单元格是否已被占用
    bool isOccupied(int x, int y) const {
        if (x < 0 || x >= gridWidth || y < 0 || y >= gridHeight)
            return true;
        return gridCells[y * gridWidth + x];
    }

    // 标记某个单元格为已占用
    void markOccupied(int x, int y) {
        if (x >= 0 && x < gridWidth && y >= 0 && y < gridHeight) {
            gridCells[y * gridWidth + x] = true;
        }
    }

    // 将屏幕坐标转换为网格坐标
    std::pair<int, int> screenToGrid(float screenX, float screenY) const {
        int gridX = std::floor(screenX / cellWidth);
        int gridY = std::floor(screenY / cellHeight);
        return {gridX, gridY};
    }
};


struct GridCell {
    std::vector<osg::Vec3> positions; // 存储网格内的标签位置
};

class MarkerData : public osg::Referenced {
  public:
    MarkerData(const int &id, const std::string &label, const osg::Vec3 &position)
        : id(id), label(label), position(position) {}

    int id;             // 标记的 ID
    std::string label;  // 标记的标签
    osg::Vec3 position; // 标记的位置
};

class EarthMarkerManager {
  public:
    EarthMarkerManager(osg::ref_ptr<osg::Group> root, osg::ref_ptr<osg::Camera> camera)
        : _root(root), _camera(camera) {
        // 只加载一次字体
        _font = osgText::readFontFile("fonts/arial.ttf");
    }

    ~EarthMarkerManager() {}

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
        // auto extent = getViewExtent();
        // auto filteredMarkers = filterMarkersInView(extent);
        //_visibleMarkers = filteredMarkers;
    }

    // 更新标记
    void updateMarkers() {
        auto extent = getViewExtent();
        auto filteredMarkers = filterMarkersInView(extent);
        _visibleMarkers = filteredMarkers;
        //  更新点和标签
        updateLabelVisibility();
        _root->removeChildren(1, _root->getNumChildren() - 1);
        _root->addChild(createMarkerGeometry(_visibleMarkers));
    }
    void updateHover() {
        updateLabelVisibility();
        _root->removeChildren(1, _root->getNumChildren() - 1);
        _root->addChild(createMarkerGeometry(_visibleMarkers));
    }

    osg::Group *getGroup() { return _root.get(); }
    void setGeode(osg::ref_ptr<osg::Geode> geode) { _geode = geode; }

    osg::ref_ptr<osg::Geode> getGeode() { return _geode; }
    std::vector<Marker> _visibleMarkers;

  private:
    osg::ref_ptr<osg::Geode> _geode;
    osg::ref_ptr<osg::Group> _root;
    osg::ref_ptr<osg::Camera> _camera;
    std::vector<Marker> _markers; // 所有标记
    // std::vector<Marker> _visibleMarkers;                     // 前台缓冲区
    std::vector<osg::ref_ptr<osgText::Text>> _visibleLabels; // 前台标签缓冲区
    double _clusterDistance;

    osg::ref_ptr<osgText::Font> _font; // 缓存字体对象

    constexpr static double WGS_84_RADIUS_POLAR = 6356752.3142;
    // 网格化设置
    float gridLongitudeSpan = 5.0f; // 每个网格的经度跨度（单位：度）
    float gridLatitudeSpan = 5.0f;  // 每个网格的纬度跨度（单位：度）

    // 根据经纬度划分网格的行列
    int numLongitudeCells = 72; // 经度的网格数量，全球范围 -180 到 180 度
    int numLatitudeCells = 36;  // 纬度的网格数量，全球范围 -90 到 90 度

    int getGridRow(double lat) {
        // 纬度范围 -90 到 90，按照每个网格的纬度跨度计算网格的行
        return std::min(std::max(int((lat + 90.0) / gridLatitudeSpan), 0), numLatitudeCells - 1);
    }

    int getGridCol(double lon) {
        // 经度范围 -180 到 180，按照每个网格的经度跨度计算网格的列
        return std::min(std::max(int((lon + 180.0) / gridLongitudeSpan), 0), numLongitudeCells - 1);
    }
    // 经纬度转球面坐标
    osg::Vec3 latLonToSphere(double lon, double lat, double offset = 60000.0) {
        double radLat = osg::DegreesToRadians(lat);
        double radLon = osg::DegreesToRadians(lon);

        double radius = WGS_84_RADIUS_POLAR + offset;
        double x = radius * cos(radLat) * cos(radLon);
        double y = radius * cos(radLat) * sin(radLon);
        double z = radius * sin(radLat);
        return osg::Vec3(x, y, z);
    }

    // 获取视图范围（假设全球范围）
    ViewExtent getViewExtent() { return {-90.0, 90.0, -40.0, 40.0}; }

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

    // 创建标记几何体并创建标签
    osg::ref_ptr<osg::Geode> createMarkerGeometry(const std::vector<Marker> &markers) {
        ScreenGrid screenGrid(1000, 1000, 20.0f,
                              20.0f); // 屏幕大小为 1000x1000，网格单元大小为 50x20
        osg::ref_ptr<osg::Geode> geode = new osg::Geode();

        osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
        std::vector<osg::ref_ptr<osgText::Text>> textNodes;
        osg::ref_ptr<osg::FloatArray> vertexIDs = new osg::FloatArray(); // 用于存储点的 ID
        // 创建标记点
        for (const auto &marker : markers) {

            osg::Vec3 position = latLonToSphere(marker.lon, marker.lat);
            vertices->push_back(position);
            if (marker.isHover) {
                colors->push_back(rgbToVec4(255.0, 255.0, 224.0, 1.0));
            } else
                colors->push_back(rgbToVec4(160.0, 82.0, 45.0, 1.0));
            //
            osg::Vec3 labelPosition = latLonToSphere(marker.lon, marker.lat, 70000.0);

            // 为每个顶点创建 userData
            osg::ref_ptr<MarkerData> vertexUserData =
                new MarkerData(marker.id, marker.label, position);
            vertexIDs->push_back(static_cast<float>(marker.id));

            // 创建文本标签
            if (marker.visible) {
                osg::ref_ptr<osgText::Text> label = new osgText::Text();
                label->setFont(_font);
                label->setCharacterSize(15.0f); // 标签的大小
                label->setText(marker.label);
                label->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
                // 设置标签的对齐方式为屏幕对齐
                label->setAxisAlignment(osgText::Text::SCREEN); // 屏幕对齐，始终面向相机
                label->setPosition(labelPosition +
                                   osg::Vec3(0.0f, 0.0f, 1000.0f)); // 设置标签位置（稍微偏移）
                if (marker.isHover) {
                    label->setColor(rgbToVec4(255.0f, 250.0f, 250.0f, 1.0f));
                } else
                    label->setColor(rgbToVec4(238.0f, 180.0f, 180.0f, 1.0f));
                //label->setUserData(vertexUserData);
                //  **屏幕空间检测逻辑开始**
                osg::Vec3 screenPos =
                    projectToScreen(labelPosition, _camera); // 将标签位置投影到屏幕空间
                std::pair<int, int> gridCoords =
                    screenGrid.screenToGrid(screenPos.x(), screenPos.y());
                int gridX = gridCoords.first;
                int gridY = gridCoords.second;

                if (screenGrid.isOccupied(gridX, gridY)) {
                    // 如果该网格单元已被占用，则隐藏文字
                    label->setNodeMask(0x0); // 隐藏标签
                } else {
                    // 如果未占用，则显示文字并标记该区域为已占用
                    for (int dx = 0; dx <= 40 / 50; ++dx) {     // 40 是文字宽度的估算
                        for (int dy = 0; dy <= 15 / 20; ++dy) { // 15 是文字高度的估算
                            screenGrid.markOccupied(gridX + dx, gridY + dy);
                        }
                    }
                    label->setNodeMask(0xffffffff); // 显示标签
                }
                // **屏幕空间检测逻辑结束**
                geode->addDrawable(label.get());
                textNodes.push_back(label);
            }
        }

        adjustMarkTextPosition(textNodes);
        geometry->setVertexArray(vertices);
        geometry->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
        geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, vertices->size()));
        geometry->setUseVertexBufferObjects(true);

        // 添加顶点属性数组并绑定
        geometry->setVertexAttribArray(1, vertexIDs, osg::Array::BIND_PER_VERTEX);

        // 设置点的大小和抗锯齿
        osg::ref_ptr<osg::Point> pointSize = new osg::Point();
        pointSize->setSize(5.0f); // 调整点大小
        geometry->getOrCreateStateSet()->setAttributeAndModes(pointSize, osg::StateAttribute::ON);
        geometry->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        geometry->getOrCreateStateSet()->setMode(GL_POINT_SMOOTH, osg::StateAttribute::ON);

        geode->setName("MarkerGeode");

        geode->addDrawable(geometry);
        setGeode(geode);
        return geode;
    }

    osg::Vec3 projectToScreen(const osg::Vec3 &worldPosition, osg::Camera *camera) {
        // 获取相机的视图矩阵和投影矩阵
        osg::Matrixd viewMatrix = camera->getViewMatrix();
        osg::Matrixd projectionMatrix = camera->getProjectionMatrix();

        // 获取相机的视口信息
        osg::Viewport *viewport = camera->getViewport();
        if (!viewport) {
            throw std::runtime_error("Camera viewport is not set.");
        }
        double screenWidth = viewport->width();
        double screenHeight = viewport->height();

        // 将世界坐标转换到裁剪空间 (Clip Space)
        osg::Vec4 clipSpacePos = osg::Vec4(worldPosition, 1.0) * (viewMatrix * projectionMatrix);

        // 如果 w 分量为 0，说明点在无穷远，返回不可见的坐标
        if (clipSpacePos.w() == 0.0) {
            return osg::Vec3(-1, -1, -1); // 表示不可见点
        }

        // 从裁剪空间转换到 NDC（Normalized Device Coordinates）
        osg::Vec3 ndcPos(clipSpacePos.x() / clipSpacePos.w(), // x 坐标归一化
                         clipSpacePos.y() / clipSpacePos.w(), // y 坐标归一化
                         clipSpacePos.z() / clipSpacePos.w()  // z 坐标归一化
        );

        // 从 NDC 转换到屏幕空间
        double screenX =
            (ndcPos.x() * 0.5 + 0.5) * screenWidth; // 将 [-1, 1] 转换到 [0, screenWidth]
        double screenY =
            (ndcPos.y() * 0.5 + 0.5) * screenHeight; // 将 [-1, 1] 转换到 [0, screenHeight]

        // 返回屏幕空间坐标，z 值可用于深度检测（可选）
        return osg::Vec3(screenX, screenY, ndcPos.z());
    }

    // 检查两个矩形是否重叠，并返回重叠的距离
    osg::Vec3 calculateTextOverlapDistance(const osg::BoundingBox &bb1,
                                           const osg::BoundingBox &bb2) {
        float overlapY = std::min(bb1.yMax(), bb2.yMax()) - std::max(bb1.yMin(), bb2.yMin());
        float overlapZ = std::min(bb1.zMax(), bb2.zMax()) - std::max(bb1.zMin(), bb2.zMin());
        return osg::Vec3(0.0f, overlapY, overlapZ);
    }

    // 检查两个矩形是否重叠
    bool checkTextOverlap(const osg::BoundingBox &bb1, const osg::BoundingBox &bb2) {
        return !(bb1.zMax() < bb2.zMin() || bb1.zMin() > bb2.zMax() || bb1.yMax() < bb2.yMin() ||
                 bb1.yMin() > bb2.yMax());
    }
    // 调整文字位置以避免重叠
    void adjustMarkTextPosition(std::vector<osg::ref_ptr<osgText::Text>> &texts) {
        for (size_t i = 0; i < texts.size(); ++i) {
            osg::BoundingBox bb1 = texts[i]->getBoundingBox();
            for (size_t j = 0; j < i; ++j) {
                osg::BoundingBox bb2 = texts[j]->getBoundingBox();
                if (checkTextOverlap(bb1, bb2)) {
                    osg::Vec3 overlap = calculateTextOverlapDistance(bb1, bb2);
                    osg::Vec3 pos1 = texts[i]->getPosition();
                    osg::Vec3 pos2 = texts[j]->getPosition();

                    // y 代表上下，z 代表左右
                    if (pos1.y() < pos2.y()) {
                        pos1.y() -= overlap.y() / 2;
                        pos2.y() += overlap.y() / 2;
                    } else {
                        pos1.y() += overlap.y() / 2;
                        pos2.y() -= overlap.y() / 2;
                    }

                    if (pos1.z() < pos2.z()) {
                        pos1.z() -= overlap.z() / 2;
                        pos2.z() += overlap.z() / 2;
                    } else {
                        pos1.z() += overlap.z() / 2;
                        pos2.z() -= overlap.z() / 2;
                    }

                    texts[i]->setPosition(pos1);
                    texts[j]->setPosition(pos2);

                    bb1 = texts[i]->getBoundingBox();
                    bb2 = texts[j]->getBoundingBox();
                }
            }
        }
    }
    osg::Vec4 rgbToVec4(int r, int g, int b, float alpha = 1.0f) {
        // 检查 RGB 值范围
        if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255) {
            throw std::invalid_argument("RGB values must be in the range 0-255.");
        }

        // 检查 Alpha 值范围
        if (alpha < 0.0f || alpha > 1.0f) {
            throw std::invalid_argument("Alpha value must be in the range 0.0-1.0.");
        }

        // 将 RGB 转换为 [0.0f, 1.0f] 范围
        return osg::Vec4(r / 255.0f, g / 255.0f, b / 255.0f, alpha);
    }

    // 更新标签的可见性并判断是否被遮挡
    void updateLabelVisibility() {
        std::vector<std::vector<GridCell>> grid(numLatitudeCells,
                                                std::vector<GridCell>(numLongitudeCells));

        osg::Viewport *viewport = _camera->getViewport();
        std::vector<osg::Vec3> screenPositions;
        // 计算世界坐标到屏幕坐标的转换
        osg::Matrixd MVP = _camera->getViewMatrix() * _camera->getProjectionMatrix();
        // 可视性检测的阈值
        float overlapThreshold = 50.0f;
        for (auto &marker : _visibleMarkers) {
            if (!marker.visible)
                continue; // 跳过不显示标签的点

            osg::Vec3 worldPos = marker.position;
            osg::Vec3 screenPos = worldPos * MVP;

            // 将标准化设备坐标(NDC)转换为屏幕坐标
            screenPos.x() = (screenPos.x() * 0.5 + 0.5) * viewport->width();
            screenPos.y() = (screenPos.y() * 0.5 + 0.5) * viewport->height();
            bool visible = (screenPos.x() >= 0.0 && screenPos.x() <= viewport->width()) &&
                           (screenPos.y() >= 0.0 && screenPos.y() <= viewport->height());
            // 将世界坐标转换为经纬度
            double lat = osg::RadiansToDegrees(asin(worldPos.z() / WGS_84_RADIUS_POLAR));
            double lon = osg::RadiansToDegrees(atan2(worldPos.y(), worldPos.x()));

            // 计算标签所在网格的行列
            int gridRow = getGridRow(lat);
            int gridCol = getGridCol(lon);
            if (gridRow < 0 || gridRow >= numLatitudeCells) {
                std::cout << "chucuol ";
            }
            if (gridCol < 0 || gridCol >= numLongitudeCells) {
                std::cout << "chucuol ";
            }

            // 检查当前网格和相邻的网格
            for (int i = -1; i <= 1; ++i) {
                for (int j = -1; j <= 1; ++j) {
                    int neighborRow = gridRow + i;
                    int neighborCol = gridCol + j;

                    if (neighborRow >= 0 && neighborRow < numLatitudeCells && neighborCol >= 0 &&
                        neighborCol < numLongitudeCells) {

                        for (const auto &otherPos : grid[neighborRow][neighborCol].positions) {
                            // 检查标签之间是否有重叠
                            if (fabs(screenPos.x() - otherPos.x()) < overlapThreshold &&
                                fabs(screenPos.y() - otherPos.y()) < overlapThreshold) {
                                visible = false;
                                break;
                            }
                        }

                        if (!visible)
                            break;
                    }
                }
                if (!visible)
                    break;
            }

            // 设置标签的可见性
            marker.visible = visible;
            if (visible) {
                // 将标签位置添加到相应的网格
                grid[gridRow][gridCol].positions.push_back(screenPos);
            }
        }
    }
};

#endif