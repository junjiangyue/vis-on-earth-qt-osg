#ifndef NODECLICKHANDLER_H
#define NODECLICKHANDLER_H

#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <set>
#include <string>
#include <vis4earth/graph_viser/graph_display.h>

namespace VIS4Earth {

class CameraMovementCallback : public osg::NodeCallback {
  public:
    // 构造函数注入GraphRenderer指针
    explicit CameraMovementCallback(VIS4Earth::GraphRenderer *graphRenderer)
        : _graphRenderer(graphRenderer) {}
    virtual void operator()(osg::Node *node, osg::NodeVisitor *nv) {
        // 获取当前相机
        osg::Camera *camera = dynamic_cast<osg::Camera *>(node);
        if (!camera)
            return;

        // 计算相机变化
        checkCameraMovement(camera);

        // 继续场景遍历
        traverse(node, nv);
    }
    // 优化后的经纬度范围计算函数
    void calculateEarthIntersection(osg::Camera *camera, const osg::Vec3d &eyePos, double &minLon,
                                    double &maxLon, double &minLat, double &maxLat) {
        // 获取视锥体的8个顶点
        osg::Matrixd viewMatrix = camera->getViewMatrix();
        osg::Matrixd projMatrix = camera->getProjectionMatrix();
        osg::Matrixd invViewProj = osg::Matrixd::inverse(viewMatrix * projMatrix);

        std::vector<osg::Vec3d> projPoints;
        projPoints.reserve(9); // 8个顶点+相机位置

        // 添加相机位置的投影
        double eyeDistance = eyePos.length();
        if (eyeDistance > osg::WGS_84_RADIUS_POLAR) {
            osg::Vec3d projEye = eyePos;
            projEye.normalize();
            projEye *= osg::WGS_84_RADIUS_POLAR;
            projPoints.push_back(projEye);
        }

        // 计算视锥体的8个角点
        const osg::Vec3d corners[8] = {osg::Vec3d(-1.0, -1.0, -1.0), osg::Vec3d(1.0, -1.0, -1.0),
                                       osg::Vec3d(1.0, 1.0, -1.0),   osg::Vec3d(-1.0, 1.0, -1.0),
                                       osg::Vec3d(-1.0, -1.0, 1.0),  osg::Vec3d(1.0, -1.0, 1.0),
                                       osg::Vec3d(1.0, 1.0, 1.0),    osg::Vec3d(-1.0, 1.0, 1.0)};

        // 初始化范围
        minLon = 180.0;
        maxLon = -180.0;
        minLat = 90.0;
        maxLat = -90.0;
        bool hasValidPoints = false;

        // 处理每个视锥体顶点
        for (const auto &corner : corners) {
            osg::Vec3d worldPoint = corner * invViewProj;

            // 计算射线与球体交点
            osg::Vec3d dir = worldPoint - eyePos;
            dir.normalize();

            // 射线-球体相交检测
            double a = dir * dir;
            double b = 2.0 * (eyePos * dir);
            double c = (eyePos * eyePos) - (osg::WGS_84_RADIUS_POLAR * osg::WGS_84_RADIUS_POLAR);
            double discriminant = b * b - 4.0 * a * c;

            if (discriminant >= 0.0) {
                double t = (-b - sqrt(discriminant)) / (2.0 * a);
                if (t > 0.0) {
                    osg::Vec3d intersectPoint = eyePos + dir * t;

                    // 计算经纬度
                    double lon = atan2(intersectPoint.y(), intersectPoint.x()) * 180.0 / osg::PI;
                    double lat =
                        asin(intersectPoint.z() / osg::WGS_84_RADIUS_POLAR) * 180.0 / osg::PI;

                    // 更新范围
                    minLon = std::min(minLon, lon);
                    maxLon = std::max(maxLon, lon);
                    minLat = std::min(minLat, lat);
                    maxLat = std::max(maxLat, lat);
                    hasValidPoints = true;
                }
            }
        }

        if (!hasValidPoints) {
            // 如果没有有效的交点，返回默认范围
            minLon = -180.0;
            maxLon = 180.0;
            minLat = -90.0;
            maxLat = 90.0;
            return;
        }

        // 添加余量
        double lonMargin = (maxLon - minLon) * 0.1;
        double latMargin = (maxLat - minLat) * 0.1;

        minLon = std::max(-180.0, minLon - lonMargin);
        maxLon = std::min(180.0, maxLon + lonMargin);
        minLat = std::max(-90.0, minLat - latMargin);
        maxLat = std::min(90.0, maxLat + latMargin);

        // 处理经度跨越180度的情况
        if (maxLon - minLon > 350.0) {
            minLon = -180.0;
            maxLon = 180.0;
        }
    }

    void checkCameraMovement(osg::Camera *camera) {

        static double lastHeight;
        static osg::Vec3d lastEyePosition;
        static osg::Vec3d lastCenter;
        static bool isFirstCheck = true;
        
        // 获取当前相机的世界坐标位置（直接从相机矩阵中提取）
        osg::Matrixd viewMatrix = camera->getViewMatrix();
        osg::Vec3d eyePosition, center, up;
        camera->getViewMatrixAsLookAt(eyePosition, center, up);

        // 计算相机高度
        double R_earth = 6371.0;                // 地球半径，单位：公里
        double distance = eyePosition.length(); // 相机到地球中心的距离
        double height = distance - R_earth; // 相机到地球表面的高度（单位：公里）

        // 计算相机位置变化
        bool heightChanged = abs(height - lastHeight) > 1000.0;
        bool positionChanged = false;
        bool viewDirectionChanged = false;
        
        if (!isFirstCheck) {
            // 计算位置变化（距离）
            double positionDistance = (eyePosition - lastEyePosition).length();
            positionChanged = positionDistance > 500000.0; // 500km的位置变化阈值
            
            // 计算视角方向变化
            osg::Vec3d currentViewDir = center - eyePosition;
            osg::Vec3d lastViewDir = lastCenter - lastEyePosition;
            currentViewDir.normalize();
            lastViewDir.normalize();
            
            // 计算两个方向向量的夹角
            double dotProduct = currentViewDir * lastViewDir;
            dotProduct = std::max(-1.0, std::min(1.0, dotProduct)); // 限制在[-1,1]范围内
            double angle = acos(dotProduct) * 180.0 / osg::PI; // 转换为角度
            
            viewDirectionChanged = angle > 5.0; // 5度的视角变化阈值
        }

        // 当高度、位置或视角发生显著变化时更新LOD
        if (heightChanged || positionChanged || viewDirectionChanged || isFirstCheck) {
            if (!camera || !_graphRenderer)
                return;
                
            std::cout << "Camera movement detected - Height: " << heightChanged 
                      << ", Position: " << positionChanged 
                      << ", ViewDirection: " << viewDirectionChanged << std::endl;
                      
            _graphRenderer->updateActiveLOD(height);
            if (firstDraw) {
                _graphRenderer->cameraUpdate("LoadedGraph", height); // 调用外部对象的更新方法
                firstDraw = false;
            }
               
            std::cout << "updatecheck" << std::endl;
        }

        // 更新记录
        lastHeight = height;
        lastEyePosition = eyePosition;
        lastCenter = center;
        isFirstCheck = false;
    }

  private:
    VIS4Earth::GraphRenderer *_graphRenderer; // 弱引用（不管理生命周期）
    bool firstDraw = true;
};
class NodeClickHandler : public osgGA::GUIEventHandler {
  public:
    NodeClickHandler(GraphRenderer *graphRenderer, osgViewer::Viewer *viewer);

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

  private:
    GraphRenderer *graphRenderer;
    osgViewer::Viewer *viewer;
    std::set<std::string> collapsedNodes;

    void collapseNode(const std::string &nodeId);
    void expandNode(const std::string &nodeId);
    std::vector<std::string> getNeighbors(const std::string &nodeId);
    void setNodeVisible(const std::string &nodeId, bool visible);
    void setEdgeVisible(const std::string &from, const std::string &to, bool visible);
    std::string getNodeIdFromSphere(osg::ShapeDrawable *sphere);
};

} // namespace VIS4Earth

#endif // NODECLICKHANDLER_H