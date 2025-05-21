#include <iostream>

#include <QtWidgets/QApplication>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>

#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

#include <vis4earth/graph_viser/NodeClickHandler.h>
#include <vis4earth/graph_viser/graph_display.h>
#include <vis4earth/graph_viser/graph_draw.h>
#include <vis4earth/graph_viser/markManager.h>
#include <vis4earth/graph_viser/nodeHoverHandler.h>

#include <osgViewer/ViewerEventHandlers>

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

                    //// 调试输出
                    //std::cout << "Intersection point: " << intersectPoint.x() << ", "
                    //          << intersectPoint.y() << ", " << intersectPoint.z() << std::endl;
                    //std::cout << "Lon, Lat: " << lon << ", " << lat << std::endl;
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
        // 获取当前相机的世界坐标位置（直接从相机矩阵中提取）
        osg::Matrixd viewMatrix = camera->getViewMatrix();
        osg::Vec3d eyePosition, center, up;
        camera->getViewMatrixAsLookAt(eyePosition, center, up);

        // 计算相机高度
        double R_earth = 6371.0;                // 地球半径，单位：公里
        double distance = eyePosition.length(); // 相机到地球中心的距离
        double height = distance - R_earth; // 相机到地球表面的高度（单位：公里）
        
        if (abs(height - lastHeight)>1000.0) {
            // 触发标签更新等后续操作
            // 获取当前视锥体
            osg::Polytope frustum;
            //getViewFrustum(camera, frustum);
            if (!camera || !_graphRenderer)
                return;
            // 计算经纬度范围
            double minLon, maxLon, minLat, maxLat;
            _graphRenderer->cameraUpdate("LoadedGraph", height, frustum, 0, 0, 0,
                                         0); // 调用外部对象的更新方法

            std::cout << "updatecheck" << std::endl;
        }

        // 更新记录
        lastHeight = height;
    }

  private:
    VIS4Earth::GraphRenderer *_graphRenderer; // 弱引用（不管理生命周期）
};

int main(int argc, char **argv) {
    QApplication app(argc, argv);

    auto *viewer = new osgViewer::Viewer;
    viewer->setUpViewInWindow(200, 50, 1000, 1000);
    auto *manipulator = new osgGA::TrackballManipulator;
    viewer->setCameraManipulator(manipulator);

    osg::ref_ptr<osg::Group> grp = new osg::Group;
    grp->addChild(VIS4Earth::CreateEarth());

    VIS4Earth::GraphRenderer *graphLayout = new VIS4Earth::GraphRenderer;
    // 设置camera
    graphLayout->param.setCamera(viewer->getCamera());

    grp->addChild(graphLayout->getGroup());
    graphLayout->show();

    osg::ref_ptr<CameraMovementCallback> cb = new CameraMovementCallback(graphLayout);
    viewer->getCamera()->addUpdateCallback(cb);
    // 添加点击事件
    VIS4Earth::NodeClickHandler *nodeClickHandler =
        new VIS4Earth::NodeClickHandler(graphLayout, viewer);
    // 将 NodeClickHandler 添加到 Viewer 的事件处理器中
    viewer->addEventHandler(nodeClickHandler);

    auto pStatsEventHandler = new osgViewer::StatsHandler; // 构造一视景器统计事件处理器
    viewer->addEventHandler(pStatsEventHandler); // 向视景器增加统计事件处理器

    viewer->setSceneData(grp);
    auto prevClk = clock();
    while (!viewer->done()) {
        auto currClk = clock();
        auto duration = currClk - prevClk;

        app.processEvents();
        viewer->frame();
        prevClk = clock();
        // if (duration >= CLOCKS_PER_SEC / 200) {
        //
        // }
    }

    return 0;
}