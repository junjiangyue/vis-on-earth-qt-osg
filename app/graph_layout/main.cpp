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

    void checkCameraMovement(osg::Camera *camera) {
        // 静态变量记录上一帧状态
        static osg::Vec3d lastEye, lastCenter;
        static osg::Matrixd lastViewMatrix;

        // 获取当前相机的世界坐标位置（直接从相机矩阵中提取）
        osg::Matrixd viewMatrix = camera->getViewMatrix();
        osg::Vec3d eyePosition =
            osg::Vec3d(viewMatrix(3, 0), viewMatrix(3, 1), viewMatrix(3, 2)); // 提取相机位置
        osg::Vec3d center = camera->getViewMatrix().getTrans();               // 获取目标位置

        // 计算位移变化（世界坐标系）
        double positionDelta = (eyePosition - lastEye).length();

        // 计算旋转变化（矩阵差异）
        osg::Matrixd deltaMatrix = viewMatrix * osg::Matrixd::inverse(lastViewMatrix);
        double angleChange = getRotationAngle(deltaMatrix);

        // 判断是否超过阈值
        const double POSITION_THRESHOLD = 100.0; // 单位：米
        const double ANGLE_THRESHOLD = 3.0;      // 单位：度

        // 计算相机高度
        double R_earth = 6371.0;                // 地球半径，单位：公里
        double distance = eyePosition.length(); // 相机到地球中心的距离
        double height = distance - R_earth; // 相机到地球表面的高度（单位：公里）
        // 获取当前视锥体
        osg::Polytope frustum;
        getViewFrustum(camera, frustum);

        if (positionDelta > POSITION_THRESHOLD || angleChange > ANGLE_THRESHOLD) {
            // 触发标签更新等后续操作

            if (!camera || !_graphRenderer)
                return;
            _graphRenderer->cameraUpdate("LoadedGraph", height,
                                         frustum); // 调用外部对象的更新方法

            std::cout << "updatecheck" << std::endl;
        }

        // 更新记录
        lastEye = eyePosition;
        lastCenter = center;
        lastViewMatrix = viewMatrix;
    }
    // 辅助函数：提取视锥体
    void getViewFrustum(osg::Camera *cam, osg::Polytope &frustum) {
        osg::Matrixd proj = cam->getProjectionMatrix();
        osg::Matrixd mv = cam->getViewMatrix();
        frustum.setToUnitFrustum();
        frustum.transformProvidingInverse(proj * mv);
    }
    // 辅助函数：从矩阵提取旋转角度
    double getRotationAngle(const osg::Matrixd &mat) {
        osg::Quat rot;
        mat.get(rot);

        // 正确获取四元数夹角（返回弧度值）
        double angle = 2.0 * acos(rot.w());

        // 弧度转角度（使用osg定义的PI_常量）
        return angle * 180.0 / osg::PI;
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