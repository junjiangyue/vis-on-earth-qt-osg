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

    void checkCameraMovement(osg::Camera* camera) {
        // 静态变量记录上一帧状态
        static osg::Vec3d lastEye, lastCenter;
        static osg::Matrixd lastViewMatrix;
    
        // 获取当前状态
        osg::Vec3d eye, center, up;
        camera->getViewMatrixAsLookAt(eye, center, up);
        osg::Matrixd currentViewMatrix = camera->getViewMatrix();

        // 计算位移变化（世界坐标系）
        double positionDelta = (eye - lastEye).length();
    
        // 计算旋转变化（矩阵差异）
        osg::Matrixd deltaMatrix = currentViewMatrix * osg::Matrixd::inverse(lastViewMatrix);
        double angleChange = getRotationAngle(deltaMatrix);

        // 判断是否超过阈值
        const double POSITION_THRESHOLD = 100.0; // 单位：米
        const double ANGLE_THRESHOLD = 3.0;      // 单位：度
    
        if (positionDelta > POSITION_THRESHOLD || angleChange > ANGLE_THRESHOLD) {
            // 触发标签更新等后续操作
            //onCameraMovedSignificantly();
            if (!camera || !_graphRenderer)
                return;

            //_graphRenderer->updateLabels(); // 调用外部对象的更新方法
           
            std::cout << "updatecheck" << std::endl;
        }

        // 更新记录
        lastEye = eye;
        lastCenter = center;
        lastViewMatrix = currentViewMatrix;
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