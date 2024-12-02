#include <iostream>

#include <QtWidgets/QApplication>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>

#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

#include <vis4earth/graph_viser/NodeClickHandler.h>
#include <vis4earth/graph_viser/graph_display.h>
#include <vis4earth/graph_viser/markManager.h>

class UpdateMarkersCallback : public osg::Camera::DrawCallback {
  public:
    UpdateMarkersCallback(EarthMarkerManager *manager) : _manager(manager) {}

    virtual void operator()(osg::RenderInfo &) const override { _manager->updateMarkers(); }

  private:
    EarthMarkerManager *_manager;
};

int main(int argc, char **argv) {
    QApplication app(argc, argv);

    auto *viewer = new osgViewer::Viewer;
    viewer->setUpViewInWindow(200, 50, 1000, 1000);
    auto *manipulator = new osgGA::TrackballManipulator;
    viewer->setCameraManipulator(manipulator);

    osg::ref_ptr<osg::Group> grp = new osg::Group;
    grp->addChild(VIS4Earth::CreateEarth());

    osg::ref_ptr<osg::Camera> camera = viewer->getCamera();

    // 创建地球和标记管理器
    EarthMarkerManager manager(grp, camera);
    // 加载标记
    manager.loadMarkers(DATA_PATH_PREFIX "usairportsfull.csv");
    // 启动后台线程
    manager.startBackgroundThread();
    // 动态更新标记
    viewer->getCamera()->setFinalDrawCallback(new UpdateMarkersCallback(&manager));

    // VIS4Earth::GraphRenderer *graphLayout = new VIS4Earth::GraphRenderer;

    // grp->addChild(graphLayout->getGroup());
    // graphLayout->show();

    //// 添加点击事件
    // VIS4Earth::NodeClickHandler *nodeClickHandler =
    //     new VIS4Earth::NodeClickHandler(graphLayout, viewer);
    //// 将 NodeClickHandler 添加到 Viewer 的事件处理器中
    // viewer->addEventHandler(nodeClickHandler);

    viewer->setSceneData(grp);
    auto prevClk = clock();
    while (!viewer->done()) {
        auto currClk = clock();
        auto duration = currClk - prevClk;

        app.processEvents();

        if (duration >= CLOCKS_PER_SEC / 45) {
            viewer->frame();
            prevClk = clock();
        }
    }
    // 停止后台线程
    manager.stopBackgroundThread();

    return 0;
}