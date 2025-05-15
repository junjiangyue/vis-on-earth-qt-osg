#include <iostream>

#include <QtWidgets/QApplication>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>

#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

#include <osgViewer/ViewerEventHandlers>
#include <vis4earth/graph_viser/markManager.h>
#include <vis4earth/graph_viser/nodeHoverHandler.h>
#include <vis4earth/graph_viser/resizeWindowHandler.h>
#include <vis4earth/osg_util.h>

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
    EarthMarkerManager *manager = new EarthMarkerManager(grp, camera);
    manager->loadMarkers(DATA_PATH_PREFIX "usairportsfull.csv");
    // manager->updateMarkers();

    viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);

    viewer->getCamera()->setFinalDrawCallback(new UpdateMarkersCallback(manager));
    HoverEventHandler *nodeHoverHandler = new HoverEventHandler(manager, viewer);
    ResizeHandler *resizeHandler = new ResizeHandler(manager, viewer);
    viewer->addEventHandler(nodeHoverHandler);
    viewer->addEventHandler(resizeHandler);
    auto pStatsEventHandler = new osgViewer::StatsHandler; // 构造一视景器统计事件处理器
    viewer->addEventHandler(pStatsEventHandler); // 向视景器增加统计事件处理器

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

    return 0;
}