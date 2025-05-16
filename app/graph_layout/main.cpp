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

class MouseWheelEventHandler : public osgGA::GUIEventHandler {
  public:
    MouseWheelEventHandler(VIS4Earth::GraphRenderer *graphLayout) : _graphLayout(graphLayout) {}

    // 处理鼠标滚轮事件
    virtual bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
        // 检查事件类型是否是鼠标滚轮
        if (ea.getEventType() == osgGA::GUIEventAdapter::SCROLL) {
            // 如果滚轮事件发生，更新图形布局
            if (ea.getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_UP ||
                ea.getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_DOWN) {
                // 调用 graphLayout 的更新方法 直接卡住！
                //_graphLayout->update("LoadedGraph");
                return true; // 表示事件已被处理
            }
        }
        return false; // 事件未被处理
    }

  private:
    VIS4Earth::GraphRenderer *_graphLayout;
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

    // 添加点击事件
    VIS4Earth::NodeClickHandler *nodeClickHandler =
        new VIS4Earth::NodeClickHandler(graphLayout, viewer);
    // 将 NodeClickHandler 添加到 Viewer 的事件处理器中
    viewer->addEventHandler(nodeClickHandler);
    // 添加鼠标滚轮事件处理器
    // MouseWheelEventHandler *mouseWheelHandler = new MouseWheelEventHandler(graphLayout);
    // viewer->addEventHandler(mouseWheelHandler);

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