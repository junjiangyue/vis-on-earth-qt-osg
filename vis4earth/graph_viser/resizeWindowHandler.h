#ifndef RESIZEWINDOWHANDLER_H
#define RESIZEWINDOWHANDLER_H
#include <iostream>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>

// 自定义事件处理器
class ResizeHandler : public osgGA::GUIEventHandler {
  public:
    ResizeHandler(EarthMarkerManager *markerManager, osgViewer::Viewer *viewer)
        : _markerManager(markerManager), viewer(viewer) {}
    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override {
        // 检测窗口大小改变事件
        if (ea.getEventType() == osgGA::GUIEventAdapter::RESIZE) {
            int width = ea.getWindowWidth();   // 获取窗口宽度
            int height = ea.getWindowHeight(); // 获取窗口高度

            //std::cout << "Window resized: " << width << " x " << height << std::endl;
            _markerManager->setWindow(width, height);
            
        }

        // 返回 false 表示事件未被完全处理，其他处理器可以继续处理
        return false;
    }

  private:
    EarthMarkerManager *_markerManager; // 管理 Geode 的类
    osgViewer::Viewer *viewer;
};
#endif