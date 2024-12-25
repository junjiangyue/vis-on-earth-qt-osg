#ifndef NODEHOVERHANDLER_H
#define NODEHOVERHANDLER_H

#include "markManager.h"
#include <osg/Geode>    // 用于操作 Geode
#include <osg/Geometry> // 用于操作几何体
#include <osg/ShapeDrawable>
#include <osg/Vec3>                       // 用于三维向量
#include <osg/Vec4>                       // 用于四维向量（颜色）
#include <osgGA/GUIEventHandler>          // GUIEventHandler基类
#include <osgUtil/LineSegmentIntersector> // 线段交点检测
#include <osgViewer/Viewer>

class HoverEventHandler : public osgGA::GUIEventHandler {
  public:
    HoverEventHandler(EarthMarkerManager *markerManager, osgViewer::Viewer *viewer)
        : _markerManager(markerManager), viewer(viewer) {}
    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override {
        if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE ||
            ea.getEventType() == osgGA::GUIEventAdapter::FRAME) {
            float x = ea.getX();
            float y = ea.getY();

            osgUtil::LineSegmentIntersector::Intersections intersections;
            if (viewer->computeIntersections(x, y, intersections)) {
                for (const auto &intersection : intersections) {
                    // 遍历 intersection.nodePath 以找到 MarkerGeode
                    osg::Geode *markerGeode = findMarkerGeode(intersection.nodePath);
                    if (!markerGeode)
                        continue;

                    // 获取 Geometry
                    osg::Geometry *geometry = getGeometryFromGeode(markerGeode);
                    if (!geometry)
                        continue;

                    // 获取 vertexIDs
                    osg::FloatArray *vertexIDs =
                        dynamic_cast<osg::FloatArray *>(geometry->getVertexAttribArray(1));
                    if (!vertexIDs) {
                        std::cout << "Failed to retrieve vertexIDs!" << std::endl;
                        continue;
                    }

                    // 获取命中顶点的 ID
                    float hitVertexID = getHitVertexID(intersection, geometry, vertexIDs);
                    if (hitVertexID != -1) {
                        highlightNode(hitVertexID);
                        return true; // 成功处理事件
                    }
                }

                // 如果没有命中标记点，重置高亮
                resetHighlight();
            }

            return true; // 处理了事件
        }
        return false; // 没有处理事件
    }

  private:
    EarthMarkerManager *_markerManager; // 管理 Geode 的类
    osgViewer::Viewer *viewer;
    osg::Geode *findMarkerGeode(const osg::NodePath &nodePath) {
        for (const auto &node : nodePath) {
            if (!node)
                continue;

            osg::Group *group = dynamic_cast<osg::Group *>(node);
            if (group) {
                for (unsigned int i = 0; i < group->getNumChildren(); ++i) {
                    osg::Node *child = group->getChild(i);
                    if (child && child->getName() == "MarkerGeode") {
                        return dynamic_cast<osg::Geode *>(child);
                    }
                }
            }
        }
        return nullptr;
    }
    osg::Geometry *getGeometryFromGeode(osg::Geode *geode) {
        if (!geode)
            return nullptr;

        for (unsigned int i = 0; i < geode->getNumDrawables(); ++i) {
            osg::Drawable *drawable = geode->getDrawable(i);
            if (drawable) {
                osg::Geometry *geometry = dynamic_cast<osg::Geometry *>(drawable);
                if (geometry)
                    return geometry;
            }
        }
        return nullptr;
    }
    float getHitVertexID(const osgUtil::LineSegmentIntersector::Intersection &intersection,
                         osg::Geometry *geometry, osg::FloatArray *vertexIDs) {
        if (!geometry || !vertexIDs)
            return -1;
        float distanceThreshold = 65000.0f;
        // 检查 indexList
        if (!intersection.indexList.empty()) {
            unsigned int hitVertexIndex = intersection.indexList[0];
            if (hitVertexIndex < vertexIDs->size()) {
                return (*vertexIDs)[hitVertexIndex];
            }
        }

        // 如果 indexList 为空，计算最近点
        osg::Vec3 hitPoint = intersection.localIntersectionPoint;
        osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(geometry->getVertexArray());
        if (vertices) {
            unsigned int closestVertexIndex = -1;
            float minDistance = std::numeric_limits<float>::max();

            for (unsigned int i = 0; i < vertices->size(); ++i) {
                float distance = (vertices->at(i) - hitPoint).length2();
                if (distance < minDistance) {
                    minDistance = distance;
                    closestVertexIndex = i;
                }
            }

            if (closestVertexIndex != -1 && closestVertexIndex < vertexIDs->size()) {
                if (std::sqrt(minDistance) <= distanceThreshold) { // 检查距离阈值
                    return (*vertexIDs)[closestVertexIndex];
                }
            }
        }

        return -1; // 未找到有效的顶点
    }
    void resetHighlight() {
        for (auto &marker : _markerManager->_visibleMarkers) {
            marker.isHover = false; // 隐藏标签
        }
        _markerManager->updateMarkers();
    };
    void highlightNode(int nodeId) {
        for (auto &marker : _markerManager->_visibleMarkers) {
            if (marker.id == nodeId) {
                marker.isHover = true; // 显示标签
                marker.visible = true;
            }
        }
        _markerManager->updateHover();
    };
};

#endif // NODECLICKHANDLER_H