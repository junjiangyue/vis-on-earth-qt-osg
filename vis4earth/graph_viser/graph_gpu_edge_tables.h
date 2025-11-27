#ifndef VIS4EARTH_GRAPH_VISER_GRAPH_GPU_EDGE_TABLES_H
#define VIS4EARTH_GRAPH_VISER_GRAPH_GPU_EDGE_TABLES_H

#include <osg/Vec3>

#include <map>
#include <string>
#include <vector>

namespace VIS4Earth {
namespace GpuEdge {

// 控制点表条目
struct ControlPoint {
    osg::Vec3 pos; // 世界空间或经纬度空间位置，具体含义由调用方决定
};

// 段表条目：描述一条边在控制点表中的一个连续段
struct SegmentInfo {
    unsigned int controlStart; // 在 ControlPointTable 中的起始索引
    unsigned short count;      // 本段包含的控制点数量
    unsigned short type;       // 曲线类型（0=直线/默认，1=弧线，后续可扩展）
};

// 边 Meta 信息：描述一条逻辑边在各个表中的映射关系
struct EdgeMetaInfo {
    unsigned int controlStart;   // 对应的控制点起始索引
    unsigned short segmentStart; // 对应的 SegmentInfo 起始索引
    unsigned short segmentCount; // 段数量
    float weight;                // 边权重
    unsigned int lineID;         // 动画 / 纹理用的行索引
};

/**
 * @brief 从通用的节点 / 边集合构建 GPU 曲线渲染用的控制点表、段表和 EdgeMeta 表。
 *
 * 模板参数要求：
 *  - NodeT 需至少包含成员：osg::Vec3 pos;
 *  - EdgeT 需至少包含成员：
 *      - std::string from;
 *      - std::string to;
 *      - std::vector<osg::Vec3> subDivs;
 *      - float weight;
 *
 * 本函数不依赖具体的 GraphRenderer 类型，只约定字段接口，因此可以在多个渲染器中复用。
 */
template <typename NodeT, typename EdgeT>
void buildTablesFromEdges(const std::map<std::string, NodeT> &nodes,
                          const std::vector<EdgeT> &edges,
                          std::vector<ControlPoint> &outControlPoints,
                          std::vector<SegmentInfo> &outSegments,
                          std::vector<EdgeMetaInfo> &outMetas, bool useBundling) {
    outControlPoints.clear();
    outSegments.clear();
    outMetas.clear();

    if (nodes.empty() || edges.empty()) {
        return;
    }

    unsigned int currentControlStart = 0;
    unsigned short currentSegmentStart = 0;
    unsigned int lineID = 0;

    typename std::vector<EdgeT>::const_iterator eit = edges.begin();
    for (; eit != edges.end(); ++eit) {
        const EdgeT &edge = *eit;

        typename std::map<std::string, NodeT>::const_iterator fromIt = nodes.find(edge.from);
        typename std::map<std::string, NodeT>::const_iterator toIt = nodes.find(edge.to);
        if (fromIt == nodes.end() || toIt == nodes.end()) {
            continue;
        }

        std::vector<ControlPoint> localCP;
        localCP.reserve(edge.subDivs.size() + 2);

        // 起点
        ControlPoint cpStart;
        cpStart.pos = fromIt->second.pos;
        localCP.push_back(cpStart);

        // Bundling 细分点（如果启用且存在）
        if (useBundling && !edge.subDivs.empty()) {
            typename std::vector<osg::Vec3>::const_iterator sit = edge.subDivs.begin();
            for (; sit != edge.subDivs.end(); ++sit) {
                ControlPoint cpMid;
                cpMid.pos = *sit;
                localCP.push_back(cpMid);
            }
        }

        // 终点
        ControlPoint cpEnd;
        cpEnd.pos = toIt->second.pos;
        localCP.push_back(cpEnd);

        if (localCP.size() < 2) {
            continue;
        }

        // 写入全局控制点表
        const unsigned int thisControlStart = currentControlStart;
        typename std::vector<ControlPoint>::const_iterator cpIt = localCP.begin();
        for (; cpIt != localCP.end(); ++cpIt) {
            outControlPoints.push_back(*cpIt);
            ++currentControlStart;
        }

        // 为整条边记录一个 SegmentInfo
        SegmentInfo seg;
        seg.controlStart = thisControlStart;
        seg.count = static_cast<unsigned short>(localCP.size());
        seg.type = 0;
        outSegments.push_back(seg);

        // EdgeMeta
        EdgeMetaInfo meta;
        meta.controlStart = thisControlStart;
        meta.segmentStart = currentSegmentStart;
        meta.segmentCount = 1;
        meta.weight = edge.weight;
        meta.lineID = lineID;
        outMetas.push_back(meta);

        ++currentSegmentStart;
        ++lineID;
    }
}

} // namespace GpuEdge
} // namespace VIS4Earth

#endif // VIS4EARTH_GRAPH_VISER_GRAPH_GPU_EDGE_TABLES_H


