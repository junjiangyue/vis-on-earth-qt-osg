#ifndef VIS4EARTH_GRAPH_VISER_EDGE_BUNDLING_H
#define SCIVIS_GRAPH_VISER_EDGE_BUNDLING_H

#include "spatial_grid_bundling.h"
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <vis4earth/graph_viser/graph.h>
#include <chrono>

namespace VIS4Earth {
class EdgeBundling {
    // 类似上面
  private:
    std::shared_ptr<VIS4Earth::Graph> origGrph; // 原图，全程不修改
    std::shared_ptr<VIS4Earth::Graph> layoutedGrph;
    float initialRadius = 1.0f; // 初始radius
    float minRadius = 1.0f;     // 最小radius
    int maxCycles = 5;          // 最大cycle数

  public:
    struct BundlingParam {
        // Algorithm parameters
        double K;                      // Global spring constant (K).
        int I;                         // Number of iterations in cycle.
        int iter;                      // Number of remaining iterations.
        int cycles;                    // Cycles left;
        double compatibilityThreshold; // Compatibility threshold.
        double smoothWidth;            // Width of the Gaussian smoothing.

        // Physical parameters
        double S;                    // Displacement of division points in a single iteration.
        double edgeDistance;         // Minimum distance between edges.
        bool gravitationIsOn;        // Marks whether gravitation is on.
        glm::vec3 gravitationCenter; // Gravitation center.
        double gravitationExponent;  // Gravitation exponent.

        // Network parameters
        double edgeWeightThreshold;     // Threshold on edge weights (for dense graphs).
        double edgePercentageThreshold; // Percentage of edges being kept (for dense graphs).
    };
    // 边集束的代码转移到edge_Bundling
    std::shared_ptr<VIS4Earth::Graph> GetLayoutedGraph() { return layoutedGrph; }
    void SetGraph(std::shared_ptr<VIS4Earth::Graph> grph) {
        origGrph = grph;
        layoutedGrph = grph; // 深拷贝
    }
    void SetParameter(const BundlingParam &param) {
        layoutedGrph->setAlgorithmParams(param.K, param.cycles, param.I,
                                         param.compatibilityThreshold, param.smoothWidth);
        layoutedGrph->setPhysicsParams(param.S, param.edgeDistance, param.gravitationCenter,
                                       param.gravitationExponent);
        layoutedGrph->setNetworkParams(param.edgeWeightThreshold, param.edgePercentageThreshold);
        layoutedGrph->setCycles(5);
    }
    void EdgeBundle() {
        using namespace std::chrono;
        auto t_start = high_resolution_clock::now();
        int cycleCount = 0;
        // 计算初始radius
        auto edges = layoutedGrph->getEdges();
        float minX = edges[0].subdivs[0].x, maxX = edges[0].subdivs[0].x;
        float minY = edges[0].subdivs[0].y, maxY = edges[0].subdivs[0].y;
        int edgesNum = (int)edges.size();
        for (int i = 0; i < edgesNum; ++i) {
            for (size_t k = 0; k < edges[i].subdivs.size(); ++k) {
                minX = std::min(minX, edges[i].subdivs[k].x);
                maxX = std::max(maxX, edges[i].subdivs[k].x);
                minY = std::min(minY, edges[i].subdivs[k].y);
                maxY = std::max(maxY, edges[i].subdivs[k].y);
            }
        }
        float buffer = 0.01f * (maxX - minX);
        minX -= buffer; maxX += buffer; minY -= buffer; maxY += buffer;
        initialRadius = ((maxX - minX) + (maxY - minY)) / (2.0f * std::sqrt((float)edgesNum));
        if (initialRadius < 1e-6f) initialRadius = 1.0f;
        minRadius = initialRadius * 0.1f; // 最小为初始的10%
        maxCycles = layoutedGrph->getCycles();
        do {
            // 动态递减radius
            float radius = initialRadius * (1.0f - (float)cycleCount / (float)maxCycles);
            if (radius < minRadius) radius = minRadius;
            auto iter_start = high_resolution_clock::now();
            while (Iterate(layoutedGrph, radius) > 0)
                ;
            auto iter_end = high_resolution_clock::now();
            double iter_ms = duration_cast<milliseconds>(iter_end - iter_start).count();
            printf("[EdgeBundling] Cycle %d completed, time cost: %.2f ms, radius=%.4f\n", ++cycleCount, iter_ms, radius);
            auto edges = layoutedGrph->getEdges();
            AddSubvisions(edges);
            layoutedGrph->setEdges(edges);
        } while (UpdateCycle(layoutedGrph) > 0);
        auto edges2 = layoutedGrph->getEdges();
        int edgesNum2 = (int)edges2.size();
        for (int i = 0; i < edgesNum2; i++)
            edges2[i].smooth(layoutedGrph->getSmoothWidth());
        layoutedGrph->setEdges(edges2);
        auto t_end = high_resolution_clock::now();
        double total_ms = duration_cast<milliseconds>(t_end - t_start).count();
        printf("[EdgeBundling] All cycles completed, total time cost: %.2f ms\n", total_ms);
    }
    int Iterate(std::shared_ptr<VIS4Earth::Graph> grph, float radius) {
        std::vector<VIS4Earth::Edge> edges = grph->getEdges();
        int edgesNum = (int)edges.size();
        std::vector<std::vector<glm::vec3>> forces(
            edgesNum,
            std::vector<glm::vec3>((int)edges[0].subdivs.size(), glm::vec3(0.0, 0.0, 0.0)));

        // spring forces
        for (int i = 0; i < edgesNum; i++)
            edges[i].addSpringForces(forces[i], grph->getK());

        // ---空间分区加速静电力---
        float minX = edges[0].subdivs[0].x, maxX = edges[0].subdivs[0].x;
        float minY = edges[0].subdivs[0].y, maxY = edges[0].subdivs[0].y;
        for (int i = 0; i < edgesNum; ++i) {
            for (size_t k = 0; k < edges[i].subdivs.size(); ++k) {
                minX = std::min(minX, edges[i].subdivs[k].x);
                maxX = std::max(maxX, edges[i].subdivs[k].x);
                minY = std::min(minY, edges[i].subdivs[k].y);
                maxY = std::max(maxY, edges[i].subdivs[k].y);
            }
        }
        float buffer = 0.01f * (maxX - minX);
        minX -= buffer; maxX += buffer; minY -= buffer; maxY += buffer;
        int maxPointsPerNode = 16; // 可调
        int maxDepth = 12;         // 可调
        VIS4Earth::QuadtreeBundling quadtree(minX, minY, maxX, maxY, maxPointsPerNode, maxDepth);
        // 2. 插入所有细分点
        for (int edgeIdx = 0; edgeIdx < edgesNum; ++edgeIdx) {
            for (int subdivIdx = 0; subdivIdx < (int)edges[edgeIdx].subdivs.size(); ++subdivIdx) {
                quadtree.insert(edgeIdx, subdivIdx, edges[edgeIdx].subdivs[subdivIdx]);
            }
        }
        // 调试信息统计
        size_t totalQueries = 0;
        size_t totalNeighbors = 0;
        size_t maxNeighbors = 0;
        size_t minNeighbors = (size_t)-1;
        // 3. 静电力计算
        for (int edgeIdx = 0; edgeIdx < edgesNum; ++edgeIdx) {
            for (int subdivIdx = 0; subdivIdx < (int)edges[edgeIdx].subdivs.size(); ++subdivIdx) {
                glm::vec3 pos = edges[edgeIdx].subdivs[subdivIdx];
                auto neighbors = quadtree.query(pos, radius);
                size_t nsize = neighbors.size();
                totalQueries++;
                totalNeighbors += nsize;
                if (nsize > maxNeighbors) maxNeighbors = nsize;
                if (nsize < minNeighbors) minNeighbors = nsize;
                for (const auto &n : neighbors) {
                    int nEdgeIdx = n.first;
                    if (nEdgeIdx == edgeIdx)
                        continue; // 跳过自身
                    // 可选：只对兼容边
                    if (std::find(edges[edgeIdx].compatibleEdges.begin(),
                                  edges[edgeIdx].compatibleEdges.end(),
                                  nEdgeIdx) == edges[edgeIdx].compatibleEdges.end())
                        continue;
                    // 只对同一细分点序号
                    int nSubdivIdx = n.second;
                    if (nSubdivIdx != subdivIdx)
                        continue;
                    // 计算静电力
                    glm::vec3 dist = edges[nEdgeIdx].subdivs[nSubdivIdx] - pos;
                    double dlen = glm::length(dist);
                    if (dlen > grph->getEdgeDistance()) {
                        forces[edgeIdx][subdivIdx] += dist / glm::vec3(dlen);
                    }
                }
            }
        }
        // 输出调试信息
        printf("[Quadtree] totalQueries=%zu, avgNeighbors=%.2f, maxNeighbors=%zu, minNeighbors=%zu, radius=%.4f\n",
            totalQueries, totalQueries ? (double)totalNeighbors / totalQueries : 0.0, maxNeighbors, minNeighbors == (size_t)-1 ? 0 : minNeighbors, radius);

        // gravitation
        if (grph->getGravitationIsOn()) {
            for (int i = 0; i < edgesNum; i++)
                edges[i].addGravitationalForces(forces[i], grph->getGravitationCenter(),
                                                grph->getGravitationExponent());
        }

        // update edges
        for (int i = 0; i < edgesNum; i++)
            edges[i].update(forces[i], grph->getS());
        int iter = grph->getIter();
        iter--;
        grph->setIter(iter);
        grph->setEdges(edges);
        return iter;
    }

    int UpdateCycle(std::shared_ptr<VIS4Earth::Graph> grph) {
        double S = grph->getS();
        int I = grph->getI();
        int iter = grph->getIter();
        int cycles = grph->getCycles();
        S *= 0.5;
        I = 2 * I / 3;
        iter = I;
        cycles--;
        grph->setS(S);
        grph->setI(I);
        grph->setIter(iter);
        grph->setCycles(cycles);
        return cycles;
    }

    void AddSubvisions(std::vector<VIS4Earth::Edge> &edges) {
        int edgesNum = (int)edges.size();
        for (int i = 0; i < edgesNum; i++)
            edges[i].addSubdivisions();
    }

    void Smooth(std::shared_ptr<VIS4Earth::Graph> grph, double smoothWidth) {
        auto edges = grph->getEdges();
        int edgesNum = (int)edges.size();
        for (int i = 0; i < edgesNum; i++)
            edges[i].smooth(smoothWidth);
        grph->setEdges(edges);
    }
};
} // namespace VIS4Earth

#endif // VIS4EARTH_GRAPH_VISER_EDGE_BUNDLING_H