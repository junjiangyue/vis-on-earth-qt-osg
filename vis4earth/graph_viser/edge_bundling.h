#ifndef VIS4EARTH_GRAPH_VISER_EDGE_BUNDLING_H
#define SCIVIS_GRAPH_VISER_EDGE_BUNDLING_H

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <vis4earth/graph_viser/graph.h>

namespace VIS4Earth {
class EdgeBundling {
    // 类似上面
  private:
    VIS4Earth::Graph origGrph; // 原图，全程不修改
    VIS4Earth::Graph layoutedGrph;

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
    VIS4Earth::Graph GetLayoutedGraph() { return layoutedGrph; }
    void SetGraph(VIS4Earth::Graph grph) {
        origGrph = grph;
        layoutedGrph = grph;
    }
    void SetParameter(const BundlingParam &param) {
        layoutedGrph.setAlgorithmParams(param.K, param.cycles, param.I,
                                        param.compatibilityThreshold, param.smoothWidth);
        layoutedGrph.setPhysicsParams(param.S, param.edgeDistance, param.gravitationCenter,
                                      param.gravitationExponent);
        layoutedGrph.setNetworkParams(param.edgeWeightThreshold, param.edgePercentageThreshold);
        layoutedGrph.setCycles(5);
    }
    void EdgeBundle() {

        // TODO:
        do {
            while (Iterate(layoutedGrph) > 0)
                ;
            auto edges = layoutedGrph.getEdges();
            AddSubvisions(edges);
            layoutedGrph.setEdges(edges);
        } while (UpdateCycle(layoutedGrph) > 0);
        auto edges = layoutedGrph.getEdges();
        int edgesNum = (int)edges.size();
        for (int i = 0; i < edgesNum; i++)
            edges[i].smooth(layoutedGrph.getSmoothWidth());
        layoutedGrph.setEdges(edges);
        // Smooth(layoutedGrph, layoutedGrph.getSmoothWidth());
    }
    int Iterate(VIS4Earth::Graph &grph) {
        std::vector<VIS4Earth::Edge> edges = grph.getEdges();
        int edgesNum = (int)edges.size();
        std::vector<std::vector<glm::vec3>> forces(
            edgesNum,
            std::vector<glm::vec3>((int)edges[0].subdivs.size(), glm::vec3(0.0, 0.0, 0.0)));

        // spring forces
        for (int i = 0; i < edgesNum; i++)
            edges[i].addSpringForces(forces[i], grph.getK());

        // electrostatic forces
        for (int i = 0; i < edgesNum; i++) {
            int compatibleEdgesNum = (int)edges[i].compatibleEdges.size();
            for (int j = 0; j < compatibleEdgesNum; j++)
                edges[i].addElectrostaticForces(forces[i], edges[edges[i].compatibleEdges[j]],
                                                grph.getEdgeDistance());
        }

        // gravitation
        if (grph.getGravitationIsOn()) {
            for (int i = 0; i < edgesNum; i++)
                edges[i].addGravitationalForces(forces[i], grph.getGravitationCenter(),
                                                grph.getGravitationExponent());
        }

        // update edges
        for (int i = 0; i < edgesNum; i++)
            edges[i].update(forces[i], grph.getS());
        int iter = grph.getIter();
        iter--;
        grph.setIter(iter);
        grph.setEdges(edges);
        return iter;
    }

    int UpdateCycle(VIS4Earth::Graph &grph) {
        double S = grph.getS();
        int I = grph.getI();
        int iter = grph.getIter();
        int cycles = grph.getCycles();
        S *= 0.5;
        I = 2 * I / 3;
        iter = I;
        cycles--;
        grph.setS(S);
        grph.setI(I);
        grph.setIter(iter);
        grph.setCycles(cycles);
        return cycles;
    }

    void AddSubvisions(std::vector<VIS4Earth::Edge> &edges) {
        int edgesNum = (int)edges.size();
        for (int i = 0; i < edgesNum; i++)
            edges[i].addSubdivisions();
    }

    void Smooth(VIS4Earth::Graph &grph, double smoothWidth) {
        auto edges = grph.getEdges();
        int edgesNum = (int)edges.size();
        for (int i = 0; i < edgesNum; i++)
            edges[i].smooth(smoothWidth);
        grph.setEdges(edges);
    }
};
} // namespace VIS4Earth

#endif // VIS4EARTH_GRAPH_VISER_EDGE_BUNDLING_H