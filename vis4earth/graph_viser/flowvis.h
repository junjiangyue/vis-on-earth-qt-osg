#pragma once
#include "graph.h"
#ifndef FLOWVIS_H
#define FLOWVIS_H

namespace VIS4Earth {

class FlowVisualization {
  private:
    VIS4Earth::Graph graph;

  public:
    static void drawArrowFlow();
    void setGraph(VIS4Earth::Graph grph);
    void getGraph();
};

} // namespace VIS4Earth

#endif // FLOWVIS_H