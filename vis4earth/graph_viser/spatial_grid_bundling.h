#ifndef VIS4EARTH_GRAPH_VISER_SPATIAL_GRID_BUNDLING_H
#define VIS4EARTH_GRAPH_VISER_SPATIAL_GRID_BUNDLING_H

#include <cmath>
#include <utility>
#include <vector>

#include <glm/vec3.hpp>

namespace VIS4Earth {

class GridCellBundling {
  public:
    std::vector<std::pair<int, int>> points;
    void addPoint(int edgeIdx, int subdivIdx) {
        points.push_back(std::make_pair(edgeIdx, subdivIdx));
    }
    void clear() { points.clear(); }
};

class SpatialGridBundling {
  public:
    SpatialGridBundling(double minX_, double minY_, double maxX_, double maxY_, double cellSize_)
        : minX(minX_), minY(minY_), maxX(maxX_), maxY(maxY_), cellSize(cellSize_) {
        rows = static_cast<int>(std::ceil((maxY - minY) / cellSize));
        cols = static_cast<int>(std::ceil((maxX - minX) / cellSize));
        grid.resize(rows, std::vector<GridCellBundling>(cols));
    }

    void insertPoint(int edgeIdx, int subdivIdx, const glm::vec3 &pos) {
        int row, col;
        getCellIndex(pos, row, col);
        if (row >= 0 && row < rows && col >= 0 && col < cols) {
            grid[row][col].addPoint(edgeIdx, subdivIdx);
        }
    }

    std::vector<std::pair<int, int>> queryNeighbors(const glm::vec3 &pos, double radius) {
        std::vector<std::pair<int, int>> result;
        int centerRow, centerCol;
        getCellIndex(pos, centerRow, centerCol);
        int r = static_cast<int>(std::ceil(radius / cellSize));
        for (int dr = -r; dr <= r; ++dr) {
            for (int dc = -r; dc <= r; ++dc) {
                int row = centerRow + dr;
                int col = centerCol + dc;
                if (row >= 0 && row < rows && col >= 0 && col < cols) {
                    for (const auto &p : grid[row][col].points) {
                        result.push_back(p);
                    }
                }
            }
        }
        return result;
    }

    void clear() {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                grid[i][j].clear();
            }
        }
    }

  private:
    std::vector<std::vector<GridCellBundling>> grid;
    double minX, minY, maxX, maxY, cellSize;
    int rows, cols;

    void getCellIndex(const glm::vec3 &pos, int &row, int &col) const {
        col = static_cast<int>((pos.x - minX) / cellSize);
        row = static_cast<int>((pos.y - minY) / cellSize);
    }
};

// 四叉树节点
class QuadtreeBundling {
public:
    struct Point {
        int edgeIdx;
        int subdivIdx;
        glm::vec3 pos;
        Point(int e, int s, const glm::vec3& p) : edgeIdx(e), subdivIdx(s), pos(p) {}
    };
    QuadtreeBundling(float minX_, float minY_, float maxX_, float maxY_, int maxPointsPerNode_ = 16, int maxDepth_ = 12)
        : minX(minX_), minY(minY_), maxX(maxX_), maxY(maxY_), maxPointsPerNode(maxPointsPerNode_), maxDepth(maxDepth_) {
        root = new Node(minX, minY, maxX, maxY, 0);
    }
    ~QuadtreeBundling() { clear(); delete root; }
    void clear() { root->clear(); }
    void insert(int edgeIdx, int subdivIdx, const glm::vec3& pos) {
        root->insert(Point(edgeIdx, subdivIdx, pos), maxPointsPerNode, maxDepth);
    }
    std::vector<std::pair<int, int>> query(const glm::vec3& pos, float radius) const {
        std::vector<std::pair<int, int>> result;
        root->query(pos, radius, result);
        return result;
    }
private:
    struct Node {
        float minX, minY, maxX, maxY;
        int depth;
        std::vector<Point> points;
        Node* children[4];
        Node(float minX_, float minY_, float maxX_, float maxY_, int d)
            : minX(minX_), minY(minY_), maxX(maxX_), maxY(maxY_), depth(d) {
            for (int i = 0; i < 4; ++i) children[i] = nullptr;
        }
        ~Node() { for (int i = 0; i < 4; ++i) if (children[i]) delete children[i]; }
        void clear() {
            points.clear();
            for (int i = 0; i < 4; ++i) if (children[i]) { children[i]->clear(); delete children[i]; children[i] = nullptr; }
        }
        bool isLeaf() const { return children[0] == nullptr; }
        void insert(const Point& p, int maxPoints, int maxDepth) {
            if (isLeaf()) {
                points.push_back(p);
                if ((int)points.size() > maxPoints && depth < maxDepth) {
                    subdivide();
                    for (const auto& pt : points) insertToChildren(pt, maxPoints, maxDepth);
                    points.clear();
                }
            } else {
                insertToChildren(p, maxPoints, maxDepth);
            }
        }
        void insertToChildren(const Point& p, int maxPoints, int maxDepth) {
            int idx = getChildIndex(p.pos);
            if (!children[idx]) createChild(idx);
            children[idx]->insert(p, maxPoints, maxDepth);
        }
        void query(const glm::vec3& pos, float radius, std::vector<std::pair<int, int>>& result) const {
            if (!intersects(pos, radius)) return;
            if (isLeaf()) {
                for (const auto& pt : points) {
                    float dx = pt.pos.x - pos.x;
                    float dy = pt.pos.y - pos.y;
                    if (dx * dx + dy * dy <= radius * radius) {
                        result.push_back(std::make_pair(pt.edgeIdx, pt.subdivIdx));
                    }
                }
            } else {
                for (int i = 0; i < 4; ++i) if (children[i]) children[i]->query(pos, radius, result);
            }
        }
        bool intersects(const glm::vec3& pos, float radius) const {
            float cx = std::max(minX, std::min(pos.x, maxX));
            float cy = std::max(minY, std::min(pos.y, maxY));
            float dx = pos.x - cx, dy = pos.y - cy;
            return (dx * dx + dy * dy) <= radius * radius;
        }
        int getChildIndex(const glm::vec3& pos) const {
            float midX = 0.5f * (minX + maxX);
            float midY = 0.5f * (minY + maxY);
            if (pos.x < midX) {
                if (pos.y < midY) return 0; // 左下
                else return 2; // 左上
            } else {
                if (pos.y < midY) return 1; // 右下
                else return 3; // 右上
            }
        }
        void createChild(int idx) {
            float midX = 0.5f * (minX + maxX);
            float midY = 0.5f * (minY + maxY);
            switch (idx) {
                case 0: children[0] = new Node(minX, minY, midX, midY, depth + 1); break;
                case 1: children[1] = new Node(midX, minY, maxX, midY, depth + 1); break;
                case 2: children[2] = new Node(minX, midY, midX, maxY, depth + 1); break;
                case 3: children[3] = new Node(midX, midY, maxX, maxY, depth + 1); break;
            }
        }
        void subdivide() {
            for (int i = 0; i < 4; ++i) createChild(i);
        }
    };
    Node* root;
    float minX, minY, maxX, maxY;
    int maxPointsPerNode, maxDepth;
};

} // namespace VIS4Earth

#endif // VIS4EARTH_GRAPH_VISER_SPATIAL_GRID_BUNDLING_H