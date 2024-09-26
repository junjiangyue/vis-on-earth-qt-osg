#ifndef VIS4EARTH_COMMON_GRAPH_H
#define VIS4EARTH_COMMON_GRAPH_H

#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <stack>
#include <queue>


#define EPSILON 1e-6
#define INV_SQRT_2PI 0.3989422804

namespace VIS4Earth {

struct Node {
    std::string id;
    int degree;
    int radius;
    double mass;
    double repulsion;
    double stiffness;
    double damping;
    glm::vec3 pos;
    glm::vec3 vel;
    glm::vec3 acc;
    glm::vec3 force;
    double level;
    std::string name;

    Node()
        : degree(0), radius(1), mass(1.0), repulsion(1.0), stiffness(1.0), damping(1.0), pos(0.0f),
          vel(1.0f), acc(1.0f), force(0.0f) {}

    Node(double x, double y)
        : degree(0), radius(1), mass(1.0), repulsion(1.0), stiffness(1.0), damping(1.0),
          pos(x, y, 0.0), vel(1.0f), acc(1.0f), force(0.0f) {}
    Node(double x, double y, double level)
        : degree(0), radius(1), mass(1.0), repulsion(1.0), stiffness(1.0), damping(1.0),
          pos(x, y, 0.0), vel(1.0f), acc(1.0f), force(0.0f), level(level) {}
    Node(double x, double y, double level, std::string name)
        : degree(0), radius(1), mass(1.0), repulsion(1.0), stiffness(1.0), damping(1.0),
          pos(x, y, 0.0), vel(1.0f), acc(1.0f), force(0.0f), level(level), name(name) {}
};

struct Edge {
    std::string sourceLabel;
    std::string targetLabel;
    std::string id;
    glm::vec3 start;
    glm::vec3 end;
    std::vector<glm::vec3> subdivs;
    double width;
    std::vector<int> compatibleEdges;
    int weight;

    Edge(const std::string &mySourceLabel, const std::string &myTargetLabel,
         const glm::vec3 &myStart, const glm::vec3 &myEnd, double myWidth)
        : sourceLabel(mySourceLabel), targetLabel(myTargetLabel), start(myStart), end(myEnd),
          width(myWidth) {
        arrangeDirection();
        addSubdivisions();
    }

    void addSubdivisions() {
        int oldSubdivsNum = static_cast<int>(subdivs.size());
        if (oldSubdivsNum == 0) {
            subdivs.assign(1, center(start, end));
        } else {
            int newSubdivsNum = 2 * oldSubdivsNum, subdivIndex = 0, v1Index = -1, v2Index = 0;
            double segmentLength = double(oldSubdivsNum + 1) / double(newSubdivsNum + 1);
            std::vector<glm::vec3> subdivisions(newSubdivsNum, glm::vec3(1.0f));
            glm::vec3 v1 = start, v2 = subdivs[0];
            double r = segmentLength;
            while (subdivIndex < newSubdivsNum) {
                subdivisions[subdivIndex] = v1 + (v2 - v1) * glm::vec3(r, r, r);
                subdivIndex++;
                if (r + segmentLength > 1.0) {
                    r = segmentLength - (1.0 - r);
                    v1Index++;
                    v2Index++;

                    if (v1Index >= static_cast<int>(subdivs.size()) ||
                        v2Index > static_cast<int>(subdivs.size())) {
                        break;
                    }

                    if (v1Index >= 0) {
                        v1 = subdivs[v1Index];
                    }
                    if (v2Index < oldSubdivsNum) {
                        v2 = subdivs[v2Index];
                    } else {
                        v2 = end;
                    }
                } else {
                    r += segmentLength;
                }
            }
            subdivs = subdivisions;
        }
    }

    void arrangeDirection() {
        glm::vec3 v = end - start;
        if ((std::fabs(v.x) > std::fabs(v.y) && end.x < start.x) ||
            (std::fabs(v.x) < std::fabs(v.y) && end.y < start.y)) {
            std::swap(start, end);
            std::swap(sourceLabel, targetLabel);
        }
    }

    void update(std::vector<glm::vec3> &forces, double displacement) {
        int len = static_cast<int>(subdivs.size());
        for (int i = 0; i < len; ++i) {
            double flen = glm::length(forces[i]);
            if (flen > EPSILON) {
                subdivs[i] += forces[i] * glm::vec3(displacement / flen);
            }
        }
    }

    double gaussWeight(int dist, double sigma) const {
        return INV_SQRT_2PI * std::exp(-0.5 * std::pow(double(dist) / sigma, 2.0)) / sigma;
    }

    void smooth(double sigma) {
        int len = static_cast<int>(subdivs.size());
        std::vector<glm::vec3> subdivisions(len, glm::vec3(0.0f));
        double weight, totalWeight;
        for (int i = 0; i < len; ++i) {
            totalWeight = 0.0;
            weight = gaussWeight(i + 1, sigma);
            subdivisions[i] = subdivisions[i] + start * glm::vec3(weight, weight, weight);
            totalWeight += weight;

            for (int j = 0; j < len; ++j) {
                weight = gaussWeight(i - j, sigma);
                subdivisions[i] = subdivisions[i] + subdivs[j] * glm::vec3(weight, weight, weight);
                totalWeight += weight;
            }

            weight = gaussWeight(len - i + 1, sigma);
            subdivisions[i] = subdivisions[i] + end * glm::vec3(weight, weight, weight);
            totalWeight += weight;

            subdivisions[i] = subdivisions[i] / glm::vec3(totalWeight, totalWeight, totalWeight);
        }

        subdivs = subdivisions;
    }

    void addSpringForces(std::vector<glm::vec3> &forces, double springConstant) {
        int len = static_cast<int>(subdivs.size());
        double kP = springConstant / (glm::length(end - start) * double(len + 1));

        if (len == 1) {
            forces[0] += (start + end - subdivs[0] * glm::vec3(2.0)) * glm::vec3(kP);
        } else {
            forces[0] += (start + subdivs[1] - subdivs[0] * glm::vec3(2.0)) * glm::vec3(kP);
            for (int i = 1; i < len - 1; ++i) {
                forces[i] +=
                    (subdivs[i - 1] + subdivs[i + 1] - subdivs[i] * glm::vec3(2.0)) * glm::vec3(kP);
            }
            forces[len - 1] +=
                (subdivs[len - 2] + end - subdivs[len - 1] * glm::vec3(2.0)) * glm::vec3(kP);
        }
    }

    void addElectrostaticForces(std::vector<glm::vec3> &forces, const Edge &edge, double epsilon) {
        int len = static_cast<int>(subdivs.size());
        for (int i = 0; i < len; ++i) {
            glm::vec3 dist = edge.subdivs[i] - subdivs[i];
            double dlen = glm::length(dist);
            if (dlen > epsilon) {
                forces[i] += dist / glm::vec3(dlen);
            }
        }
    }

    void addGravitationalForces(std::vector<glm::vec3> &forces, const glm::vec3 &center,
                                double exponent) {
        int len = static_cast<int>(subdivs.size());
        for (int i = 0; i < len; ++i) {
            glm::vec3 dist = center - subdivs[i];
            double dlen = glm::length(dist);
            double temp = 0.1 * std::pow(dlen + 1.0, exponent);
            forces[i] += dist * glm::vec3(temp);
        }
    }

    static glm::vec3 center(const glm::vec3 &p1, const glm::vec3 &p2) {
        return (p1 + p2) / glm::vec3(2.0);
    }

    glm::vec3 vector() const { return end - start; }

    static double angleCompatibility(const Edge &edge1, const Edge &edge2) {
        glm::vec3 v1 = glm::normalize(edge1.vector());
        glm::vec3 v2 = glm::normalize(edge2.vector());
        glm::vec3 res = v1 * v2;
        return std::fabs(res.x + res.y + res.z);
    }

    static double scaleCompatibility(const Edge &edge1, const Edge &edge2) {
        double l1 = glm::length(edge1.vector());
        double l2 = glm::length(edge2.vector());
        double lavg = (l1 + l2) / 2.0;
        if (lavg > EPSILON) {
            return 2.0 / (lavg / std::min(l1, l2) + std::max(l1, l2) / lavg);
        } else {
            return 0.0;
        }
    }

    static double positionCompatibility(const Edge &edge1, const Edge &edge2) {
        double lavg = (glm::length(edge1.vector()) + glm::length(edge2.vector())) / 2.0;
        if (lavg > EPSILON) {
            glm::vec3 mid1 = center(edge1.start, edge1.end);
            glm::vec3 mid2 = center(edge2.start, edge2.end);
            return lavg / (lavg + glm::length(mid1 - mid2));
        } else {
            return 0.0;
        }
    }

    static double edgeVisibility(const Edge &edge1, const Edge &edge2) {
        glm::vec3 I0 = project(edge1.start, edge2.start, edge2.end);
        glm::vec3 I1 = project(edge1.end, edge2.start, edge2.end);
        glm::vec3 midI = center(I0, I1);
        glm::vec3 midP = center(edge2.start, edge2.end);
        return std::max(0.0, 1.0 - 2.0 * glm::length(midP - midI) / glm::length(I0 - I1));
    }

    static double visibilityCompatibility(const Edge &edge1, const Edge &edge2) {
        return std::min(edgeVisibility(edge1, edge2), edgeVisibility(edge2, edge1));
    }

    static glm::vec3 project(const glm::vec3 &point, const glm::vec3 &lineStart,
                             const glm::vec3 &lineEnd) {
        double L = glm::length(lineStart - lineEnd);
        double r = ((lineStart.y - point.y) * (lineStart.y - lineEnd.y) -
                    (lineStart.x - point.x) * (lineEnd.x - lineStart.x)) /
                   (L * L);
        return lineStart + (lineEnd - lineStart) * glm::vec3(r);
    }
};

struct Area {
    double leftBound;
    double rightBound;
    double upperBound;
    double bottomBound;
};

struct BundlingParam {
    double K;
    int I;
    int iter;
    int cycles;
    double compatibilityThreshold;
    double smoothWidth;
    double S;
    double edgeDistance;
    bool gravitationIsOn;
    glm::vec3 gravitationCenter;
    double gravitationExponent;
    double edgeWeightThreshold;
    double edgePercentageThreshold;
};

struct Graph {
    struct pair_hash {
        template <class T1, class T2> std::size_t operator()(const std::pair<T1, T2> &pair) const {
            std::size_t h1 = std::hash<T1>()(pair.first);
            std::size_t h2 = std::hash<T2>()(pair.second);
            return h1 ^ h2;
        }
    };

  private:
    std::unordered_map<std::string, Node> nodes;
    std::vector<Edge> edges;
    std::unordered_map<std::string, std::vector<std::string>>
        adj_list; // 邻接表，键为 string 类型的节点 id
    std::unordered_set<std::pair<int, int>, pair_hash> nodePairs;
    std::unordered_set<int> nodesNotMove;

    double K;
    int I;
    int iter;
    int cycles;
    double compatibilityThreshold;
    double smoothWidth;
    double S;
    double edgeDistance;
    bool gravitationIsOn;
    glm::vec3 gravitationCenter;
    double gravitationExponent;
    double edgeWeightThreshold;
    double edgePercentageThreshold;
    double edgeOpacity;
    bool nodeRestrictionIsOn;
    double repulsion;
    double spring_k;
    double attraction;
    int n_iterations;
    double edgeLength;
    Area restrictedArea;

  public:
    Graph()
        : K(0.000001), I(10), iter(I), cycles(6), compatibilityThreshold(0.6), smoothWidth(30.0),
          S(0.000001), edgeDistance(1e-4), gravitationIsOn(false),
          gravitationCenter(-900.0, -400.0, 0), gravitationExponent(-2.0),
          edgeWeightThreshold(-1.0), edgePercentageThreshold(-1.0), edgeOpacity(0.1),
          nodeRestrictionIsOn(false), repulsion(0.2), spring_k(5.4), attraction(0.32),
          n_iterations(0), edgeLength(50) {}

    int getLayoutIteration() const { return n_iterations; }

    const std::unordered_map<std::string, Node> &getNodes() const { return nodes; }

    const std::vector<Edge> &getEdges() const { return edges; }

    bool getNodesRestriction() const { return nodeRestrictionIsOn; }

    const std::unordered_set<int> &getNodesNotMove() const { return nodesNotMove; }

    const std::unordered_set<std::pair<int, int>, pair_hash> &getNodePairs() const {
        return nodePairs;
    }

    glm::vec3 getGravitationCenter() const { return gravitationCenter; }

    double getAttraction() const { return attraction; }

    double getRepulsion() const { return repulsion; }

    double getSpring() const { return spring_k; }

    double getEdgeLength() const { return edgeLength; }

    Area getRestrictedArea() const { return restrictedArea; }

    double getK() const { return K; }

    int getI() const { return I; }

    int getIter() const { return iter; }

    int getCycles() const { return cycles; }

    double getCompatibilityThreshold() const { return compatibilityThreshold; }

    double getSmoothWidth() const { return smoothWidth; }

    double getS() const { return S; }

    double getEdgeDistance() const { return edgeDistance; }

    bool getGravitationIsOn() const { return gravitationIsOn; }

    double getGravitationExponent() const { return gravitationExponent; }

    double getEdgeWeightThreshold() const { return edgeWeightThreshold; }

    double getEdgePercentageThreshold() const { return edgePercentageThreshold; }

    void setIter(int itr) { iter = itr; }

    void setS(double newS) { S = newS; }

    void setI(int newI) { I = newI; }

    void setCycles(int newCycles) { cycles = newCycles; }

    void enableGravitation() { gravitationIsOn = true; }

    void enableNodeRestriction(const Area &myRestrictedArea) {
        nodeRestrictionIsOn = true;
        restrictedArea = myRestrictedArea;
    }

    void setNetworkParams(double edgeWeightThreshold_, double edgePercentageThreshold_) {
        if (edgeWeightThreshold_ > 0.0) {
            edgeWeightThreshold = edgeWeightThreshold_;
        } else if (edgePercentageThreshold_ > 0.0) {
            edgePercentageThreshold = edgePercentageThreshold_;
        }
    }

    void setAlgorithmParams(double K_, int cycles_, int I0_, double compat_, double sigma_) {
        K = K_;
        cycles = cycles_;
        I = I0_;
        iter = I;
        compatibilityThreshold = compat_;
        smoothWidth = sigma_;
    }

    void setPhysicsParams(double S0_, double edgeDistance_, const glm::vec3 &gravCenter_,
                          double gravExponent_) {
        S = S0_;
        edgeDistance = edgeDistance_;
        gravitationCenter = gravCenter_;
        gravitationExponent = gravExponent_;
    }

    void setNodesRestriction(const std::unordered_set<int> &myNodesNotMove) {
        nodesNotMove = myNodesNotMove;
    }

    void setGraphicsParams(double alpha_) { edgeOpacity = alpha_; }

    void setGraphLayoutParams(double myRepulsion, double mySpringK, double myAttraction,
                              double myEdgeLength) {
        repulsion = myRepulsion;
        spring_k = mySpringK;
        attraction = myAttraction;
        edgeLength = myEdgeLength;
    }

    void buildCompatibilityLists() {
        int edgesNum = static_cast<int>(edges.size());
        for (int i = 0; i < edgesNum; ++i) {
            for (int j = i + 1; j < edgesNum; ++j) {
                double comp = Edge::angleCompatibility(edges[i], edges[j]) *
                              Edge::scaleCompatibility(edges[i], edges[j]) *
                              Edge::positionCompatibility(edges[i], edges[j]) *
                              Edge::visibilityCompatibility(edges[i], edges[j]);
                if (comp >= compatibilityThreshold) {
                    edges[i].compatibleEdges.push_back(j);
                    edges[j].compatibleEdges.push_back(i);
                }
            }
        }
    }

    void setNodes(const std::unordered_map<std::string, Node> &myNodes) { nodes = myNodes; }

    void setEdges(const std::vector<Edge> &myEdges) { edges = myEdges; }

    void set(const std::unordered_map<std::string, Node> &myNodes, std::vector<Edge> &myEdges) {
        nodes = myNodes;
        std::sort(myEdges.begin(), myEdges.end(),
                  [](const Edge &edge1, const Edge &edge2) { return edge1.width < edge2.width; });

        double wmax = myEdges[0].width;
        if (edgeWeightThreshold > 0.0) {
            for (const auto &edge : myEdges) {
                if (edge.width > edgeWeightThreshold) {
                    edges.push_back(edge);
                    nodes[edge.sourceLabel].degree++;
                    nodes[edge.targetLabel].degree++;
                }
            }
        } else if (edgePercentageThreshold > 0.0) {
            int nEdges = static_cast<int>(edgePercentageThreshold * (myEdges.size()) / 100);
            for (int i = 0; i < nEdges; ++i) {
                edges.push_back(myEdges[i]);
                nodes[myEdges[i].sourceLabel].degree++;
                nodes[myEdges[i].targetLabel].degree++;
            }
        } else {
            for (const auto &edge : myEdges) {
                edges.push_back(edge);
                nodes[edge.sourceLabel].degree++;
                nodes[edge.targetLabel].degree++;
            }
        }

        for (auto &edge : edges) {
            edge.width *= 1.0 / (wmax + 1.0);
        }

        for (const auto &edge : edges) {
            nodePairs.insert({std::stoi(edge.sourceLabel), std::stoi(edge.targetLabel)});
        }
        buildCompatibilityLists();
    }

    static double distance(const glm::vec3 &v1, const glm::vec3 &v2) {
        return std::sqrt(std::pow(v1.x - v2.x, 2.0) + std::pow(v1.y - v2.y, 2.0) +
                         std::pow(v1.z - v2.z, 2.0));
    }
    // 计算度中心性
    std::unordered_map<std::string, double> degree_centrality() {
        std::unordered_map<std::string, double> degree_centrality;
        int n = nodes.size();
        for (const auto &node : nodes) {
            degree_centrality[node.first] = (double)adj_list[node.first].size() / (n - 1);
        }
        return degree_centrality;
    }

    // 计算介数中心性（基于 BFS）
    std::unordered_map<std::string, double> betweenness_centrality() {
        std::unordered_map<std::string, double> betweenness;
        for (const auto &node : nodes) {
            betweenness[node.first] = 0.0;
        }

        for (const auto &s : nodes) {
            std::unordered_map<std::string, double> sigma;
            std::unordered_map<std::string, double> dist;
            std::unordered_map<std::string, std::vector<std::string>> pred;
            std::unordered_map<std::string, double> delta;
            std::stack<std::string> S;
            std::queue<std::string> Q;

            for (const auto &v : nodes) {
                sigma[v.first] = 0.0;
                dist[v.first] = -1;
            }
            sigma[s.first] = 1.0;
            dist[s.first] = 0;

            Q.push(s.first);

            while (!Q.empty()) {
                std::string v = Q.front();
                Q.pop();
                S.push(v);

                for (const std::string &w : adj_list[v]) {
                    if (dist[w] < 0) {
                        Q.push(w);
                        dist[w] = dist[v] + 1;
                    }
                    if (dist[w] == dist[v] + 1) {
                        sigma[w] += sigma[v];
                        pred[w].push_back(v);
                    }
                }
            }

            for (const auto &v : nodes) {
                delta[v.first] = 0.0;
            }

            while (!S.empty()) {
                std::string w = S.top();
                S.pop();
                for (const std::string &v : pred[w]) {
                    delta[v] += (sigma[v] / sigma[w]) * (1 + delta[w]);
                }
                if (w != s.first) {
                    betweenness[w] += delta[w];
                }
            }
        }

        int n = nodes.size();
        for (auto &node : nodes) {
            betweenness[node.first] /= (n - 1) * (n - 2) / 2.0;
        }

        return betweenness;
    }

    // 计算特征向量中心性（幂迭代法）
    std::unordered_map<std::string, double> eigenvector_centrality(int max_iter = 100,
                                                              double tol = 1e-6) {
        std::unordered_map<std::string, double> eigenvector;
        std::unordered_map<std::string, double> last_eigenvector;
        int n = nodes.size();

        // 初始化特征向量
        for (const auto &node : nodes) {
            eigenvector[node.first] = 1.0 / sqrt(n);
        }

        for (int iter = 0; iter < max_iter; iter++) {
            last_eigenvector = eigenvector;
            double max_diff = 0.0;

            for (const auto &node : nodes) {
                double sum = 0.0;
                for (const std::string &neighbor : adj_list[node.first]) {
                    sum += last_eigenvector[neighbor];
                }
                eigenvector[node.first] = sum;
            }

            // 归一化
            double norm = 0.0;
            for (const auto &node : nodes) {
                norm += eigenvector[node.first] * eigenvector[node.first];
            }
            norm = sqrt(norm);
            for (auto &node : nodes) {
                eigenvector[node.first] /= norm;
            }

            // 检查收敛
            for (const auto &node : nodes) {
                max_diff =
                    std::max(max_diff, fabs(eigenvector[node.first] - last_eigenvector[node.first]));
            }
            if (max_diff < tol)
                break;
        }

        return eigenvector;
    }
    void calculateGeoNodeLevel() {
        // 计算顶点的权重
        for (const auto &edge : edges) {
            adj_list[edge.sourceLabel].push_back(edge.targetLabel);
            adj_list[edge.targetLabel].push_back(edge.sourceLabel); // 无向图
        }

        std::unordered_map<std::string, double> degree = degree_centrality();
        std::unordered_map<std::string, double> betweenness = betweenness_centrality();
        std::unordered_map<std::string, double> eigenvector = eigenvector_centrality();

        for (auto &node : nodes) {
            std::string id = node.first;
            // 直接将权重存储在节点的 level 字段中
            node.second.level = 0.5 * betweenness[id] + 0.3 * eigenvector[id] + 0.2 * degree[id];
        }


    }
    // 计算节点的 PageRank 值，并直接覆盖到每个节点的 level 字段中
    void calculate_pagerank(double damping_factor = 0.85, int max_iter = 100, double tol = 1e-6) {
        std::unordered_map<std::string, double> pagerank;
        std::unordered_map<std::string, double> last_pagerank;
        int n = nodes.size();
        double initial_rank = 1.0 / n;

        // 初始化每个节点的 PageRank 值
        for (const auto &node : nodes) {
            pagerank[node.first] = initial_rank;
        }

        for (int iter = 0; iter < max_iter; iter++) {
            last_pagerank = pagerank;
            double diff = 0.0;

            // 遍历每个节点，计算新的 PageRank
            for (const auto &node : nodes) {
                std::string node_id = node.first;
                double rank_sum = 0.0;

                for (const auto &pair : adj_list) {
                    const std::string &source_id = pair.first;               // 当前节点的 ID
                    const std::vector<std::string> &neighbors = pair.second; // 当前节点的邻居列表

                    // 检查当前节点是否指向 node_id
                    if (find(neighbors.begin(), neighbors.end(), node_id) != neighbors.end()) {
                        // 如果找到指向 node_id 的边，累加 source_id 的 PageRank
                        rank_sum += last_pagerank[source_id] / adj_list[source_id].size();
                    }
                }

                // 更新当前节点的 PageRank 值
                pagerank[node_id] = (1 - damping_factor) / n + damping_factor * rank_sum;
                diff = std::max(diff, fabs(pagerank[node_id] - last_pagerank[node_id]));
            }

            // 检查是否收敛
            if (diff < tol) {
                break;
            }
        }
        // 将 PageRank 值存储在节点的 level 字段中
        for (auto &node : nodes) {
            node.second.level = pagerank[node.first];
        }
    }
    void calculateNoGeoNodeLevel() {
        // 计算顶点的权重
        for (const auto &edge : edges) {
            adj_list[edge.sourceLabel].push_back(edge.targetLabel);
            adj_list[edge.targetLabel].push_back(edge.sourceLabel); // 无向图
        }
        calculate_pagerank();
    }
    void calculateGeoEdgeWeight() {
        // 计算边的权重
    }
    void calculateNoGeoEdgeWeight() {
        std::unordered_map<std::string, double> edge_betweenness;

        for (const auto &node_pair : nodes) {
            std::string s = node_pair.first;
            // BFS 初始化
            std::unordered_map<std::string, int> distance;
            std::unordered_map<std::string, std::vector<std::string>> predecessors;
            std::unordered_map<std::string, double> sigma;
            std::unordered_map<std::string, double> delta;
            std::stack<std::string> node_stack;
            std::queue<std::string> q;

            // 初始化
            for (const auto &node : nodes) {
                distance[node.first] = -1; // -1 表示未访问
                sigma[node.first] = 0;
                delta[node.first] = 0;
            }

            distance[s] = 0;
            sigma[s] = 1;
            q.push(s);

            // BFS 阶段
            while (!q.empty()) {
                std::string v = q.front();
                q.pop();
                node_stack.push(v);

                for (const auto &neighbor : adj_list[v]) {
                    // 未访问的节点
                    if (distance[neighbor] < 0) {
                        distance[neighbor] = distance[v] + 1;
                        q.push(neighbor);
                    }
                    // 更新前驱节点
                    if (distance[neighbor] == distance[v] + 1) {
                        sigma[neighbor] += sigma[v];
                        predecessors[neighbor].push_back(v);
                    }
                }
            }

            // 反向传播阶段
            while (!node_stack.empty()) {
                std::string w = node_stack.top();
                node_stack.pop();

                for (const auto &v : predecessors[w]) {
                    double c = (sigma[v] / sigma[w]) * (1.0 + delta[w]);
                    delta[v] += c;

                    // 找到边 (v, w)，增加其介数中心性
                    for (auto &edge : edges) {
                        if ((edge.sourceLabel == v && edge.targetLabel == w) ||
                            (edge.sourceLabel == w && edge.targetLabel == v)) {
                            edge.weight += c;
                        }
                    }
                }
            }
        }

        // 最后将介数中心性值除以 2（因为每条边在无向图中被计算了两次）
        for (auto &edge : edges) {
            edge.weight /= 2.0;
        }
    }
};

} // namespace VIS4Earth

#endif // VIS4EARTH_COMMON_GRAPH_H
