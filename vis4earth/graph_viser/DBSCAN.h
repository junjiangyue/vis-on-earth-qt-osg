#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

// ����ŷ����þ���
float calculateDistance(const osg::Vec3 &pos1, const osg::Vec3 &pos2) {
    osg::Vec3 diff = pos1 - pos2;
    return diff.length();
}

// ��ȡ������� epsilon ����
std::vector<int> regionQuery(int pointIndex, const std::vector<osg::Vec3> &positions,
                             float epsilon) {
    std::vector<int> neighbors;
    const osg::Vec3 &point = positions[pointIndex];

    // �������е㣬�ҵ�����С�� epsilon �ĵ�
    for (int i = 0; i < positions.size(); ++i) {
        if (i != pointIndex && calculateDistance(point, positions[i]) <= epsilon) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

// DBSCAN �㷨�����ڵ��λ�ã�osg::Vec3�����о���
std::vector<int> DBSCAN(const std::vector<osg::Vec3> &positions, float epsilon, int minPts) {
    std::vector<int> labels(positions.size(), -1); // -1 ��ʾδ����
    int clusterId = 0;
    std::vector<bool> visited(positions.size(), false); // ��¼ÿ�����Ƿ���ʹ�

    // �������е�
    for (int i = 0; i < positions.size(); ++i) {
        if (visited[i])
            continue; // �����Ѿ����ʹ��ĵ�

        visited[i] = true;

        // �ҵ��õ�� epsilon ����
        std::vector<int> neighbors = regionQuery(i, positions, epsilon);

        // ��������еĵ���С�� minPts������Ϊ������
        if (neighbors.size() < minPts) {
            labels[i] = -1; // ������
        } else {
            // �����γ��µĴ�
            labels[i] = clusterId;

            // �������ڵĵ�����ӵ�����
            std::set<int> neighborSet(neighbors.begin(), neighbors.end());

            // ��չ��
            while (!neighborSet.empty()) {
                auto iter = neighborSet.begin();
                int currentNeighborIndex = *iter;
                neighborSet.erase(iter);

                if (!visited[currentNeighborIndex]) {
                    visited[currentNeighborIndex] = true;

                    // �ҵ���ǰ��� epsilon ����
                    std::vector<int> neighborNeighbors =
                        regionQuery(currentNeighborIndex, positions, epsilon);
                    if (neighborNeighbors.size() >= minPts) {
                        neighborSet.insert(neighborNeighbors.begin(), neighborNeighbors.end());
                    }
                }

                // ����õ㻹δ�����䵽���У�������ӵ���ǰ��
                if (labels[currentNeighborIndex] == -1) {
                    labels[currentNeighborIndex] = clusterId;
                }
            }
            // ��ɵ�ǰ�ص���չ��׼����һ����
            clusterId++;
        }
    }
    return labels;
}