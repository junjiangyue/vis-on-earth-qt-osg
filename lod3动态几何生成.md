

---

## 会话总结：LOD3动态几何生成系统设计

### 会话主要目的
设计LOD3专用的动态几何生成系统，解决复杂曲线边（具有起点、n个控制点、终点）的高性能渲染问题，实现视锥剔除与增量更新的完美结合。

### 问题背景分析

**LOD3层级特点**：
- 边数据具有复杂几何结构：起点 + n个控制点 + 终点
- 每条边的几何形状完全不同（不适合传统实例化渲染）
- 经过视锥剔除后，可见边数量有限（几百到几千条）
- 频繁的视角变化导致可见性频繁变化

**当前视锥剔除机制**：
- **第一阶段**：地理网格粗筛（复用`EarthGridPartition`）
- **第二阶段**：精确视锥测试（`performPreciseCulling`）
- **问题**：每次视角变化都需要全量重建边几何体，造成性能瓶颈

### 解决方案：动态几何生成系统

#### 核心设计理念

**动态几何生成定义**：
- 根据需要**实时构建和修改几何体数据**的渲染技术
- 只为当前可见的对象生成几何体
- 支持精确的增量添加/删除操作
- 避免传统的"全部删除→全部重建"模式

**技术优势**：
```cpp
// 传统方式（全量重建）
void traditionalUpdate() {
    clearAllEdgeGeometry();           // 删除所有边
    for (auto& edge : allEdges) {
        if (edge.visible) {
            createEdgeGeometry(edge); // 重新创建所有可见边
        }
    }
    uploadCompleteVBO();
}

// 动态几何生成（增量更新）
void incrementalUpdate() {
    auto changes = detectVisibilityChanges();
    
    for (const auto& newEdgeId : changes.newlyVisible) {
        addSingleEdgeGeometry(newEdgeId);     // 只添加新出现的边
    }
    
    for (const auto& hiddenEdgeId : changes.newlyHidden) {
        removeSingleEdgeGeometry(hiddenEdgeId); // 只删除消失的边
    }
    // 其他边保持不变，无需重新计算
}
```

#### 系统架构设计

**核心组件**：
```
LOD3DynamicGeometrySystem
├── DynamicGeometryManager      // 动态几何体管理器
├── IncrementalUpdater         // 增量更新协调器
├── CurveGeometryGenerator     // 曲线几何体生成器
└── VisibilityChangeTracker    // 可见性变化跟踪器
```

**新增头文件**：`vis4earth/graph_viser/lod3_dynamic_geometry.h`

#### 详细组件设计

**1. DynamicGeometryManager**
```cpp
class DynamicGeometryManager {
public:
    // 初始化与管理
    void initialize(osg::Group* parentGroup);
    void cleanup();
    
    // 增量几何体操作
    void addEdge(const std::string& edgeId, const Edge& edge, 
                 const Node& fromNode, const Node& toNode);
    void removeEdge(const std::string& edgeId);
    void batchUpdate(const std::vector<std::string>& toAdd,
                     const std::vector<std::string>& toRemove,
                     const std::map<std::string, Edge>& edgeData);
    
    // 状态查询
    int getActiveEdgeCount() const;
    bool hasEdge(const std::string& edgeId) const;
    
private:
    struct EdgeGeometryInfo {
        int startVertexIndex;    // VBO中的起始顶点索引
        int vertexCount;         // 顶点数量
        int startIndexIndex;     // 索引数组中的起始位置
        int indexCount;          // 索引数量
    };
    
    std::map<std::string, EdgeGeometryInfo> edgeGeometryMap;
    std::vector<int> freeVertexSlots;  // 可复用的顶点slot
    osg::ref_ptr<osg::Geometry> dynamicGeometry;
    osg::ref_ptr<osg::Vec3Array> vertexArray;
    osg::ref_ptr<osg::Vec4Array> colorArray;
};
```

**2. CurveGeometryGenerator**
```cpp
class CurveGeometryGenerator {
public:
    // 曲线几何体生成
     // 曲线算法
    //lod3曲线的生成逻辑和update里面原有的逻辑一样，就是插值寻找最高点。
    std::vector<osg::Vec3> generateCurveVertices(const Edge& edge);
    std::vector<osg::Vec4> generateCurveColors(const Edge& edge);
    std::vector<unsigned int> generateCurveIndices(int vertexCount);    
    // 配置参数
    void setTessellationLevel(float level);  // 曲线细分精度
    void setAdaptiveTessellation(bool enable); // 自适应细分
    
private:
    float tessellationLevel = 0.02f;  // 默认50个点的精度
    bool adaptiveTessellation = false;
};
```

**3. VisibilityChangeTracker**
```cpp
class VisibilityChangeTracker {
public:
    struct VisibilityChanges {
        std::vector<std::string> newlyVisible;   // 新出现的边ID
        std::vector<std::string> newlyHidden;    // 新消失的边ID
        bool hasChanges() const;
    };
    
    // 变化检测
    VisibilityChanges detectChanges(const std::vector<Edge>& currentEdges);
    void updateLastState(const std::set<std::string>& currentVisible);
    
    // 性能优化
    void enableBatchDetection(bool enable);
    void setDetectionThreshold(int minChanges);
    
private:
    std::set<std::string> lastVisibleEdges;
    bool batchDetectionEnabled = true;
    int detectionThreshold = 5;  // 最少5个变化才触发更新
};
```

#### 与现有系统集成

**需要修改的核心函数**：

**1. GraphRenderer::updateActiveLOD()**
```cpp
// 在updateActiveLOD中集成LOD3动态几何生成
void GraphRenderer::updateActiveLOD(double cameraHeight) {
    int targetLevel = getCurrentLevel(cameraHeight);
    
    if (targetLevel == 3) {
        // LOD3：启用动态几何生成系统
        if (!lod3DynamicSystem) {
            lod3DynamicSystem = std::make_unique<LOD3DynamicGeometrySystem>();
            lod3DynamicSystem->initialize(param.grp.get());
        }
        
        // 同步LOD数据源，但不立即更新
        lod3DynamicSystem->getStateSynchronizer().syncLODDataSource(
            lodNodesData[3], lodEdgesData[3]);
        
        // 请求更新（防抖处理）
        lod3DynamicSystem->getScheduler().requestUpdate(
            LOD3UpdateScheduler::LOD_SWITCH);

    } else {
        // 其他LOD：使用传统VBO渲染
        if (lod3DynamicSystem) {
            lod3DynamicSystem->cleanup();
        }
        // 使用原有逻辑...
    }
}
```

**2. GraphRenderer::frustumCulling()**
```cpp
// 修改视锥剔除以支持LOD3动态更新
void GraphRenderer::frustumCulling(const std::string &graphName, double minLon,
                                   double maxLon, double minLat, double maxLat,
                                   const int currentLevel) {
    // 执行传统的视锥剔除逻辑
    // 设置 edge.visible 和 node.visible 属性
    
    if (currentLevel == 3 && lod3DynamicSystem) {
        // LOD3特殊处理：只同步可见性状态，不立即更新
        lod3DynamicSystem->getStateSynchronizer().syncVisibilityState(
            *graphParam->edges, *graphParam->nodes);
        
        // 请求更新（防抖处理）
        lod3DynamicSystem->getScheduler().requestUpdate(
            LOD3UpdateScheduler::FRUSTUM_CULLING);
            

    } else {
        // 其他LOD：传统更新方式保持不变
        // 原有逻辑...
    }
}
```

**3. PerGraphParam::update()**
```cpp
// 修改PerGraphParam的update方法
void GraphRenderer::PerGraphParam::update() {
    if (currentLODLevel == 3) {
        // LOD3：跳过传统VBO更新，使用动态几何系统
        // 动态几何系统会在视锥剔除时自动更新
        return;
    } else {
        // 其他LOD：使用传统updateEdgeVBO
        updateEdgeVBO();
    }
}
```

**4. 新增专用接口**
```cpp
// 在GraphRenderer类中新增LOD3专用接口
class GraphRenderer {
private:
    std::unique_ptr<LOD3DynamicGeometrySystem> lod3DynamicSystem;
    
public:
    // LOD3动态几何系统控制接口
    void enableLOD3DynamicGeometry(bool enable);
    void setLOD3TessellationLevel(float level);
    LOD3GeometryStats getLOD3Stats() const;
};
```

#### 新增专用接口详细说明

**接口设计目的**：
- 提供LOD3动态几何系统的外部控制入口
- 支持运行时参数调整和性能监控
- 实现系统级别的开关控制和调试支持

**1. enableLOD3DynamicGeometry(bool enable)**
```cpp
void GraphRenderer::enableLOD3DynamicGeometry(bool enable) {
    if (enable) {
        // 启用LOD3动态几何系统
        if (!lod3DynamicSystem) {
            lod3DynamicSystem = std::make_unique<LOD3DynamicGeometrySystem>();
            lod3DynamicSystem->initialize(param.grp.get());
        }
        lod3DynamicSystem->setEnabled(true);
    } else {
        // 禁用并回退到传统VBO渲染
        if (lod3DynamicSystem) {
            lod3DynamicSystem->setEnabled(false);
        }
        // 切换回传统LOD3渲染方式
        if (param.currentLODLevel == 3) {
            param.update(); // 强制使用传统VBO更新
        }
    }
}
```

**用途**：
- **系统级开关**：允许用户在运行时启用/禁用LOD3动态几何系统
- **性能对比**：可以在动态几何和传统VBO渲染之间切换，便于性能对比测试
- **故障回退**：当动态几何系统出现问题时，可以快速回退到稳定的传统渲染方式
- **调试支持**：开发和调试阶段可以随时切换渲染方式验证效果

**使用场景**：
```cpp
// 在UI界面中添加开关控件
if (useDynamicGeometry) {
    graphRenderer->enableLOD3DynamicGeometry(true);
} else {
    graphRenderer->enableLOD3DynamicGeometry(false);
}
```

**2. setLOD3TessellationLevel(float level)**
```cpp
void GraphRenderer::setLOD3TessellationLevel(float level) {
    if (lod3DynamicSystem) {
        lod3DynamicSystem->getGeometryGenerator().setTessellationLevel(level);
        
        // 如果当前正在LOD3模式，触发重新生成
        if (param.currentLODLevel == 3) {
            lod3DynamicSystem->regenerateAllVisibleEdges();
        }
    }
}
```

**用途**：
- **质量控制**：动态调整曲线细分精度，平衡渲染质量和性能
- **性能调优**：根据硬件性能和场景复杂度实时调整细分级别
- **自适应渲染**：可以根据相机距离或边的重要性动态调整不同边的细分精度
- **用户偏好**：允许用户根据视觉需求选择不同的渲染精度

**参数说明**：
```cpp
// tessellationLevel范围建议: 0.005 - 0.1
// 0.005: 高精度 (200个点/曲线) - 适合近距离观察
// 0.02:  中等精度 (50个点/曲线)  - 默认值，平衡质量和性能  
// 0.05:  低精度 (20个点/曲线)  - 适合远距离或低性能设备
// 0.1:   极低精度 (10个点/曲线) - 性能优先模式
```

**使用场景**：
```cpp
// 根据相机高度自适应调整精度
void GraphRenderer::onCameraHeightChanged(double height) {
    if (height < 1000) {
        setLOD3TessellationLevel(0.005f); // 近距离高精度
    } else if (height < 5000) {
        setLOD3TessellationLevel(0.02f);  // 中距离中精度
    } else {
        setLOD3TessellationLevel(0.05f);  // 远距离低精度
    }
}
```

**3. getLOD3Stats() const**
```cpp
struct LOD3GeometryStats {
    int totalEdgesInLOD3;        // LOD3层级总边数
    int currentVisibleEdges;     // 当前可见边数
    int activeGeometryObjects;   // 活跃的几何体对象数
    
    // 性能统计
    float lastUpdateTime;        // 上次更新耗时(ms)
    float averageUpdateTime;     // 平均更新耗时(ms)
    int updateCount;             // 更新次数
    
    // 内存使用统计
    int vboVertexCount;          // VBO中的顶点总数
    int vboMemoryUsage;          // VBO内存使用量(字节)
    float memoryFragmentation;   // 内存碎片率(0-1)
    
    // 变化统计
    int edgesAddedThisFrame;     // 本帧新增的边数
    int edgesRemovedThisFrame;   // 本帧移除的边数
    float visibilityChangeRate;  // 可见性变化率
};

LOD3GeometryStats GraphRenderer::getLOD3Stats() const {
    if (lod3DynamicSystem) {
        return lod3DynamicSystem->getPerformanceStats();
    }
    return LOD3GeometryStats{}; // 返回空统计
}
```

**用途**：
- **性能监控**：实时监控LOD3动态几何系统的性能表现
- **内存管理**：跟踪VBO内存使用情况，识别内存泄漏或碎片问题
- **调试分析**：提供详细的运行时数据，便于性能调优和问题排查
- **用户反馈**：在UI中显示系统状态，让用户了解当前渲染负载

**使用场景**：
```cpp
// 在调试界面中显示性能统计
void GraphRenderer::updateDebugInfo() {
    auto stats = getLOD3Stats();
    
    debugUI->setText(QString(
        "LOD3 Dynamic Geometry Stats:\n"
        "Visible Edges: %1/%2\n"
        "Update Time: %.2f ms\n"
        "Memory Usage: %.1f MB\n"
        "Fragmentation: %.1f%%"
    ).arg(stats.currentVisibleEdges)
     .arg(stats.totalEdgesInLOD3)
     .arg(stats.lastUpdateTime)
     .arg(stats.vboMemoryUsage / 1024.0f / 1024.0f)
     .arg(stats.memoryFragmentation * 100.0f));
}

// 自动性能优化
void GraphRenderer::autoOptimizeLOD3() {
    auto stats = getLOD3Stats();
    
    // 如果更新时间过长，降低细分精度
    if (stats.averageUpdateTime > 20.0f) { // 超过20ms
        float currentLevel = lod3DynamicSystem->getTessellationLevel();
        setLOD3TessellationLevel(currentLevel * 1.5f); // 降低精度
    }
    
    // 如果内存碎片严重，触发整理
    if (stats.memoryFragmentation > 0.3f) { // 超过30%碎片
        lod3DynamicSystem->compactVBOMemory();
    }
}
```

#### 接口集成示例

**完整的LOD3控制接口使用示例**：
```cpp
class LOD3ControlPanel {
public:
    void initialize(GraphRenderer* renderer) {
        this->renderer = renderer;
        
        // 启用动态几何系统
        renderer->enableLOD3DynamicGeometry(true);
        
        // 设置初始参数
        renderer->setLOD3TessellationLevel(0.02f);
        
        // 启动性能监控定时器
        QTimer* statsTimer = new QTimer(this);
        connect(statsTimer, &QTimer::timeout, this, &LOD3ControlPanel::updateStats);
        statsTimer->start(1000); // 每秒更新一次统计
    }
    
private slots:
    void onEnableChanged(bool enabled) {
        renderer->enableLOD3DynamicGeometry(enabled);
    }
    
    void onQualityChanged(int quality) {
        // 质量档位: 1(低) - 5(高)
        float tessellationLevels[] = {0.1f, 0.05f, 0.02f, 0.01f, 0.005f};
        renderer->setLOD3TessellationLevel(tessellationLevels[quality-1]);
    }
    
    void updateStats() {
        auto stats = renderer->getLOD3Stats();
        
        // 更新UI显示
        edgeCountLabel->setText(QString("%1/%2").arg(stats.currentVisibleEdges).arg(stats.totalEdgesInLOD3));
        performanceLabel->setText(QString("%.1f ms").arg(stats.lastUpdateTime));
        memoryLabel->setText(QString("%.1f MB").arg(stats.vboMemoryUsage / 1024.0f / 1024.0f));
        
        // 性能警告
        if (stats.lastUpdateTime > 16.7f) { // 超过60FPS
            performanceWarning->show();
        } else {
            performanceWarning->hide();
        }
    }
    
private:
    GraphRenderer* renderer;
    QLabel* edgeCountLabel;
    QLabel* performanceLabel;
    QLabel* memoryLabel;
    QLabel* performanceWarning;
};
```

#### 性能优化策略

**1. 批量更新控制**
```cpp
struct BatchUpdateConfig {
    int maxBatchSize = 50;           // 每次最多更新50条边
    float updateInterval = 16.0f;    // 16ms更新间隔（60FPS）
    bool enableTimeSlicing = true;   // 时间分片
};
```

**2. VBO空间管理**
```cpp
class VBOSpaceManager {
    std::vector<bool> slotOccupied;     // slot占用状态
    std::queue<int> freeSlots;          // 空闲slot队列
    bool needsCompaction;               // 是否需要整理
    
    // 当空闲slot超过30%时进行VBO整理
    void compactIfNeeded();
};
```

**3. 自适应曲线细分**
```cpp
// 根据相机距离和边权重动态调整曲线精度
float calculateTessellationLevel(const Edge& edge, double cameraDistance) {
    float baseTessellation = 0.02f;
    float distanceFactor = std::min(1.0f, 10000.0f / cameraDistance);
    float weightFactor = std::min(2.0f, edge.weight);
    return baseTessellation * distanceFactor * weightFactor;
}
```


### 文件结构设计

**新增文件**：
```
vis4earth/graph_viser/lod3_dynamic_geometry.h
├── LOD3DynamicGeometrySystem     // 主系统类
├── DynamicGeometryManager        // 几何体管理器
├── CurveGeometryGenerator        // 曲线生成器
├── VisibilityChangeTracker       // 变化跟踪器
└── 相关数据结构定义
```

**修改文件**：
```
vis4earth/graph_viser/graph_display.h
├── 添加LOD3动态几何系统成员变量
├── 添加LOD3专用接口声明
└── 添加性能统计结构

vis4earth/graph_viser/graph_display.cpp  
├── 修改updateActiveLOD()函数
├── 修改frustumCulling()函数
├── 修改PerGraphParam::update()函数
└── 集成LOD3动态几何系统调用
```

这次设计彻底解决了LOD3层级复杂曲线边的高性能渲染问题，通过动态几何生成实现了视锥剔除与增量更新的完美结合，为用户提供流畅的交互体验。

## 补充设计：防抖机制与更新调度优化

### 问题分析

**重复触发问题**：
- `frustumCulling()` 会触发LOD3更新
- `updateActiveLOD()` 也会触发LOD3更新  
- 在相机移动时，两个函数可能在短时间内连续调用，导致重复计算

**性能影响**：
```cpp
// 问题场景：相机移动时的调用序列
1. checkCameraMovement() → updateActiveLOD() → LOD3更新
2. cameraUpdate() → frustumCulling() → LOD3更新 (重复!)
// 结果：同一帧内进行了两次昂贵的几何体更新
```

### 解决方案：智能调度系统

#### 核心设计原则

1. **责任分离**：
   - `frustumCulling()` **只负责标记可见性**，不触发立即更新
   - `updateActiveLOD()` **只负责数据源切换**，不触发立即更新
   - `LOD3DynamicGeometrySystem` **统一负责调度和更新**

2. **防抖机制**：
   - 收集短时间内的多次更新请求
   - 在合适的时机进行一次性批量更新
   - 避免重复计算和GPU状态切换

#### 新增组件设计

**1. 更新调度器 (UpdateScheduler)**
```cpp
class LOD3UpdateScheduler {
public:
    enum UpdateTrigger {
        FRUSTUM_CULLING = 1 << 0,    // 视锥剔除触发
        LOD_SWITCH = 1 << 1,         // LOD切换触发  
        MANUAL_REQUEST = 1 << 2,     // 手动请求触发
        TESSELLATION_CHANGE = 1 << 3 // 细分精度变化触发
    };
    
    // 请求更新（不立即执行）
    void requestUpdate(UpdateTrigger trigger);
    
    // 检查是否需要执行更新
    bool shouldUpdate() const;
    
    // 执行批量更新并清空请求
    void executeUpdate();
    
    // 配置防抖参数
    void setDebounceInterval(float intervalMs);
    void setMaxPendingTime(float maxMs);
    
private:
    struct UpdateRequest {
        UpdateTrigger trigger;
        osg::Timer_t timestamp;
        int frameNumber;
    };
    
    std::vector<UpdateRequest> pendingRequests;
    float debounceInterval = 16.0f;  // 16ms防抖间隔
    float maxPendingTime = 50.0f;    // 最大50ms必须更新
    osg::Timer_t lastUpdateTime;
};
```

```

#### 修改现有接口

**1. 修改 frustumCulling() - 只标记，不更新**
```cpp
void GraphRenderer::frustumCulling(const std::string &graphName, double minLon,
                                   double maxLon, double minLat, double maxLat,
                                   const int currentLevel) {
    // 执行传统的视锥剔除逻辑
    // 设置 edge.visible 和 node.visible 属性
    
    if (currentLevel == 3 && lod3DynamicSystem) {
        // LOD3特殊处理：只同步可见性状态，不立即更新
        lod3DynamicSystem->getStateSynchronizer().syncVisibilityState(
            *graphParam->edges, *graphParam->nodes);
        
        // 请求更新（防抖处理）
        lod3DynamicSystem->getScheduler().requestUpdate(
            LOD3UpdateScheduler::FRUSTUM_CULLING);
            
        // 不再调用立即更新
        // lod3DynamicSystem->updateFromFrustumCulling(); // 删除这行
    } else {
        // 其他LOD：传统更新方式保持不变
        // 原有逻辑...
    }
}
```

**2. 修改 updateActiveLOD() - 只切换数据源**
```cpp
void GraphRenderer::updateActiveLOD(double cameraHeight) {
    int targetLevel = getCurrentLevel(cameraHeight);
    
    if (targetLevel == 3) {
        // LOD3：启用动态几何生成系统
        if (!lod3DynamicSystem) {
            lod3DynamicSystem = std::make_unique<LOD3DynamicGeometrySystem>();
            lod3DynamicSystem->initialize(param.grp.get());
        }
        
        // 同步LOD数据源，但不立即更新
        lod3DynamicSystem->getStateSynchronizer().syncLODDataSource(
            lodNodesData[3], lodEdgesData[3]);
        
        // 请求更新（防抖处理）
        lod3DynamicSystem->getScheduler().requestUpdate(
            LOD3UpdateScheduler::LOD_SWITCH);
            
        // 不再调用立即更新
        // lod3DynamicSystem->updateFromLODSwitch(); // 删除这行
    } else {
        // 其他LOD：使用传统VBO渲染
        if (lod3DynamicSystem) {
            lod3DynamicSystem->cleanup();
        }
        // 使用原有逻辑...
    }
}
```

#### LOD3DynamicGeometrySystem 扩展

**新增调度相关方法**：
```cpp
class LOD3DynamicGeometrySystem {
public:
    // 现有接口保持不变...
    
    // 新增：获取调度组件
    LOD3UpdateScheduler& getScheduler() { return updateScheduler; }
    
    // 新增：统一更新入口
    void processScheduledUpdates();
    
    // 新增：手动强制更新
    void forceImmediateUpdate();
    
    
private:
    LOD3UpdateScheduler updateScheduler;

    
    // 新增：执行实际的几何体更新
    void executeGeometryUpdate();
};
```

#### 统一更新调度逻辑

**主更新循环集成**：
```cpp
// 在GraphRenderer的主更新循环中添加
void GraphRenderer::update() {
    // 现有更新逻辑...
    
    // LOD3动态几何调度处理
    if (lod3DynamicSystem) {
        lod3DynamicSystem->processScheduledUpdates();
    }
}
```

**processScheduledUpdates 实现**：
```cpp
void LOD3DynamicGeometrySystem::processScheduledUpdates() {
    // 检查是否有待处理的更新请求
    if (!updateScheduler.shouldUpdate()) {
        return;
    }
    
    // 获取状态变化
    bool hasVisibilityChanges = stateSynchronizer.hasVisibilityChanges();
    bool hasDataSourceChanges = stateSynchronizer.hasDataSourceChanges();
    
    if (!hasVisibilityChanges && !hasDataSourceChanges) {
        // 没有实际变化，清空请求
        updateScheduler.executeUpdate();
        return;
    }
    
    // 根据变化类型执行相应更新
    if (hasDataSourceChanges) {
        // 数据源变化：重新初始化
        auto dataChanges = stateSynchronizer.getDataSourceChanges();
        reinitializeWithNewData(dataChanges);
    }
    
    if (hasVisibilityChanges) {
        // 可见性变化：增量更新
        auto visChanges = stateSynchronizer.getVisibilityChanges();
        performIncrementalUpdate(visChanges);
    }
    
    // 标记更新完成
    updateScheduler.executeUpdate();
}
```

#### 性能优化配置

**防抖参数建议**：
```cpp
struct DebounceConfig {
    float minInterval = 8.0f;    // 最小8ms间隔（120FPS限制）
    float maxInterval = 16.0f;   // 最大16ms间隔（60FPS保证）
    float maxPendingTime = 33.0f; // 最大33ms必须更新（30FPS底线）
    
    // 自适应参数
    bool enableAdaptive = true;
    float targetFrameTime = 16.7f; // 目标60FPS
};
```

```


---

## 🗺️ **模块交互关系图**

以下是 **LOD3DynamicGeometrySystem** 及其核心组件的模块交互关系示意图：

```
╔════════════════════════════════════╗
║          GraphRenderer (主入口)      ║
╚════════════════════════════════════╝
              │
              ▼
╔════════════════════════════════════╗
║     LOD3DynamicGeometrySystem (核心管理)  ║
╚════════════════════════════════════╝
     │         │           │          │
     │         │           │          │
     ▼         ▼           ▼          ▼
╔══════════╗ ╔══════════╗ ╔══════════╗ ╔═════════════════╗
║ Dynamic  ║ ║ Curve    ║ ║ Visibility║ ║  LOD3Update     ║
║ Geometry ║ ║ Geometry ║ ║ Change    ║ ║ Scheduler (调度) ║
║ Manager  ║ ║ Generator║ ║ Tracker   ║ ║                 ║
╚══════════╝ ╚══════════╝ ╚══════════╝ ╚═════════════════╝
      │           │             │
      │           │             ▼
      │           │      状态同步调用
      │           └──> 曲线生成逻辑调用
      ▼
动态几何缓冲区（VBO/IBO、顶点/颜色数组）
```

模块职责简述：
- **GraphRenderer**：触发系统初始化、更新、销毁。
- **LOD3DynamicGeometrySystem**：主控中心，管理调度、状态同步、更新。
- **DynamicGeometryManager**：管理VBO分配、增删几何体。
- **CurveGeometryGenerator**：生成曲线顶点、颜色、索引。
- **VisibilityChangeTracker**：管理视锥剔除可见性变化。
- **LOD3UpdateScheduler**：防抖与更新调度策略核心。

---

## ⏱️ **更新调度时序图**

以下是 **LOD3更新流程时序**（含防抖机制）的完整流程图（简化版）：

```
时序轴: ─────────────────────────────────────────────>

GraphRenderer::updateActiveLOD()      (触发LOD切换)
GraphRenderer::frustumCulling()       (触发视锥剔除)
       │                                      │
       ▼                                      ▼
LOD3DynamicGeometrySystem::StateSynchronizer::syncLODDataSource()
LOD3DynamicGeometrySystem::StateSynchronizer::syncVisibilityState()
       │                                      │
       └──────────────┬───────────────────────┘
                      ▼
      LOD3DynamicGeometrySystem::UpdateScheduler::requestUpdate()
                      │
                      ▼
      [等待防抖时间 | 达到最大等待时间 | 外部强制触发]
                      │
                      ▼
      LOD3DynamicGeometrySystem::processScheduledUpdates()
                      │
                      ├── 检查触发源 (FRUSTUM_CULLING, LOD_SWITCH, ...)
                      ├── 检查Visibility/Data变化
                      ├── 调用DynamicGeometryManager/CurveGenerator增删边
                      └── 更新几何体到GPU (VBO Upload)
                      ▼
      LOD3DynamicGeometrySystem::UpdateScheduler::executeUpdate()
```

关键点：
- **触发点**：`updateActiveLOD()` & `frustumCulling()`。
- **更新决策**：由 `UpdateScheduler` 根据时间窗口和触发源决定是否立即更新。
- **执行时机**：统一由 `processScheduledUpdates()` 调度。
- **防止重复更新**：多次触发合并为一次几何更新。

