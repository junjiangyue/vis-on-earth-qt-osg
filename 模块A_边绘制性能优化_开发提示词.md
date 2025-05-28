
# 模块 A 开发提示词：边绘制性能优化（Shader + VBO）

## 任务说明
本任务旨在重构边绘制逻辑，替换原有基于 CPU 的边分段和颜色插值方案，
实现使用 GPU（Shader + VBO）进行高性能批量绘制。
边将具有渐变色、线宽动态控制等属性，以提升渲染性能和视觉表现。

## 目标文件
- `vis4earth/graph_viser/graph_display.cpp`（主要修改 `GraphRenderer::PerGraphParam::update()` 方法）
- 新的 Shader 文件（可以内联GLSL）

## 具体任务

### 1. 清理旧逻辑
移除或注释掉当前 update() 中使用 CPU 构建线段数据并通过 DrawArrays 绘制的部分，
为后续替换为 VBO + Shader 做准备。

### 2. 顶点数据准备
遍历 `*(this->edges)`，只处理 `edge.visible == true` 的边。

- 获取起点、终点节点的颜色和位置。
- 获取边的权重 `edge.weight`。
- 获取插值点 `edge.subDivs` 并转换为球体坐标。
- 对每条细分线段，构建如下结构：

```cpp
struct LineVertex {
    osg::Vec3 position;
    osg::Vec4 colorFrom;
    osg::Vec4 colorTo;
    float weight;
};
```

- 对于 LOD3，抬升插值点高度（遮挡处理）应在 CPU 上完成。

### 3. VBO 构建
将位置、颜色、权重数据分别存入 `osg::Vec3Array`、`Vec4Array`、`FloatArray`。

- 使用 `setVertexAttribArray(location, array, binding)` 设置 attribute。
- 使用 `addPrimitiveSet` 设置为 `LINES` 模式进行绘制。

### 4. Shader 编写（Vertex）
Vertex Shader 接收属性：

```glsl
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColorFrom;
layout(location = 2) in vec4 aColorTo;
layout(location = 3) in float aWeight;
```

传出变量：`vColorFrom`, `vColorTo`, `vWeight`（和可能的 `vT` 插值因子）。

计算：

```glsl
gl_Position = gl_ModelViewProjectionMatrix * vec4(aPos, 1.0);
```

### 5. Shader 编写（Fragment）
- 接收 `vColorFrom / vColorTo / vWeight`。
- 若需颜色渐变，建议为每段线提供归一化插值因子 `vT`。
- 实现：

```glsl
vec4 finalColor = mix(vColorFrom, vColorTo, vT);
```

- 输出：

```glsl
fragColor = finalColor;
```

> 注：如无 Geometry Shader，线宽控制建议先用统一 `LineWidth` 状态。

### 6. Shader 集成
- 创建 `osg::Program`，附加 Vertex 和 Fragment Shader。
- 使用 `addBindAttribLocation` 对应属性位置。
- 在 `lineGeode->getOrCreateStateSet()` 设置 program 和属性。
- 启用 `GL_BLEND`，设置 `BlendFunc` 以支持透明度。

## 开发建议与注意事项

- 插值因子 `vT` 可通过顶点属性传入（0~1），用于颜色渐变。
- 地形遮挡高度计算必须在 CPU 中完成，Shader 不具备访问地形高度的能力。
- OSG 3.4 支持 OpenGL 3.3，可使用 GLSL 330 编写 Shader。
- 若后续支持真实线宽建议引入 Geometry Shader 实现 quad 展开。
- 调试异常时请验证：
  - Shader 是否成功编译；
  - Program 是否绑定；
  - Attribute 位置是否对齐。
