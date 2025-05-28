
# 提示词 2：实现模块 B - 视觉表现增强（仅限新绘制路径）

**目标文件**:  
- `vis4earth/graph_viser/graph_display.cpp` （修改 `GraphRenderer::PerGraphParam::update()` 与 `updateEdgeVBO()`）  
- `vis4earth/graph_viser/graph_display.h` （确认 `Node` 结构体中含有 `color` 成员）

---

## 任务目标

增强图形视觉表现，在新绘制路径（`mUseNewRenderer == true`）下实现：

1. ✅ 节点颜色：使用 `Node::color` 设置；  
2. ✅ 边颜色：统一为起点与终点颜色的平均值（非渐变）；  
3. ⚠️ 不修改绘制逻辑或动画效果，仅优化颜色上传；

---

## 具体任务说明

### 1. 节点颜色设置

在 `update()` 方法中构建 `osg::Sphere` 时：

```cpp
sphere->setColor(osg::Vec4(itr->second.color, 1.0f));
```

不再使用 `generateColor()`，也不再基于 cluster 设置颜色。

---

### 2. 边颜色统一为混合色

在 `updateEdgeVBO()` 中，为每条边计算混合色：

```cpp
osg::Vec4 fromColor = osg::Vec4(nodes->at(edge.from).color, 1.0f);
osg::Vec4 toColor   = osg::Vec4(nodes->at(edge.to).color, 1.0f);
osg::Vec4 mixColor  = (fromColor + toColor) * 0.5f;
```

然后填入：

```cpp
mColorFromArray->push_back(mixColor);
mColorToArray->push_back(mixColor);
```

---

### 3. Shader 兼容逻辑（⚠️ 禁止修改动画）

- 在 Shader 中可直接使用：

```glsl
vec4 baseColor = vColorFrom;
```

或保留：

```glsl
vec4 baseColor = mix(vColorFrom, vColorTo, 0.5);
```

- **禁止修改 Shader 中已有的动画高亮逻辑**，如：

```glsl
uEnableHighlight, uLineDataTex, uHighlightColor 等处理逻辑。
```

这些用于线段流动特效，必须保留不动。

---

## 附加说明

- ❌ 不修改绘制结构、渲染管线；
- ❌ 不启用颜色插值或动态变换；
- ✅ 保留现有动画；
- ✅ 使用静态颜色增强节点与边的统一表现。

---

## 效果预期

- 视觉表现更统一、清晰；
- 不影响性能与已有动画；
- GPU 处理简洁，利于调试与后续扩展。

---
