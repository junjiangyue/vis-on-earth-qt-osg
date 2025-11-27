#ifndef VIS4EARTH_GRAPH_VISER_GRAPH_ANIMATION_H
#define VIS4EARTH_GRAPH_VISER_GRAPH_ANIMATION_H

#include <osg/AnimationPath>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Image>
#include <osg/NodeCallback>
#include <osg/Program>
#include <osg/Referenced>
#include <osg/Shader>
#include <osg/Timer>
#include <osg/Vec3>
#include <osg/Vec4>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <cmath>

namespace VIS4Earth {

// 前向声明
class GraphRenderer;

namespace GraphAnimation {

// ============================================================================
// 时间控制器
// ============================================================================

/**
 * @brief 时间控制器，用于获取动画时间
 */
class TimeController : public osg::Referenced {
public:
    TimeController() : startTime(osg::Timer::instance()->tick()) {}
    
    /**
     * @brief 获取从创建时起经过的时间
     * @return 经过的时间（秒）
     */
    float getTime() {
        return osg::Timer::instance()->delta_s(startTime, osg::Timer::instance()->tick());
    }

private:
    osg::Timer_t startTime;
};

// ============================================================================
// 箭头动画回调
// ============================================================================

/**
 * @brief 箭头动画回调，控制箭头的透明度动画
 */
class ArrowAnimationCallback : public osg::NodeCallback {
public:
    /**
     * @brief 构造函数
     * @param colors 颜色数组
     * @param geometry 几何体
     * @param animationCallback 动画路径回调
     */
    ArrowAnimationCallback(osg::Vec4Array* colors, 
                          osg::Geometry* geometry,
                          osg::AnimationPathCallback* animationCallback)
        : colors_(colors)
        , geometry_(geometry)
        , animationCallback_(animationCallback) {
    }

    /**
     * @brief 动画更新回调
     */
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) override {
        // 调用AnimationPathCallback来执行原来的动画路径逻辑
        if (animationCallback_) {
            (*animationCallback_)(node, nv);
        }

        // 获取当前的动画时间进度
        double currentTime = animationCallback_->getAnimationTime();
        double duration = animationCallback_->getAnimationPath()->getPeriod();

        // 计算进度百分比，确保 t 始终在 0.0 到 1.0 之间
        float t = fmod(static_cast<float>(currentTime / duration), 1.0f);

        // 根据动画进度更新颜色的 alpha 值
        for (size_t i = 0; i < colors_->size(); ++i) {
            if (t <= 0.1f) {
                // 在 0.0 到 0.1 的范围内，alpha 值从 0 逐渐增加到 1
                (*colors_)[i].a() = t / 0.1f;
            } else if (t >= 0.9995f) {
                // 在 0.9995 到 1.0 的范围内，alpha 值从 1 逐渐减少到 0
                (*colors_)[i].a() = (1.0f - t) / 0.1f;
            } else {
                // 在 0.1 到 0.9995 的范围内，alpha 值保持为 1
                (*colors_)[i].a() = 1.0f;
            }
        }

        // 标记颜色数组已修改
        colors_->dirty();
        geometry_->setColorArray(colors_, osg::Array::BIND_PER_VERTEX);
        geometry_->dirtyDisplayList();
        geometry_->dirtyBound();

        // 调用父类的traverse方法
        traverse(node, nv);
    }

private:
    osg::ref_ptr<osg::Vec4Array> colors_;
    osg::ref_ptr<osg::Geometry> geometry_;
    osg::ref_ptr<osg::AnimationPathCallback> animationCallback_;
};

// ============================================================================
// 纹理动画回调
// ============================================================================

/**
 * @brief 纹理动画回调，用于纹理高光动画效果
 */
class TextureBasedAnimationCallback : public osg::NodeCallback {
public:
    /**
     * @brief 构造函数
     * @param lineDataImage 线条数据纹理图像
     * @param lines 边数据（需要Edge定义，这里用void*临时替代）
     */
    TextureBasedAnimationCallback(osg::Image* lineDataImage,
                                 std::shared_ptr<void> lines)
        : lineDataImage_(lineDataImage)
        , lines_(lines)
        , firstFrame_(true) {
        
        // 预分配缓存 - 需要知道边的数量
        if (lines_) {
            auto* edges = static_cast<std::vector<GraphRenderer::Edge>*>(lines_.get());
            paramCache_.resize(edges->size() * 4);
        }
    }

    /**
     * @brief 动画更新回调
     */
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) override {
        static double lastTime = nv->getFrameStamp()->getSimulationTime();

        double currentTime = nv->getFrameStamp()->getSimulationTime();
        
        // 首次运行初始化时间
        if (firstFrame_) {
            lastTime = currentTime;
            firstFrame_ = false;
            return;
        }
        
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        if (!lines_) {
            traverse(node, nv);
            return;
        }

        // 获取实际的边数据
        auto* edges = static_cast<std::vector<GraphRenderer::Edge>*>(lines_.get());

        // 更新本地缓存
        for (size_t i = 0; i < edges->size(); ++i) {
            // 更新高光位置
            (*edges)[i].highlightPos += (*edges)[i].speed * deltaTime;
            if ((*edges)[i].highlightPos >= 1.0f) {
                (*edges)[i].highlightPos = 0.0f;
            }

            int baseIdx = i * 4;
            paramCache_[baseIdx] = (*edges)[i].highlightPos;
            paramCache_[baseIdx + 1] = (*edges)[i].speed;
            paramCache_[baseIdx + 2] = 0.0f;
            paramCache_[baseIdx + 3] = 0.0f;
        }

        // 更新纹理
        if (lineDataImage_.valid()) {
            float* data = reinterpret_cast<float*>(lineDataImage_->data());
            if (data) {
                for (size_t i = 0; i < edges->size(); ++i) {
                    int dstPos = i * 4;
                    int srcPos = i * 4;
                    data[dstPos] = paramCache_[srcPos];
                    data[dstPos + 1] = paramCache_[srcPos + 1];
                    data[dstPos + 2] = paramCache_[srcPos + 2];
                    data[dstPos + 3] = paramCache_[srcPos + 3];
                }
                lineDataImage_->dirty();
            }
        }

        traverse(node, nv);
    }

    // 设置实际的边数据
    void setEdges(std::shared_ptr<void> lines) {
        lines_ = lines;
        if (lines_) {
            auto* edges = static_cast<std::vector<GraphRenderer::Edge>*>(lines_.get());
            paramCache_.resize(edges->size() * 4);
        }
    }

private:
    osg::ref_ptr<osg::Image> lineDataImage_;
    std::shared_ptr<void> lines_; // 实际类型为 std::shared_ptr<std::vector<GraphRenderer::Edge>>
    std::vector<float> paramCache_;
    bool firstFrame_;
};

// ============================================================================
// 颜色流动动画回调
// ============================================================================

/**
 * @brief 颜色流动动画回调，用于颜色渐变动画效果
 */
class TextureBasedAnimationColorCallback : public osg::NodeCallback {
public:
    /**
     * @brief 构造函数
     * @param lineDataImage 线条数据纹理图像
     * @param lines 边数据
     */
    TextureBasedAnimationColorCallback(osg::Image* lineDataImage,
                                      std::shared_ptr<void> lines)
        : lineDataImage_(lineDataImage)
        , lines_(lines)
        , firstFrame_(true) {
        
        if (lines_) {
            auto* edges = static_cast<std::vector<GraphRenderer::Edge>*>(lines_.get());
            paramCache_.resize(edges->size() * 4);
        }
    }

    /**
     * @brief 动画更新回调
     */
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) override {
        static double lastTime = nv->getFrameStamp()->getSimulationTime();

        double currentTime = nv->getFrameStamp()->getSimulationTime();
        
        if (firstFrame_) {
            lastTime = currentTime;
            firstFrame_ = false;
            return;
        }
        
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        if (!lines_) {
            traverse(node, nv);
            return;
        }

        auto* edges = static_cast<std::vector<GraphRenderer::Edge>*>(lines_.get());

        // 更新本地缓存
        for (size_t i = 0; i < edges->size(); ++i) {
            // 独立更新每条线的高光位置
            (*edges)[i].highlightPos = 
                fmod((*edges)[i].highlightPos + (*edges)[i].speed * deltaTime, 1.0f);

            int baseIdx = i * 4;
            paramCache_[baseIdx] = (*edges)[i].highlightPos;
            paramCache_[baseIdx + 1] = (*edges)[i].speed;
            paramCache_[baseIdx + 2] = 0.0f;
            paramCache_[baseIdx + 3] = 0.0f;
        }

        // 更新纹理
        if (lineDataImage_.valid()) {
            float* data = reinterpret_cast<float*>(lineDataImage_->data());
            if (data) {
                for (size_t i = 0; i < edges->size(); ++i) {
                    int dstPos = i * 4;
                    int srcPos = i * 4;
                    data[dstPos] = paramCache_[srcPos];
                    data[dstPos + 1] = paramCache_[srcPos + 1];
                    data[dstPos + 2] = paramCache_[srcPos + 2];
                    data[dstPos + 3] = paramCache_[srcPos + 3];
                }
                lineDataImage_->dirty();
            }
        }

        traverse(node, nv);
    }

    // 设置实际的边数据
    void setEdges(std::shared_ptr<void> lines) {
        lines_ = lines;
        if (lines_) {
            auto* edges = static_cast<std::vector<GraphRenderer::Edge>*>(lines_.get());
            paramCache_.resize(edges->size() * 4);
        }
    }

private:
    osg::ref_ptr<osg::Image> lineDataImage_;
    std::shared_ptr<void> lines_; // 实际类型为 std::shared_ptr<std::vector<GraphRenderer::Edge>>
    std::vector<float> paramCache_;
    bool firstFrame_;
};

// ============================================================================
// Shader 程序创建函数
// ============================================================================

/**
 * @brief 创建基础纹理动画着色器程序（高光动画）
 * @param lineCount 线条数量
 * @return 着色器程序
 */
inline osg::Program* createTextureBasedShaderProgram(int lineCount) {
    std::string vertSource = R"(
#version 120
attribute vec3 vertexPosition;
attribute float lineID;

varying vec3 vPosition;
varying float vLineID;
varying vec3 vLineStart;
varying vec3 vLineEnd;

uniform sampler2D uLineDataTex;
uniform float uTotalLines;

void main() {
    vPosition = vertexPosition;
    vLineID = lineID;
    
    // 从纹理获取当前线段的起点终点
    float texX = (lineID + 0.5) / uTotalLines;
    vLineStart = texture2D(uLineDataTex, vec2(texX, 0.25)).rgb;
    vLineEnd = texture2D(uLineDataTex, vec2(texX, 0.5)).rgb;
    
    gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexPosition, 1.0);
}
)";

    std::string fragSource = R"(
#version 120
uniform sampler2D uLineDataTex;
uniform float uTotalLines;
uniform float uHighlightWidth;
uniform vec4 uHighlightColor;
uniform float uGlowIntensity;    // 发光强度
uniform float uGlobalAlpha;      // 全局透明度
uniform float uLineThickness;    // 线条粗细

varying vec3 vPosition;
varying float vLineID;
varying vec3 vLineStart;
varying vec3 vLineEnd;

void main() {
    // 获取当前线段的动画参数
    float texX = (vLineID + 0.5) / uTotalLines;
    float highlightPos = texture2D(uLineDataTex, vec2(texX, 0.0)).r;
    
    // 计算当前点在线段上的投影位置 [0,1]
    vec3 lineVec = vLineEnd - vLineStart;
    float lineLength = length(lineVec);
    vec3 lineDir = lineVec / lineLength;
    float t = dot(vPosition - vLineStart, lineDir) / lineLength;
    t = clamp(t, 0.0, 1.0);
    
    // 计算到线段中心的距离（用于发光效果）
    vec3 projectedPos = vLineStart + t * lineVec;
    float centerDistance = length(vPosition - projectedPos);
    float glowRadius = uLineThickness * 2.0;
    float glowFactor = 1.0 - smoothstep(0.0, glowRadius, centerDistance);
    glowFactor = pow(glowFactor, 2.0); // 增强发光衰减
    
    // 计算高光效果（使用更窄的高光区域和更强的亮度）
    float dist = abs(t - highlightPos);
    float highlightWidth = uHighlightWidth * 0.5; // 减小高光宽度，使其更集中
    float highlightIntensity = 1.0 - smoothstep(0.0, highlightWidth, dist);
    highlightIntensity = pow(highlightIntensity, 0.5); // 使高光更亮
    
    // 基础颜色（深蓝色）(0.8f, 0.6f, 0.2f, 1.0f)
    vec4 baseColor = vec4(0.85f, 0.5f, 0.12f, uGlobalAlpha);
    
    // 发光效果
    vec3 glowColor = baseColor.rgb * uGlowIntensity * glowFactor;
    vec3 finalColor = baseColor.rgb + glowColor;
    
    // 高光颜色（明亮的白色）
    vec3 highlightColorRGB = vec3(1.0, 1.0, 1.0) * 1.2; // 增强高光亮度
    
    // 混合高光（使用更强的混合比例）
    finalColor = mix(finalColor, highlightColorRGB, highlightIntensity);
    
    // 最终透明度计算（增加高光处的透明度）
    float finalAlpha = baseColor.a * (1.0 + glowFactor * 0.5 + highlightIntensity * 0.5);
    finalAlpha = clamp(finalAlpha, 0.0, 1.0);
    
    gl_FragColor = vec4(finalColor, finalAlpha);
}
)";

    osg::ref_ptr<osg::Program> program = new osg::Program;
    program->addBindAttribLocation("vertexPosition", 2);
    program->addBindAttribLocation("lineID", 1);
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
    return program.release();
}

/**
 * @brief 创建颜色流动着色器程序
 * @param lineCount 线条数量
 * @return 着色器程序
 */
inline osg::Program* createTextureBasedShaderProgramColorFlow(int lineCount) {
    std::string vertSource = R"(
#version 120
attribute vec3 vertexPosition;
attribute float lineID;

varying vec3 vPosition;
varying float vLineID;
varying vec3 vLineStart;
varying vec3 vLineEnd;

uniform sampler2D uLineDataTex;
uniform float uTotalLines;
varying vec4 vColor;

void main() {
    vPosition = vertexPosition;
    vLineID = lineID;
    
    // 从纹理获取当前线段的起点终点
    float texX = (lineID + 0.5) / uTotalLines;
    vLineStart = texture2D(uLineDataTex, vec2(texX, 0.25)).rgb;
    vLineEnd = texture2D(uLineDataTex, vec2(texX, 0.5)).rgb;
    vColor = gl_Color;
    gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexPosition, 1.0);
}
)";

    std::string fragSource = R"(
#version 120
uniform sampler2D uLineDataTex;
uniform float uTotalLines;

varying vec3 vPosition;
varying float vLineID;
varying vec3 vLineStart;
varying vec3 vLineEnd;

// 定义颜色（蓝色和黄色）
const vec3 blue = vec3(0.0, 0.0, 0.0);
const vec3 yellow = vec3(1.0, 1.0, 0.0);

void main() {
    // 获取当前线段的动画相位 [0, 1]
    float texX = (vLineID + 0.5) / uTotalLines;
    float phase = texture2D(uLineDataTex, vec2(texX, 0.0)).r;
    
    // 计算当前点在线段上的投影位置 t [0, 1]
    vec3 lineVec = vLineEnd - vLineStart;
    float lineLength = length(lineVec);
    vec3 lineDir = lineVec / lineLength;
    float t = dot(vPosition - vLineStart, lineDir) / lineLength;
    t = clamp(t, 0.0, 1);

    // 关键改进：计算颜色权重（使用 cos 实现平滑循环）
    float colorWeight = 0.5 + 0.5 * cos(2.0 * 3.1415926 * (t - phase));
    
    // 混合颜色（蓝色 ↔ 黄色 ↔ 蓝色...）
    vec3 color = mix(blue, yellow, colorWeight);
    
    // 输出颜色（固定透明度 1.0）
    gl_FragColor = vec4(color, 0.2);
}
)";

    osg::ref_ptr<osg::Program> program = new osg::Program;
    // 必须显式绑定属性位置
    program->addBindAttribLocation("vertexPosition", 2);
    program->addBindAttribLocation("lineID", 1);
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
    return program.release();
}

/**
 * @brief 创建星形流动着色器程序
 * @param lineCount 线条数量
 * @return 着色器程序
 */
inline osg::Program* createTextureBasedShaderProgramStarFlow(int lineCount) {
    std::string vertSource = R"(
#version 120
attribute vec3 vertexPosition;
attribute float lineID;

varying vec3 vPosition;
varying float vLineID;
varying vec3 vLineStart;
varying vec3 vLineEnd;

uniform sampler2D uLineDataTex;
uniform float uTotalLines;
varying vec4 vColor;

void main() {
    vPosition = vertexPosition;
    vLineID = lineID;
    
    // 从纹理获取当前线段的起点终点
    float texX = (lineID + 0.5) / uTotalLines;
    vLineStart = texture2D(uLineDataTex, vec2(texX, 0.25)).rgb;
    vLineEnd = texture2D(uLineDataTex, vec2(texX, 0.5)).rgb;
    vColor = gl_Color;
    gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexPosition, 1.0);
}
)";

    std::string fragSource = R"(
#version 120
uniform sampler2D uLineDataTex;
uniform float uTotalLines;
uniform float uHighlightWidth;
uniform vec4 uHighlightColor;

varying vec3 vPosition;
varying float vLineID;
varying vec3 vLineStart;
varying vec3 vLineEnd;
varying vec4 vColor;

void main() {
    
    // 获取当前线段的高光位置
    float texX = (vLineID + 0.5) / uTotalLines;
    float highlightPos = texture2D(uLineDataTex, vec2(texX, 0.0)).r;
    
    // 计算线段方向和长度
    vec3 lineVec = vLineEnd - vLineStart;
    float lineLength = length(lineVec);
    vec3 lineDir = lineVec / lineLength;
    
    // 计算当前点在直线上的投影
    float t = dot(vPosition - vLineStart, lineDir) / lineLength;
    t = clamp(t, 0.0, 1.0);
    
    // 计算到线段的真实距离（用于线宽控制）
    vec3 projectedPos = vLineStart + t * lineVec;
    float dist = length(vPosition - projectedPos);
    float uHighlightl = 0.05;
    
    // 高光强度计算（仅在前向移动方向增强）
    float highlightIntensity = 0.0;
    if(t >= highlightPos&& t <= highlightPos + 0.05) {
        float posInHighlight = (t - highlightPos) / uHighlightl;  // 0=前端, 1=尾端
        highlightIntensity = smoothstep(0.0, 1.0, posInHighlight);  // 线性增强（尾端最亮）
    }
    
    // 基础颜色
    vec4 baseColor = vec4(0.85f, 0.5f, 0.12f, 0.2f);
    // 最终颜色
    gl_FragColor =mix(baseColor, uHighlightColor, highlightIntensity*1);
}
)";

    osg::ref_ptr<osg::Program> program = new osg::Program;
    // 必须显式绑定属性位置
    program->addBindAttribLocation("vertexPosition", 2);
    program->addBindAttribLocation("lineID", 1);
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
    return program.release();
}

// ============================================================================
// 动画管理器（未来扩展）
// ============================================================================

/**
 * @brief 动画管理器，统一管理各种动画效果
 * @note 这是为未来扩展预留的接口，当前版本动画功能仍在 PerGraphParam 中
 */
class AnimationManager {
public:
    AnimationManager() : isAnimating_(false) {}
    ~AnimationManager() {}

    // 设置数据
    void setEdges(std::shared_ptr<void> edges) { edges_ = edges; }
    void setNodes(std::shared_ptr<void> nodes) { nodes_ = nodes; }
    void setAnimationGroup(osg::Group* group) { animationGroup_ = group; }

    // 动画控制（预留接口）
    void startArrowAnimation() {
        std::cout << "AnimationManager::startArrowAnimation() - Not implemented yet" << std::endl;
    }
    
    void stopArrowAnimation() {
        std::cout << "AnimationManager::stopArrowAnimation() - Not implemented yet" << std::endl;
    }
    
    void startHighlightAnimation() {
        std::cout << "AnimationManager::startHighlightAnimation() - Not implemented yet" << std::endl;
    }
    
    void startTextureAnimation() {
        std::cout << "AnimationManager::startTextureAnimation() - Not implemented yet" << std::endl;
    }
    
    void startStarAnimation() {
        std::cout << "AnimationManager::startStarAnimation() - Not implemented yet" << std::endl;
    }
    
    void startTextureFlowAnimation() {
        std::cout << "AnimationManager::startTextureFlowAnimation() - Not implemented yet" << std::endl;
    }

    // 创建动画
    void createArrowAnimation(const osg::Vec3& start, const osg::Vec3& end,
                             const osg::Vec4& color, int startIndex, int endIndex) {
        std::cout << "AnimationManager::createArrowAnimation() - Not implemented yet" << std::endl;
    }
    
    // 创建纹理数据
    osg::Image* createLineDataTexture() {
        std::cout << "AnimationManager::createLineDataTexture() - Not implemented yet" << std::endl;
        return nullptr;
    }

private:
    std::shared_ptr<void> edges_;
    std::shared_ptr<void> nodes_;
    osg::ref_ptr<osg::Group> animationGroup_;
    bool isAnimating_;
};

} // namespace GraphAnimation
} // namespace VIS4Earth

#endif // VIS4EARTH_GRAPH_VISER_GRAPH_ANIMATION_H
