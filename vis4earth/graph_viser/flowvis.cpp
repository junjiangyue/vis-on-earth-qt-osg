
#include <QDebug>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>

#include <vis4earth/graph_viser/flowvis.h>

namespace VIS4Earth {

class ArrowFlowWidget : public QOpenGLWidget, protected QOpenGLFunctions {
  public:
    ArrowFlowWidget(QWidget *parent = nullptr)
        : QOpenGLWidget(parent), vbo(QOpenGLBuffer::VertexBuffer) {}
    ~ArrowFlowWidget() { cleanup(); }

  protected:
    void initializeGL() override {
        initializeOpenGLFunctions();
        glClearColor(0, 0, 0, 1);

        // Initialize shaders
        if (!program.addShaderFromSourceFile(QOpenGLShader::Vertex,
                                             ":/shaders/vertex_shader.glsl") ||
            !program.addShaderFromSourceFile(QOpenGLShader::Fragment,
                                             ":/shaders/fragment_shader.glsl") ||
            !program.link()) {
            qDebug() << "Shader Program Link Error";
            return;
        }

        // Initialize VBO
        vbo.create();
        vbo.bind();
        float vertices[] = {// Positions of the arrow vertices
                            -0.05f, -0.05f, 0.0f, 0.05f, -0.05f, 0.0f, 0.0f, 0.1f, 0.0f};
        vbo.allocate(vertices, sizeof(vertices));
        vbo.release();
    }

    void resizeGL(int w, int h) override { glViewport(0, 0, w, h); }

    void paintGL() override {
        glClear(GL_COLOR_BUFFER_BIT);

        program.bind();
        vbo.bind();

        int posAttr = program.attributeLocation("position");
        program.enableAttributeArray(posAttr);
        program.setAttributeBuffer(posAttr, GL_FLOAT, 0, 3);

        glDrawArrays(GL_TRIANGLES, 0, 3);

        program.disableAttributeArray(posAttr);
        vbo.release();
        program.release();
    }

  private:
    void cleanup() { vbo.destroy(); }

    QOpenGLShaderProgram program;
    QOpenGLBuffer vbo;
};

void FlowVisualization::drawArrowFlow() {
    // 这里是你的绘制箭头流动的代码
    qDebug() << "Drawing arrow flow...";
    // 实现绘制箭头流动的逻辑，例如使用 OpenGL 绘制箭头
}

void FlowVisualization::setGraph(VIS4Earth::Graph grph) {}

void FlowVisualization::getGraph() {}

} // namespace VIS4Earth