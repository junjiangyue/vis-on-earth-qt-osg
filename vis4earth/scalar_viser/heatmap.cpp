#include <vis4earth/scalar_viser/heatmap.h>

#include <array>

#include <ui_heatmap.h>
#include <vis4earth/components_ui_export.h>

VIS4Earth::HeatmapRenderer::HeatmapRenderer(QWidget *parent)
    : volCmpt(true), QtOSGReflectableWidget(ui, parent) {
    ui->scrollAreaWidgetContents_main->layout()->addWidget(&geoCmpt);
    ui->scrollAreaWidgetContents_main->layout()->addWidget(&volCmpt);

    initOSGResource();

    auto onResUpdated = [&]() { updateGeometry(); };
    connect(ui->spinBox_resX, &QSpinBox::editingFinished, onResUpdated);
    connect(ui->spinBox_resY, &QSpinBox::editingFinished, onResUpdated);

    auto genHeatmapTex = [&]() {
        auto vol = volCmpt.GetVolume(0, 0);
        if (!vol)
            return;

        auto img3D = vol->getImage();
        volSliceImg->allocateImage(img3D->s(), img3D->t(), 1, GL_RED, GL_UNSIGNED_BYTE);
        volSliceImg->setInternalTextureFormat(GL_RED);
        auto res = static_cast<size_t>(img3D->s()) * img3D->t();
        std::memcpy(volSliceImg->data(),
                    img3D->data() + ui->spinBox_height_int_VIS4EarthReflectable->value() * res,
                    res);

        auto filterMode = [&]() {
            auto mode = static_cast<ETextureFilterMode>(ui->comboBox_texFilterMode->currentIndex());
            if (mode == ETextureFilterMode::Nearest)
                return osg::Texture::FilterMode::NEAREST;
            return osg::Texture::FilterMode::LINEAR;
        }();
        volSliceTex->setFilter(osg::Texture::MAG_FILTER, filterMode);
        volSliceTex->setFilter(osg::Texture::MIN_FILTER, filterMode);
        volSliceTex->setWrap(osg::Texture::WRAP_S, osg::Texture::WrapMode::CLAMP_TO_EDGE);
        volSliceTex->setWrap(osg::Texture::WRAP_T, osg::Texture::WrapMode::CLAMP_TO_EDGE);
        volSliceTex->setWrap(osg::Texture::WRAP_R, osg::Texture::WrapMode::CLAMP_TO_EDGE);
        volSliceTex->setInternalFormatMode(osg::Texture::InternalFormatMode::USE_IMAGE_DATA_FORMAT);
        volSliceTex->setImage(volSliceImg);
    };
    connect(&volCmpt, &VolumeComponent::VolumeChanged, [&, genHeatmapTex]() {
        auto vol = volCmpt.GetVolume(0, 0);
        if (!vol)
            return;

        auto h = vol->getImage()->r();
        ui->spinBox_height_int_VIS4EarthReflectable->setMaximum(h - 1);
        volHeight->set(h);

        genHeatmapTex();
    });
    connect(ui->spinBox_height_int_VIS4EarthReflectable,
            QOverload<int>::of(&QSpinBox::valueChanged), [&, genHeatmapTex]() { genHeatmapTex(); });
    connect(ui->comboBox_texFilterMode, QOverload<int>::of(&QComboBox::currentIndexChanged),
            genHeatmapTex);

    auto changeTF = [&]() {
        auto stateSet = geode->getOrCreateStateSet();
        stateSet->setTextureAttributeAndModes(0, volCmpt.GetTransferFunction(0),
                                              osg::StateAttribute::ON);
    };
    connect(&volCmpt, &VolumeComponent::TransferFunctionChanged, changeTF);
    changeTF();

    connect(this, &HeatmapRenderer::updateHeatmap2DSignal, this, &HeatmapRenderer::updateHeatmap2D);

    updateGeometry();

    debugProperties({this, &volCmpt, &geoCmpt});
}

void VIS4Earth::HeatmapRenderer::initOSGResource() {
    grp = new osg::Group();
    geom = new osg::Geometry();
    geode = new osg::Geode();
    verts = new osg::Vec3Array();
    volSliceTex = new osg::Texture2D();
    volSliceImg = new osg::Image();
    program = new osg::Program();

    auto stateSet = geode->getOrCreateStateSet();
    volHeight = new osg::Uniform("volHeight", 0);
    stateSet->addUniform(volHeight);
    for (auto obj : std::array<QtOSGReflectableWidget *, 3>{this, &geoCmpt, &volCmpt})
        obj->ForEachProperty([&](const std::string &name, const Property &prop) {
            stateSet->addUniform(prop.GetUniform());
        });
    {
        osg::ref_ptr<osg::Shader> vertShader = osg::Shader::readShaderFile(
            osg::Shader::VERTEX,
            GetDataPathPrefix() + VIS4EARTH_SHADER_PREFIX "scalar_viser/heatmap_vert.glsl");
        osg::ref_ptr<osg::Shader> fragShader = osg::Shader::readShaderFile(
            osg::Shader::FRAGMENT,
            GetDataPathPrefix() + VIS4EARTH_SHADER_PREFIX "scalar_viser/heatmap_frag.glsl");
        program->addShader(vertShader);
        program->addShader(fragShader);
    }
    {
        auto tfTexUni = new osg::Uniform(osg::Uniform::SAMPLER_1D, "tfTex");
        tfTexUni->set(0);
        stateSet->addUniform(tfTexUni);

        tfTexUni = new osg::Uniform(osg::Uniform::SAMPLER_2D, "volSliceTex");
        tfTexUni->set(1);
        stateSet->addUniform(tfTexUni);

        stateSet->setTextureAttributeAndModes(1, volSliceTex, osg::StateAttribute::ON);
    }
    stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    stateSet->setAttributeAndModes(program, osg::StateAttribute::ON);

    geom->setUseDisplayList(false);
    geom->setDrawCallback(new Heatmap2DDrawCallback(this, ui, volSliceTex, volCmpt, heatmap2D));
    geode->addDrawable(geom);
    grp->addChild(geode);
}

void VIS4Earth::HeatmapRenderer::updateHeatmap2D() {
    auto pixmap = QPixmap::fromImage(heatmap2D);
    if (!ui->graphicsView_2DView->scene())
        ui->graphicsView_2DView->setScene(new QGraphicsScene);
    auto scn = ui->graphicsView_2DView->scene();
    scn->clear();
    scn->addPixmap(pixmap);

    ui->graphicsView_2DView->fitInView(scn->itemsBoundingRect(), Qt::KeepAspectRatio);
    update();
}

void VIS4Earth::HeatmapRenderer::updateGeometry() {
    verts->clear();

    auto res = ui->spinBox_resX->value() * ui->spinBox_resY->value();
    verts->reserve(res);
    for (int latIdx = 0; latIdx <= ui->spinBox_resY->value(); ++latIdx)
        for (int lonIdx = 0; lonIdx <= ui->spinBox_resX->value(); ++lonIdx) {
            osg::Vec3 pos(1.f * lonIdx / ui->spinBox_resX->value(),
                          1.f * latIdx / ui->spinBox_resY->value(), 0.f);

            verts->push_back(pos);
        }

    std::vector<GLuint> vertIndices;
    auto addTri = [&](std::array<GLuint, 3> triIndices) {
        for (uint8_t i = 0; i < 3; ++i)
            vertIndices.emplace_back(triIndices[i]);
    };
    auto addBotSurf = [&](int latIdx, int lonIdx) {
        std::array<GLuint, 4> quadIndices = {latIdx * ui->spinBox_resX->value() + lonIdx,
                                             latIdx * ui->spinBox_resX->value() + lonIdx + 1,
                                             (latIdx + 1) * ui->spinBox_resX->value() + lonIdx + 1,
                                             (latIdx + 1) * ui->spinBox_resX->value() + lonIdx};

        addTri({quadIndices[0], quadIndices[1], quadIndices[2]});
        addTri({quadIndices[2], quadIndices[3], quadIndices[0]});
    };
    for (int latIdx = 0; latIdx < ui->spinBox_resY->value(); ++latIdx)
        for (int lonIdx = 0; lonIdx < ui->spinBox_resX->value(); ++lonIdx)
            addBotSurf(latIdx, lonIdx);

    geom->setInitialBound([]() -> osg::BoundingBox {
        osg::Vec3 max(osg::WGS_84_RADIUS_POLAR, osg::WGS_84_RADIUS_POLAR, osg::WGS_84_RADIUS_POLAR);
        return osg::BoundingBox(-max, max);
    }()); // 必须，否则不显示

    geom->setVertexArray(verts);
    geom->getPrimitiveSetList().clear();
    geom->addPrimitiveSet(
        new osg::DrawElementsUInt(GL_TRIANGLES, vertIndices.size(), vertIndices.data()));
}

VIS4Earth::Heatmap2DDrawCallback::Heatmap2DDrawCallback(HeatmapRenderer *heatmapRenderer,
                                                        Ui::HeatmapRenderer *&ui,
                                                        osg::ref_ptr<osg::Texture2D> &volSliceTex,
                                                        VolumeComponent &volCmpt, QImage &heatmap2D)
    : heatmapRenderer(heatmapRenderer), ui(ui), volSliceTex(volSliceTex), volCmpt(volCmpt),
      heatmap2D(heatmap2D) {
    computeShader = new ComputeShader(
        (GetDataPathPrefix() + VIS4EARTH_SHADER_PREFIX "scalar_viser/heatmap_compute.glsl")
            .c_str());
}

void VIS4Earth::Heatmap2DDrawCallback::drawImplementation(osg::RenderInfo &renderInfo,
                                                          const osg::Drawable *drawable) const {
    auto state = renderInfo.getState();
    drawable->drawImplementation(renderInfo);

    if (heatmap2D.width() != ui->spinBox_resX->value() ||
        heatmap2D.height() != ui->spinBox_resY->value())
        heatmap2D =
            QImage(ui->spinBox_resX->value(), ui->spinBox_resY->value(), QImage::Format_RGBA8888);

    if (!volCmpt.GetVolume(0, 0)) {
        heatmapRenderer->emitUpdateHeatmap2DSignal();
        return;
    }

    auto tf = volCmpt.GetTransferFunction(0);
    auto ext = renderInfo.getState()->get<osg::GLExtensions>();

    computeShader->compile(ext);
    computeShader->use(ext);

    // set uniforms
    GLuint volumeTex, tfTex;
    volumeTex = volSliceTex->getTextureObject(state->getContextID())->id();
    ext->glActiveTexture(GL_TEXTURE0 + 1);
    glBindTexture(GL_TEXTURE_2D, volumeTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, volSliceTex->getImage()->s(),
                 volSliceTex->getImage()->t(), 0, GL_RED, GL_UNSIGNED_BYTE,
                 volSliceTex->getImage()->data());
    computeShader->setInt(ext, "volume", 1);

    tfTex = tf->getTextureObject(state->getContextID())->id();
    ext->glActiveTexture(GL_TEXTURE0 + 2);
    glBindTexture(GL_TEXTURE_1D, tfTex);
    glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA, tf->getImage()->s(), 0, GL_RGBA, GL_FLOAT,
                 tf->getImage()->data());
    computeShader->setInt(ext, "transferFunction", 2);

    // set output image
    GLuint heatmapTex;
    glGenTextures(1, &heatmapTex);
    ext->glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, heatmapTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, heatmap2D.width(), heatmap2D.height(), 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, nullptr);
    ext->glBindImageTexture(0, heatmapTex, 0, GL_FALSE, 0, GL_WRITE_ONLY_ARB, GL_RGBA8UI_EXT);

    // compute
    ext->glDispatchCompute(heatmap2D.width(), heatmap2D.height(), 1);
    ext->glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, heatmap2D.bits());

    glDeleteTextures(1, &heatmapTex);
    heatmapRenderer->emitUpdateHeatmap2DSignal();
}