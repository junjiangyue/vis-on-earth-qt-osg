#ifndef VIS_ON_EARTH_OSG_QT_COMPUTE_SHADER_H
#define VIS_ON_EARTH_OSG_QT_COMPUTE_SHADER_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <gl/gl.h>
#include <gl/glu.h>
#include <osg/GLDefines>
#include <osg/GLExtensions>
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glu32.lib")

class ComputeShader {
  public:
    unsigned int ID;
    ComputeShader(const char *computePath) {
        std::string computeCode;
        std::ifstream cShaderFile;
        // ensure ifstream objects can throw exceptions:
        cShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        try {
            cShaderFile.open(computePath);
            std::stringstream cShaderStream;
            cShaderStream << cShaderFile.rdbuf();
            cShaderFile.close();
            computeCode = cShaderStream.str();
        } catch (std::ifstream::failure &e) {
            std::cout << "ERROR::SHADER::FILE_NOT_SUCCESSFULLY_READ: " << e.what() << std::endl;
        }
        cShaderCode = strdup(computeCode.c_str());
    }

    // activate the shader
    void compile(osg::GLExtensions *ext) {
        // compile shaders
        unsigned int compute;
        // compute shader
        compute = ext->glCreateShader(GL_COMPUTE_SHADER);
        ext->glShaderSource(compute, 1, &cShaderCode, nullptr);
        ext->glCompileShader(compute);
        checkCompileErrors(ext, compute, "COMPUTE");

        // shader Program
        ID = ext->glCreateProgram();
        ext->glAttachShader(ID, compute);
        ext->glLinkProgram(ID);
        checkCompileErrors(ext, ID, "PROGRAM");
        // delete the shaders as they're linked into our program now and no longer necessary
        ext->glDeleteShader(compute);
    }
    void use(osg::GLExtensions *ext) const { ext->glUseProgram(ID); }

    // utility uniform functions
    void setBool(osg::GLExtensions *ext, const std::string &name, bool value) const {
        ext->glUniform1i(ext->glGetUniformLocation(ID, name.c_str()), (int)value);
    }
    void setInt(osg::GLExtensions *ext, const std::string &name, int value) const {
        ext->glUniform1i(ext->glGetUniformLocation(ID, name.c_str()), value);
    }
    void setFloat(osg::GLExtensions *ext, const std::string &name, float value) const {
        ext->glUniform1f(ext->glGetUniformLocation(ID, name.c_str()), value);
    }
    void setVec2(osg::GLExtensions *ext, const std::string &name, float x, float y) const {
        ext->glUniform2f(ext->glGetUniformLocation(ID, name.c_str()), x, y);
    }
    void setVec3(osg::GLExtensions *ext, const std::string &name, float x, float y, float z) const {
        ext->glUniform3f(ext->glGetUniformLocation(ID, name.c_str()), x, y, z);
    }
    void setVec4(osg::GLExtensions *ext, const std::string &name, float x, float y, float z,
                 float w) const {
        ext->glUniform4f(ext->glGetUniformLocation(ID, name.c_str()), x, y, z, w);
    }

  private:
    const char *cShaderCode;
    // utility function for checking shader compilation/linking errors.
    static void checkCompileErrors(osg::GLExtensions *ext, GLuint shader, std::string type) {
        GLint success;
        GLchar infoLog[1024];
        if (type != "PROGRAM") {
            ext->glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if (!success) {
                ext->glGetShaderInfoLog(shader, 1024, nullptr, infoLog);
                std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n"
                          << infoLog
                          << "\n -- --------------------------------------------------- -- "
                          << std::endl;
            }
        } else {
            ext->glGetProgramiv(shader, GL_LINK_STATUS, &success);
            if (!success) {
                ext->glGetProgramInfoLog(shader, 1024, nullptr, infoLog);
                std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n"
                          << infoLog
                          << "\n -- --------------------------------------------------- -- "
                          << std::endl;
            }
        }
    }
};

#endif // VIS_ON_EARTH_OSG_QT_COMPUTE_SHADER_H
