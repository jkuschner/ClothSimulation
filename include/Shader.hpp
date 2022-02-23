//
//  Shader.hpp
//  hw1
//
//  Created by Baichuan Wu on 12/20/21.
//

#pragma once

#include "core.hpp"

class Shader {
private:
    GLuint vs;
    GLuint fs;
    GLint vscs = 0;
    GLint fscs = 0;
    GLint linked = 0;
    std::string vsf;
    std::string fsf;
    std::string vsfn;
    std::string fsfn;
    
    std::string read(const std::string& fn) {
        std::ifstream in(fn);
        if (in.fail()) {
            throw std::runtime_error("Failed to open " + fn);
        }
        std::stringstream buf;
        buf << in.rdbuf();
        return buf.str();
    }
    
    void getShaderCompileErrors(const GLuint shader) {
        GLint length;
        GLchar* log;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
        log = new GLchar[length + 1];
        glGetShaderInfoLog(shader, length, &length, log);
        std::cout << log <<std::endl;
        delete[] log;
    }
    
    void getShaderProgramErrors(const GLuint program) {
        GLint length;
        GLchar* log;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
        log = new GLchar[length + 1];
        glGetProgramInfoLog(program, length, &length, log);
        std::cout << log <<std::endl;
        delete[] log;
    }
    
public:
    std::string name;
    GLuint program;
    
    explicit Shader(const std::string& name, const std::string& vsfn, const std::string& fsfn) : name(name) {
        read(vsfn, fsfn);
        compile();
    }
    
    ~Shader() {
        glDeleteProgram(program);
    }
    
    void read (const std::string& vsfn, const std::string& fsfn) {
        this->vsfn = vsfn;
        this->fsfn = fsfn;
        vsf = read(vsfn);
        fsf = read(fsfn);
    }
    void compile() {
        vs = glCreateShader(GL_VERTEX_SHADER);
        fs = glCreateShader(GL_FRAGMENT_SHADER);
        const GLchar* vs_cstr = vsf.c_str();
        const GLchar* fs_cstr = fsf.c_str();
        glShaderSource(vs, 1, &vs_cstr, NULL);
        glShaderSource(fs, 1, &fs_cstr, NULL);
        glCompileShader(vs);
        glCompileShader(fs);
        glGetShaderiv(vs, GL_COMPILE_STATUS, &vscs);
        glGetShaderiv(fs, GL_COMPILE_STATUS, &fscs);
        if (!vscs) {
            std::cout << "In " << vsfn << std::endl;
            getShaderCompileErrors(vs);
            throw std::runtime_error("Vertex shader compilation failed");
        }
        if (!fscs) {
            std::cout << "In " << fsfn << std::endl;
            getShaderCompileErrors(fs);
            throw std::runtime_error("Fragment shader compilation failed");
        }
        program = glCreateProgram();
        glAttachShader(program, vs);
        glAttachShader(program, fs);
        glLinkProgram(program);
        glGetProgramiv(program, GL_LINK_STATUS, &linked);
        if (linked) {
            glDetachShader(program, vs);
            glDetachShader(program, fs);
            glDeleteShader(vs);
            glDeleteShader(fs);
        } else {
            getShaderProgramErrors(program);
            throw std::runtime_error("Shader program linking failed");
        }
    }
    GLint getVertexShaderCompileStatus() { return vscs; }
    GLint getFragmentShaderCompileStatus() { return fscs; }
    GLint getLinkStatus() { return linked; }
};
