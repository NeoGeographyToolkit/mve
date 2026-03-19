/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <string>
#include <stdexcept>

#include "shader_program.h"

OGL_NAMESPACE_BEGIN

void
ShaderProgram::load_shader_code (GLuint& shader_id, GLuint shader_type,
    std::string const& code)
{
    if (shader_id == 0) {
        shader_id = glCreateShader(shader_type);
        check_gl_error();
        glAttachShader(this->prog_id, shader_id);
        check_gl_error();
    }

    this->compile_shader(shader_id, code);
    this->need_to_link = true;
}

void
ShaderProgram::compile_shader (GLuint shader_id, std::string const& code)
{
    char const* data[1] = { code.c_str() };
    glShaderSource(shader_id, 1, data, nullptr);
    check_gl_error();

    glCompileShader(shader_id);
    check_gl_error();
    if (this->get_shader_property(shader_id, GL_COMPILE_STATUS) == GL_FALSE) {
        GLint log_size = this->get_shader_property(shader_id, GL_INFO_LOG_LENGTH);
        if (log_size == 0)
            throw std::runtime_error("Shader compilation failed (no message).");

        std::string log;
        log.append(log_size + 1, '\0');
        glGetShaderInfoLog(shader_id, log_size + 1, nullptr, &log[0]);
        throw std::runtime_error(log);
    }
}

void
ShaderProgram::ensure_linked (void)
{
    if (this->need_to_link) {
        glLinkProgram(this->prog_id);
        check_gl_error();
        if (this->get_program_property(GL_LINK_STATUS) == GL_FALSE) {
            GLint log_size = this->get_program_property(GL_INFO_LOG_LENGTH);
            if (log_size == 0)
                throw std::runtime_error("Failed to link program (no message).");

            std::string log;
            log.append(log_size + 1, '\0');
            glGetProgramInfoLog(this->prog_id, log_size + 1, nullptr, &log[0]);
            throw std::runtime_error(log);
        }
        this->need_to_link = false;
    }
}

OGL_NAMESPACE_END
