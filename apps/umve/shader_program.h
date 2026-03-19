/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef OGL_SHADER_PROGRAM_HEADER
#define OGL_SHADER_PROGRAM_HEADER

#include <string>
#include <memory>

#include "math/matrix.h"
#include "ogl_common.h"

#define OGL_ATTRIB_POSITION "pos"
#define OGL_ATTRIB_NORMAL "normal"
#define OGL_ATTRIB_COLOR "color"
#define OGL_ATTRIB_TEXCOORD "texuv"

OGL_NAMESPACE_BEGIN

class ShaderProgram
{
public:
    typedef std::shared_ptr<ShaderProgram> Ptr;

public:
    ~ShaderProgram (void);
    static Ptr create (void);

    /** Loads a vertex shader from code in memory. */
    void load_vert_code (std::string const& code);
    /** Load fragment shader from code in memory. */
    void load_frag_code (std::string const& code);

    /** Returns attribute location for the program. */
    GLint get_attrib_location (char const* name);
    /** Returns the uniform location of the program. */
    GLint get_uniform_location (char const* name);

    /** Sends 4x4-matrix 'm' to uniform location 'name'. */
    void send_uniform (char const* name, math::Matrix4f const& m);

    /** Selects the shader program for rendering. */
    void bind (void);
    /** Deselects the shader program. */
    void unbind (void) const;

private:
    ShaderProgram (void);

    void load_shader_code (GLuint& shader_id, GLuint shader_type,
        std::string const& code);
    void compile_shader (GLuint shader_id, std::string const& code);

    GLint get_program_property (int pname);
    GLint get_shader_property (GLuint shader_id, int pname);

    void ensure_linked (void);

private:
    GLuint prog_id;
    GLuint vert_id;
    GLuint frag_id;

    bool need_to_link;
};

/* ---------------------------------------------------------------- */

inline
ShaderProgram::ShaderProgram (void)
{
    this->vert_id = 0;
    this->frag_id = 0;
    this->prog_id = glCreateProgram();
    check_gl_error();
    this->need_to_link = false;
}

inline
ShaderProgram::~ShaderProgram (void)
{
    glDeleteProgram(this->prog_id);
    check_gl_error();
    glDeleteShader(this->vert_id);
    check_gl_error();
    glDeleteShader(this->frag_id);
    check_gl_error();
}

inline ShaderProgram::Ptr
ShaderProgram::create (void)
{
    return Ptr(new ShaderProgram);
}

inline void
ShaderProgram::load_vert_code (std::string const& code)
{
    this->load_shader_code(this->vert_id, GL_VERTEX_SHADER, code);
}

inline void
ShaderProgram::load_frag_code (std::string const& code)
{
    this->load_shader_code(this->frag_id, GL_FRAGMENT_SHADER, code);
}

inline GLint
ShaderProgram::get_attrib_location (char const* name)
{
    this->ensure_linked();
    return glGetAttribLocation(this->prog_id, name);
}

inline GLint
ShaderProgram::get_uniform_location (char const* name)
{
    this->ensure_linked();
    return glGetUniformLocation(this->prog_id, name);
}

inline void
ShaderProgram::send_uniform (const char* name, math::Matrix4f const& m)
{
    GLint loc = this->get_uniform_location(name);
    if (loc < 0)
        return;
    glUniformMatrix4fv(loc, 1, true, *m);
}

inline void
ShaderProgram::bind (void)
{
    this->ensure_linked();
    glUseProgram(this->prog_id);
    check_gl_error();
}

inline void
ShaderProgram::unbind (void) const
{
    glUseProgram(0);
    check_gl_error();
}

inline GLint
ShaderProgram::get_program_property (int pname)
{
    GLint ret;
    glGetProgramiv(this->prog_id, pname, &ret);
    check_gl_error();
    return ret;
}

inline GLint
ShaderProgram::get_shader_property (GLuint shader_id, int pname)
{
    GLint ret;
    glGetShaderiv(shader_id, pname, &ret);
    check_gl_error();
    return ret;
}

OGL_NAMESPACE_END

#endif /* OGL_SHADER_PROGRAM_HEADER */
