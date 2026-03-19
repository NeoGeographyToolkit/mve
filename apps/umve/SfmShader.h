// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).
/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef SFM_SHADER_HEADER
#define SFM_SHADER_HEADER

#include <string>
#include <memory>

#include "SfmMath.h"
#include "GlCommon.h"

#define SFM_ATTRIB_POSITION "pos"
#define SFM_ATTRIB_NORMAL "normal"
#define SFM_ATTRIB_COLOR "color"
#define SFM_ATTRIB_TEXCOORD "texuv"

GL_NAMESPACE_BEGIN

class SfmShader
{
public:
    typedef std::shared_ptr<SfmShader> Ptr;

public:
    ~SfmShader (void);
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
    SfmShader (void);

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
SfmShader::SfmShader (void)
{
    this->vert_id = 0;
    this->frag_id = 0;
    this->prog_id = glCreateProgram();
    check_gl_error();
    this->need_to_link = false;
}

inline
SfmShader::~SfmShader (void)
{
    glDeleteProgram(this->prog_id);
    check_gl_error();
    glDeleteShader(this->vert_id);
    check_gl_error();
    glDeleteShader(this->frag_id);
    check_gl_error();
}

inline SfmShader::Ptr
SfmShader::create (void)
{
    return Ptr(new SfmShader);
}

inline void
SfmShader::load_vert_code (std::string const& code)
{
    this->load_shader_code(this->vert_id, GL_VERTEX_SHADER, code);
}

inline void
SfmShader::load_frag_code (std::string const& code)
{
    this->load_shader_code(this->frag_id, GL_FRAGMENT_SHADER, code);
}

inline GLint
SfmShader::get_attrib_location (char const* name)
{
    this->ensure_linked();
    return glGetAttribLocation(this->prog_id, name);
}

inline GLint
SfmShader::get_uniform_location (char const* name)
{
    this->ensure_linked();
    return glGetUniformLocation(this->prog_id, name);
}

inline void
SfmShader::send_uniform (const char* name, math::Matrix4f const& m)
{
    GLint loc = this->get_uniform_location(name);
    if (loc < 0)
        return;
    glUniformMatrix4fv(loc, 1, true, *m);
}

inline void
SfmShader::bind (void)
{
    this->ensure_linked();
    glUseProgram(this->prog_id);
    check_gl_error();
}

inline void
SfmShader::unbind (void) const
{
    glUseProgram(0);
    check_gl_error();
}

inline GLint
SfmShader::get_program_property (int pname)
{
    GLint ret;
    glGetProgramiv(this->prog_id, pname, &ret);
    check_gl_error();
    return ret;
}

inline GLint
SfmShader::get_shader_property (GLuint shader_id, int pname)
{
    GLint ret;
    glGetShaderiv(shader_id, pname, &ret);
    check_gl_error();
    return ret;
}

GL_NAMESPACE_END

#endif /* SFM_SHADER_HEADER */
