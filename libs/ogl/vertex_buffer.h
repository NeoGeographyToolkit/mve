/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef OGL_VERTEX_BUFFER_HEADER
#define OGL_VERTEX_BUFFER_HEADER

#include <memory>

#include "ogl/defines.h"
#include "ogl/opengl.h"
#include "ogl/check_gl_error.h"

OGL_NAMESPACE_BEGIN

// OpenGL vertex buffer object (VBO) abstraction.
class VertexBuffer
{
public:
    typedef std::shared_ptr<VertexBuffer> Ptr;

public:
    ~VertexBuffer (void);
    static Ptr create (void);

    /** Sets float data for the VBO: amount of elements and values per vertex. */
    void set_data (GLfloat const* data, GLsizei elems, GLint vpv);
    /** Sets index data for the VBO. */
    void set_indices (GLuint const* data, GLsizei num_indices);

    GLenum get_vbo_target (void) const;
    GLenum get_data_type (void) const;
    GLint get_values_per_vertex (void) const;
    GLsizei get_element_amount (void) const;

    void bind (void);

private:
    VertexBuffer (void);

private:
    GLuint vbo_id;
    GLenum vbo_target;
    GLenum datatype;
    GLint vpv;
    GLsizei elems;
};

/* ---------------------------------------------------------------- */

inline
VertexBuffer::~VertexBuffer (void)
{
    glDeleteBuffers(1, &this->vbo_id);
    check_gl_error();
}

inline VertexBuffer::Ptr
VertexBuffer::create (void)
{
    return Ptr(new VertexBuffer);
}

inline GLenum
VertexBuffer::get_vbo_target (void) const
{
    return this->vbo_target;
}

inline GLint
VertexBuffer::get_values_per_vertex (void) const
{
    return this->vpv;
}

inline GLsizei
VertexBuffer::get_element_amount (void) const
{
    return this->elems;
}

inline GLenum
VertexBuffer::get_data_type (void) const
{
    return this->datatype;
}

inline void
VertexBuffer::bind (void)
{
    glBindBuffer(this->vbo_target, this->vbo_id);
}

OGL_NAMESPACE_END

#endif /* OGL_VERTEX_BUFFER_HEADER */
