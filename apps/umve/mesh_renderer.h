/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef OGL_MESH_RENDERER_HEADER
#define OGL_MESH_RENDERER_HEADER

#include <utility>
#include <vector>
#include <string>
#include <memory>

#include "sfm_math.h"
#include "ogl_common.h"
#include "shader_program.h"

OGL_NAMESPACE_BEGIN

/* ---- VertexBuffer ---- */

// OpenGL vertex buffer object (VBO) abstraction.
class VertexBuffer
{
public:
    typedef std::shared_ptr<VertexBuffer> Ptr;

public:
    ~VertexBuffer (void);
    static Ptr create (void);

    void set_data (GLfloat const* data, GLsizei elems, GLint vpv);
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

/* ---- VertexArray ---- */

// OpenGL vertex array object (VAO) abstraction.
class VertexArray
{
public:
    typedef std::shared_ptr<VertexArray> Ptr;

    typedef std::pair<VertexBuffer::Ptr, std::string> BoundVBO;
    typedef std::vector<BoundVBO> VBOList;

public:
    virtual ~VertexArray (void);

    void set_primitive (GLuint primitive);
    void set_shader (ShaderProgram::Ptr shader);
    void set_vertex_vbo (VertexBuffer::Ptr vbo);
    void set_index_vbo (VertexBuffer::Ptr vbo);
    void add_vbo (VertexBuffer::Ptr vbo, std::string const& name);
    void reset_vertex_array (void);
    void draw (void);

protected:
    VertexArray (void);
    void assign_attrib (BoundVBO const& bound_vbo);

private:
    GLuint vao_id;
    GLuint primitive;
    ShaderProgram::Ptr shader;

    VertexBuffer::Ptr vert_vbo;
    VertexBuffer::Ptr index_vbo;
    VBOList vbo_list;
};

inline
VertexArray::VertexArray (void)
{
    glGenVertexArrays(1, &this->vao_id);
    check_gl_error();
    this->primitive = GL_TRIANGLES;
}

inline
VertexArray::~VertexArray (void)
{
    glDeleteVertexArrays(1, &this->vao_id);
    check_gl_error();
}

inline void
VertexArray::set_primitive (GLuint primitive)
{
    this->primitive = primitive;
}

inline void
VertexArray::set_vertex_vbo (VertexBuffer::Ptr vbo)
{
    this->vert_vbo = vbo;
}

inline void
VertexArray::set_index_vbo (VertexBuffer::Ptr vbo)
{
    this->index_vbo = vbo;
}

inline void
VertexArray::add_vbo (VertexBuffer::Ptr vbo, std::string const& name)
{
    this->vbo_list.push_back(std::make_pair(vbo, name));
}

inline void
VertexArray::reset_vertex_array(void)
{
    this->vert_vbo.reset();
    this->index_vbo.reset();
    this->vbo_list.clear();
    glDeleteVertexArrays(1, &this->vao_id);
    check_gl_error();
    glGenVertexArrays(1, &this->vao_id);
    check_gl_error();
}

inline void
VertexArray::set_shader (ShaderProgram::Ptr shader)
{
    this->shader = shader;
}

/* ---- MeshRenderer ---- */

// Takes a TriangleMesh and creates VBOs for rendering.
class MeshRenderer : public VertexArray
{
public:
    typedef std::shared_ptr<MeshRenderer> Ptr;

public:
    static Ptr create (void);
    static Ptr create (mve::TriangleMesh::ConstPtr mesh);
    void set_mesh (mve::TriangleMesh::ConstPtr mesh);

private:
    MeshRenderer (void);
    MeshRenderer (mve::TriangleMesh::ConstPtr mesh);
};

inline MeshRenderer::Ptr
MeshRenderer::create (void)
{
    return Ptr(new MeshRenderer());
}

inline MeshRenderer::Ptr
MeshRenderer::create (mve::TriangleMesh::ConstPtr mesh)
{
    return Ptr(new MeshRenderer(mesh));
}

inline
MeshRenderer::MeshRenderer (void)
{
}

inline
MeshRenderer::MeshRenderer (mve::TriangleMesh::ConstPtr mesh)
{
    this->set_mesh(mesh);
}

OGL_NAMESPACE_END

#endif /* OGL_MESH_RENDERER_HEADER */
