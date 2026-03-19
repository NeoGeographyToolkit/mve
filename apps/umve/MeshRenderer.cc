/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <stdexcept>

#include "MeshRenderer.h"

GL_NAMESPACE_BEGIN

/* ---- VertexBuffer ---- */

VertexBuffer::VertexBuffer (void)
{
    glGenBuffers(1, &this->vbo_id);
    check_gl_error();
    this->vbo_target = GL_ARRAY_BUFFER;
    this->datatype = GL_FLOAT;
    this->vpv = 0;
    this->elems = 0;
}

void
VertexBuffer::set_data (GLfloat const* data, GLsizei elems, GLint vpv)
{
    this->vbo_target = GL_ARRAY_BUFFER;
    this->datatype = GL_FLOAT;
    this->vpv = vpv;
    this->elems = elems;

    this->bind();
    GLsizeiptr bytes = elems * vpv * sizeof(GLfloat);
    glBufferData(this->vbo_target, bytes, data, GL_STATIC_DRAW);
    check_gl_error();
}

void
VertexBuffer::set_indices (GLuint const* data, GLsizei num_indices)
{
    this->vbo_target = GL_ELEMENT_ARRAY_BUFFER;
    this->datatype = GL_UNSIGNED_INT;
    this->vpv = 3;
    this->elems = num_indices;

    this->bind();
    GLsizeiptr bytes = num_indices * sizeof(GLuint);
    glBufferData(this->vbo_target, bytes, data, GL_STATIC_DRAW);
    check_gl_error();
}

/* ---- VertexArray ---- */

void
VertexArray::assign_attrib (BoundVBO const& bound_vbo)
{
    VertexBuffer::Ptr vbo = bound_vbo.first;
    std::string const& name = bound_vbo.second;

    GLint location = this->shader->get_attrib_location(name.c_str());
    if (location < 0)
        return;

    vbo->bind();
    glVertexAttribPointer(location, vbo->get_values_per_vertex(),
        vbo->get_data_type(), GL_TRUE, 0, nullptr);
    check_gl_error();
    glEnableVertexAttribArray(location);
    check_gl_error();
}

void
VertexArray::draw (void)
{
    if (this->vert_vbo == nullptr)
        throw std::runtime_error("No vertex VBO set!");

    if (this->shader == nullptr)
        throw std::runtime_error("No shader program set!");

    glBindVertexArray(this->vao_id);
    check_gl_error();

    this->shader->bind();

    this->assign_attrib(BoundVBO(this->vert_vbo, SFM_ATTRIB_POSITION));

    for (std::size_t i = 0; i < this->vbo_list.size(); ++i)
        this->assign_attrib(this->vbo_list[i]);

    if (this->index_vbo != nullptr) {
        this->index_vbo->bind();
        glDrawElements(this->primitive, this->index_vbo->get_element_amount(),
            GL_UNSIGNED_INT, nullptr);
        check_gl_error();
    } else {
        glDrawArrays(this->primitive, 0, this->vert_vbo->get_element_amount());
        check_gl_error();
    }

    this->shader->unbind();
    glBindVertexArray(0);
    check_gl_error();
}

/* ---- MeshRenderer ---- */

void
MeshRenderer::set_mesh (sfm::TriangleMesh::ConstPtr mesh)
{
    if (mesh == nullptr)
        throw std::invalid_argument("Got null mesh");

    this->reset_vertex_array();

    sfm::TriangleMesh::VertexList const& verts(mesh->get_vertices());
    sfm::TriangleMesh::FaceList const& faces(mesh->get_faces());
    sfm::TriangleMesh::NormalList const& vnormals(mesh->get_vertex_normals());
    sfm::TriangleMesh::ColorList const& vcolors(mesh->get_vertex_colors());
    sfm::TriangleMesh::TexCoordList const& vtexuv(mesh->get_vertex_texcoords());

    {
        VertexBuffer::Ptr vbo = VertexBuffer::create();
        vbo->set_data(&verts[0][0], (GLsizei)verts.size(), 3);
        this->set_vertex_vbo(vbo);
    }

    if (!faces.empty()) {
        VertexBuffer::Ptr vbo = VertexBuffer::create();
        vbo->set_indices(&faces[0], (GLsizei)faces.size());
        this->set_index_vbo(vbo);
    }

    if (!vnormals.empty()) {
        VertexBuffer::Ptr vbo = VertexBuffer::create();
        vbo->set_data(&vnormals[0][0], (GLsizei)vnormals.size(), 3);
        this->add_vbo(vbo, SFM_ATTRIB_NORMAL);
    }

    if (!vcolors.empty()) {
        VertexBuffer::Ptr vbo = VertexBuffer::create();
        vbo->set_data(&vcolors[0][0], (GLsizei)vcolors.size(), 4);
        this->add_vbo(vbo, SFM_ATTRIB_COLOR);
    }

    if (!vtexuv.empty()) {
        VertexBuffer::Ptr vbo = VertexBuffer::create();
        vbo->set_data(&vtexuv[0][0], (GLsizei)vtexuv.size(), 2);
        this->add_vbo(vbo, SFM_ATTRIB_TEXCOORD);
    }
}

GL_NAMESPACE_END
