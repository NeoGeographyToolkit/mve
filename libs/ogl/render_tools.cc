/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>
#include "mve/mesh.h"
#include "ogl/mesh_renderer.h"
#include "ogl/render_tools.h"

OGL_NAMESPACE_BEGIN

VertexArray::Ptr
create_axis_renderer (ShaderProgram::Ptr shader)
{
    /* Create the mesh. */
    mve::TriangleMesh::Ptr mesh(mve::TriangleMesh::create());
    mve::TriangleMesh::VertexList& verts(mesh->get_vertices());
    mve::TriangleMesh::FaceList& faces(mesh->get_faces());
    mve::TriangleMesh::ColorList& colors(mesh->get_vertex_colors());

    // TODO(oalexan1): For now the axes are not drawn, but the ground plane is drawn instead. Need to have here a customizable switch.
    bool draw_axes = false;

    for (int hs = 0; hs < 2; ++hs) {

        if (!draw_axes) 
          continue; // do not draw the axes

        for (int i = 0; i < 3; ++i) { // For each axis
            float axis[4] =
            {
                i == 0 ? 1.0f : 0.0f,
                i == 1 ? 1.0f : 0.0f,
                i == 2 ? 1.0f : 0.0f,
                1.0f
            };

            math::Vec4f color(axis);
            color = color * (hs == 0 ? 1.0f : 0.2f);
            color[3] = 1.0f;

            float sign = (hs == 0 ? 1.0f : -1.0f);
            for (int i = 0; i < 3; ++i) axis[i] *= sign;

            verts.push_back(math::Vec3f(0.0f, 0.0f, 0.0f));
            verts.push_back(math::Vec3f(axis[0] * 100.0f, axis[1] * 100.0f, axis[2] * 100.0f));
            faces.push_back(verts.size() - 2);
            faces.push_back(verts.size() - 1);
            colors.push_back(color);
            colors.push_back(color);

            for (float j = 1.0f; j < 10.0f; j += 1.0f)
            {
                for (float scale = 0.01f; scale < 15.0f; scale *= 10.0f)
                {
                    verts.push_back(math::Vec3f
                        (axis[0] * j + axis[1] * 0.1f + axis[2] * 0.1f,
                        axis[1] * j + axis[0] * 0.1f, axis[2] * j) * scale);
                    verts.push_back(math::Vec3f
                        (axis[0] * j + axis[1] * -0.1f + axis[2] * -0.1f,
                        axis[1] * j + axis[0] * -0.1f, axis[2] * j) * scale);
                    faces.push_back(verts.size() - 2);
                    faces.push_back(verts.size() - 1);
                    colors.push_back(color);
                    colors.push_back(color);

                    verts.push_back(math::Vec3f
                        (axis[0] * j, axis[1] * j + axis[2] * 0.1f,
                        axis[2] * j + axis[0] * 0.1f + axis[1] * 0.1f) * scale);
                    verts.push_back(math::Vec3f
                        (axis[0] * j, axis[1] * j + axis[2] * -0.1f,
                        axis[2] * j + axis[0] * -0.1f + axis[1] * -0.1f) * scale);
                    faces.push_back(verts.size() - 2);
                    faces.push_back(verts.size() - 1);
                    colors.push_back(color);
                    colors.push_back(color);
                }
            }
        }
    }

    // Draw the ground plane.
    // TODO(oalexan1): Add a switch for turning this off. This must not
    // show up when a mesh is loaded.
    // Set up a color
    math::Vec4f color;
    color[0] = 0.0f;
    color[1] = 1.0f;
    color[2] = 0.0f;
    color[3] = 1.0f;

    // Draw a tiled plane to signify the ground
    // TODO(oalexan1): Make this plane filled and in different color.
    for (size_t i = 1; i <= 22; i++) {
        double x[2], y[2], z[2];
        z[0] = -10.0; z[1] = -10.0;
        if (i <= 11) {
            y[0] = 2.0*(i - 6.0); y[1] = y[0];
            x[0] = -10.0; x[1] = 10.0;
        } else {
            x[0] = 2.0*(i - 11 - 6.0); x[1] = x[0];
            y[0] = -10.0; y[1] = 10.0;
        }

        verts.push_back(math::Vec3f(x[0], y[0], -10.0f));
        verts.push_back(math::Vec3f(x[1], y[1], -10.0f));
        faces.push_back(verts.size() - 2);
        faces.push_back(verts.size() - 1);
        colors.push_back(color);
        colors.push_back(color);
    }

    /* Generate the renderer. */
    MeshRenderer::Ptr ret(MeshRenderer::create());
    ret->set_primitive(GL_LINES);
    ret->set_shader(shader);
    ret->set_mesh(mesh);

    return ret;
}

/* ---------------------------------------------------------------- */

VertexArray::Ptr
create_fullscreen_quad (ShaderProgram::Ptr shader)
{
    mve::TriangleMesh::Ptr mesh(mve::TriangleMesh::create());
    mve::TriangleMesh::VertexList& verts(mesh->get_vertices());
    mve::TriangleMesh::NormalList& vnormals(mesh->get_vertex_normals());
    mve::TriangleMesh::TexCoordList& vtexuv(mesh->get_vertex_texcoords());

    verts.push_back(math::Vec3f(-1.0f, 1.0f, 0.0f));
    verts.push_back(math::Vec3f(-1.0f, -1.0f, 0.0f));
    verts.push_back(math::Vec3f(1.0f, 1.0f, 0.0f));
    verts.push_back(math::Vec3f(1.0f, -1.0f, 0.0f));
    vnormals.push_back(math::Vec3f(0.0f, 0.0f, 1.0f));
    vnormals.push_back(math::Vec3f(0.0f, 0.0f, 1.0f));
    vnormals.push_back(math::Vec3f(0.0f, 0.0f, 1.0f));
    vnormals.push_back(math::Vec3f(0.0f, 0.0f, 1.0f));
    vtexuv.push_back(math::Vec2f(0.0f, 0.0f));
    vtexuv.push_back(math::Vec2f(0.0f, 1.0f));
    vtexuv.push_back(math::Vec2f(1.0f, 0.0f));
    vtexuv.push_back(math::Vec2f(1.0f, 1.0f));

    /* Generate the renderer. */
    MeshRenderer::Ptr ret(MeshRenderer::create());
    ret->set_primitive(GL_TRIANGLE_STRIP);
    ret->set_shader(shader);
    ret->set_mesh(mesh);
    return ret;
}

OGL_NAMESPACE_END
