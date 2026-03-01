/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>

#include "addin_state.h"

// Inline GLSL shaders. Colors are baked into mesh vertices:
// green for the ground grid, white for camera frusta,
// yellow for view direction, RGB for camera coordinate axes.

static char const* const WIREFRAME_VERT =
    "#version 330 core\n"
    "in vec4 pos;\n"
    "in vec4 color;\n"
    "out vec4 ocolor;\n"
    "uniform mat4 viewmat;\n"
    "uniform mat4 projmat;\n"
    "void main(void) {\n"
    "    ocolor = color;\n"
    "    gl_Position = projmat * (viewmat * pos);\n"
    "}\n";

static char const* const WIREFRAME_FRAG =
    "#version 330 core\n"
    "in vec4 ocolor;\n"
    "layout(location=0) out vec4 frag_color;\n"
    "void main(void) {\n"
    "    gl_FragDepth = gl_FragCoord.z;\n"
    "    frag_color = ocolor;\n"
    "}\n";

AddinState::AddinState (void)
    : gl_widget(nullptr)
{
}

void
AddinState::repaint (void)
{
    if (this->gl_widget == nullptr)
        return;

    this->gl_widget->repaint();
}

void
AddinState::make_current_context (void)
{
    if (this->gl_widget == nullptr)
        return;

    this->gl_widget->makeCurrent();
}

void
AddinState::load_shaders (void)
{
    if (!this->wireframe_shader)
        this->wireframe_shader = ogl::ShaderProgram::create();

    this->wireframe_shader->load_vert_code(WIREFRAME_VERT);
    this->wireframe_shader->load_frag_code(WIREFRAME_FRAG);
}

void
AddinState::send_uniform (ogl::Camera const& cam)
{
    this->wireframe_shader->bind();
    this->wireframe_shader->send_uniform("viewmat", cam.view);
    this->wireframe_shader->send_uniform("projmat", cam.proj);
}
