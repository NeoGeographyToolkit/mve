/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef UMVE_ADDIN_STATE_HEADER
#define UMVE_ADDIN_STATE_HEADER

#include "mve/scene.h"
#include "mve/view.h"
#include "ogl/shader_program.h"
#include "ogl/camera.h"

#include "glwidget.h"

struct AddinState
{
public:
    GLWidget* gl_widget;
    ogl::ShaderProgram::Ptr wireframe_shader;
    mve::Scene::Ptr scene;
    mve::View::Ptr view;

public:
    AddinState (void);
    void load_shaders (void);
    void send_uniform (ogl::Camera const& cam);

    void repaint (void);
    void make_current_context (void);
};

#endif // UMVE_ADDIN_STATE_HEADER
