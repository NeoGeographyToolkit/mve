/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <cstdlib>
#include <iostream>

#include <QBoxLayout>

#include "guihelpers.h"
#include "scene_inspect/addin_manager.h"

AddinManager::AddinManager (GLWidget* gl_widget, QWidget* sidebar)
{
    /* Initialize state and widgets. */
    this->state.gl_widget = gl_widget;
    this->state.ui_needs_redraw = true;

    /* Instanciate addins. */
    this->frusta_renderer = new AddinFrustaSceneRenderer();

    /* Register addins. */
    this->addins.push_back(this->frusta_renderer);

    /* Create sidebar headers. */
    QCollapsible* frusta_header = new QCollapsible("Frusta Rendering",
        this->frusta_renderer->get_sidebar_widget());

    /* Create the sidebar layout. */
    QVBoxLayout* sidebar_layout = new QVBoxLayout(sidebar);
    sidebar_layout->setSpacing(5);
    sidebar_layout->setContentsMargins(5, 5, 5, 5);
    sidebar_layout->addWidget(frusta_header, 0);
    sidebar_layout->addStretch(1);
}

bool
AddinManager::keyboard_event(const ogl::KeyboardEvent &event)
{
    for (std::size_t i = 0; i < this->addins.size(); ++i)
        if (this->addins[i]->keyboard_event(event))
            return true;

    return ogl::CameraTrackballContext::keyboard_event(event);
}

bool
AddinManager::mouse_event (const ogl::MouseEvent &event)
{
    for (std::size_t i = 0; i < this->addins.size(); ++i)
        if (this->addins[i]->mouse_event(event))
            return true;

    return ogl::CameraTrackballContext::mouse_event(event);
}

void
AddinManager::set_scene (mve::Scene::Ptr scene)
{
    this->state.scene = scene;
    this->state.repaint();
}

void
AddinManager::set_view (mve::View::Ptr view)
{
    this->state.view = view;
    this->state.repaint();
}

void
AddinManager::reset_scene (void)
{
    this->state.scene = nullptr;
    this->state.view = nullptr;
    this->state.repaint();
}

void
AddinManager::init_impl (void)
{
#ifdef _WIN32
    /* Initialize GLEW. */
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK)
    {
        std::cout << "Error initializing GLEW: " << glewGetErrorString(err)
            << "\n";
        std::exit(EXIT_FAILURE);
    }
#endif

    /* Load shaders. */
    this->state.load_shaders();
    this->state.init_ui();

    for (std::size_t i = 0; i < this->addins.size(); ++i)
    {
        this->addins[i]->set_state(&this->state);
        this->addins[i]->init();
    }
}

void
AddinManager::resize_impl (int old_width, int old_height)
{
    this->ogl::CameraTrackballContext::resize_impl(old_width, old_height);
    for (std::size_t i = 0; i < this->addins.size(); ++i)
        this->addins[i]->resize(this->ogl::Context::width,
                                this->ogl::Context::height);

    this->state.ui_needs_redraw = true;
}

void
AddinManager::paint_impl (void)
{
    /* Set clear color (black) and depth. */
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearDepth(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    this->state.send_uniform(this->camera);
    if (this->state.ui_needs_redraw)
        this->state.clear_ui(ogl::Context::width, ogl::Context::height);

    /* Paint all implementations. */
    for (std::size_t i = 0; i < this->addins.size(); ++i)
    {
        if (this->state.ui_needs_redraw)
            this->addins[i]->redraw_gui();
        this->addins[i]->paint();
    }

    /* Draw UI. */
    if (this->state.ui_needs_redraw)
        this->state.gui_texture->upload(this->state.ui_image);

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    this->state.gui_texture->bind();
    this->state.texture_shader->bind();
    this->state.gui_renderer->draw();
    this->state.ui_needs_redraw = false;
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}
