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

#include "scene_inspect/addin_manager.h"

AddinManager::AddinManager (GLWidget* gl_widget)
{
    this->state.gl_widget = gl_widget;
    this->frusta_renderer = new AddinFrustaSceneRenderer();
    this->addins.push_back(this->frusta_renderer);
}

AddinFrustaSceneRenderer*
AddinManager::get_frusta_renderer (void)
{
    return this->frusta_renderer;
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
AddinManager::set_scene (sfm::Scene::Ptr scene)
{
    this->state.scene = scene;
    this->state.repaint();
}

void
AddinManager::set_view (sfm::View::Ptr view)
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

    this->state.load_shaders();

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
}

void
AddinManager::paint_impl (void)
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearDepth(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    this->state.send_uniform(this->camera);

    for (std::size_t i = 0; i < this->addins.size(); ++i)
        this->addins[i]->paint();
}
