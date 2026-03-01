/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <QBoxLayout>

#include "scenemanager.h"
#include "scene_inspect/scene_inspect.h"

SceneInspect::SceneInspect (QWidget* parent)
    : MainWindowTab(parent)
{
    this->gl_widget = new GLWidget();
    this->addin_manager = new AddinManager(this->gl_widget);
    this->gl_widget->set_context(this->addin_manager);

    this->connect(&SceneManager::get(), SIGNAL(scene_selected(sfm::Scene::Ptr)),
        this, SLOT(on_scene_selected(sfm::Scene::Ptr)));
    this->connect(&SceneManager::get(), SIGNAL(view_selected(sfm::View::Ptr)),
        this, SLOT(on_view_selected(sfm::View::Ptr)));
    this->connect(this, SIGNAL(tab_activated()), SLOT(on_tab_activated()));

    QHBoxLayout* main_layout = new QHBoxLayout(this);
    main_layout->addWidget(this->gl_widget, 1);

    this->setFocusProxy(this->gl_widget);
}

void
SceneInspect::reset (void)
{
    this->addin_manager->reset_scene();
}

void
SceneInspect::on_tab_activated (void)
{
    if (this->next_view != nullptr)
        this->on_view_selected(this->next_view);
}

void
SceneInspect::on_scene_selected (sfm::Scene::Ptr scene)
{
    this->addin_manager->set_scene(scene);
}

void
SceneInspect::on_view_selected (sfm::View::Ptr view)
{
    if (!this->is_tab_active)
    {
        this->next_view = view;
        return;
    }

    this->addin_manager->set_view(view);
    this->next_view = nullptr;
}

AddinManager*
SceneInspect::get_addin_manager (void)
{
    return this->addin_manager;
}

QString
SceneInspect::get_title (void)
{
    return tr("Scene inspect");
}
