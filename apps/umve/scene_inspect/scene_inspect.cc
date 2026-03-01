/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <QBoxLayout>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>

#include <iostream>

#include "guihelpers.h"
#include "scenemanager.h"
#include "scene_inspect/scene_inspect.h"

SceneInspect::SceneInspect (QWidget* parent)
    : MainWindowTab(parent)
{
    /* Create toolbar, add actions. */
    QToolBar* toolbar = new QToolBar("Tools");
    this->create_actions(toolbar);

    /* Create sidebar for rendering controls. */
    this->sidebar = new QWidget();

    /* Create GL context. */
    this->gl_widget = new GLWidget();
    this->addin_manager = new AddinManager(this->gl_widget, this->sidebar);
    this->gl_widget->set_context(this->addin_manager);

    /* Connect signals. */
    this->connect(&SceneManager::get(), SIGNAL(scene_selected(mve::Scene::Ptr)),
        this, SLOT(on_scene_selected(mve::Scene::Ptr)));
    this->connect(&SceneManager::get(), SIGNAL(view_selected(mve::View::Ptr)),
        this, SLOT(on_view_selected(mve::View::Ptr)));
    this->connect(this, SIGNAL(tab_activated()), SLOT(on_tab_activated()));

    /* Pack everything together. */
    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->addWidget(toolbar);
    vbox->addWidget(this->gl_widget);

    QHBoxLayout* main_layout = new QHBoxLayout(this);
    main_layout->addLayout(vbox, 1);
    main_layout->addWidget(this->sidebar);

    // Focus on this widget
    // TODO(oalexan1): Think more about this.
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
SceneInspect::create_actions (QToolBar* toolbar)
{
    this->action_show_details = new QAction(tr("Show &Details"), this);
    this->action_show_details->setCheckable(true);
    this->action_show_details->setChecked(true);
    this->connect(this->action_show_details, SIGNAL(triggered()),
        this, SLOT(on_details_toggled()));

    this->action_save_screenshot = new QAction(tr("Save Screenshot"), this);
    this->connect(this->action_save_screenshot, SIGNAL(triggered()),
        this, SLOT(on_save_screenshot()));

    toolbar->addAction(this->action_save_screenshot);
    toolbar->addWidget(get_expander());
    toolbar->addAction(this->action_show_details);
}

void
SceneInspect::on_details_toggled (void)
{
    bool show = this->action_show_details->isChecked();
    this->sidebar->setVisible(show);
}

void
SceneInspect::on_scene_selected (mve::Scene::Ptr scene)
{
    this->addin_manager->set_scene(scene);
}

void
SceneInspect::on_view_selected (mve::View::Ptr view)
{
    if (!this->is_tab_active)
    {
        this->next_view = view;
        return;
    }

    this->addin_manager->set_view(view);
    this->next_view = nullptr;
}

void
SceneInspect::on_save_screenshot (void)
{
    QString filename = QFileDialog::getSaveFileName(this,
        "Export Image...");
    if (filename.size() == 0)
        return;

    QImage img = this->gl_widget->grabFramebuffer();
    bool success = img.save(filename);
    if (!success)
        QMessageBox::critical(this, "Cannot save image",
            "Error saving image");
}

QString
SceneInspect::get_title (void)
{
    return tr("Scene inspect");
}
