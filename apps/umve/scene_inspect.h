/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef UMVE_SCENE_INSPECT_HEADER
#define UMVE_SCENE_INSPECT_HEADER

#include "ogl/opengl.h"

#include <string>

#include <QGLWidget>
#include <QString>
#include <QWidget>

#include "sfm_view_utils.h"

#include "mainwindowtab.h"
#include "glwidget.h"
#include "addin_manager.h"

class SceneInspect : public MainWindowTab
{
    Q_OBJECT

public:
    SceneInspect (QWidget* parent = nullptr);
    void reset (void);
    AddinManager* get_addin_manager (void);

    virtual QString get_title (void);

private slots:
    void on_scene_selected (sfm::Scene::Ptr scene);
    void on_view_selected (sfm::View::Ptr view);
    void on_tab_activated (void);

private:
    sfm::View::Ptr next_view;
    AddinManager* addin_manager;
    GLWidget* gl_widget;
};

#endif /* UMVE_SCENE_INSPECT_HEADER */
