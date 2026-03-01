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

#include <QWidget>

#include "sfm_view_utils.h"

#include "glwidget.h"
#include "addin_manager.h"

class SceneInspect : public QWidget
{
    Q_OBJECT

public:
    SceneInspect (QWidget* parent = nullptr);
    void reset (void);
    AddinManager* get_addin_manager (void);

private slots:
    void on_scene_selected (sfm::Scene::Ptr scene);
    void on_view_selected (sfm::View::Ptr view);

private:
    AddinManager* addin_manager;
    GLWidget* gl_widget;
};

#endif /* UMVE_SCENE_INSPECT_HEADER */
