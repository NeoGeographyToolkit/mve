// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).
/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef SFM_MAIN_WINDOW_HEADER
#define SFM_MAIN_WINDOW_HEADER

#include <string>
#include <vector>
#include <QMainWindow>

#include "GlWidget.h"
#include "SceneRenderer.h"
#include "SceneOverview.h"

class SfmMainWindow : public QMainWindow
{
    Q_OBJECT

private:
    QDockWidget* dock_scene;

    SceneOverview* scene_overview;
    GlWidget* gl_widget;
    SceneRenderer* scene_renderer;

    QAction* action_quit;
    QAction* action_about;

    QMenu* menu_file;
    QMenu* menu_view;
    QMenu* menu_help;

private:
    void create_actions (void);
    void create_menus (void);
    void perform_close_scene (void);

private slots:
    void on_scene_selected (sfm::Scene::Ptr scene);
    void on_view_selected (sfm::View::Ptr view);
    void on_about (void);
    void on_frusta_size (void);

    void closeEvent (QCloseEvent* event);

public:
    SfmMainWindow (int width, int height);
    ~SfmMainWindow (void);

    void load_scene (std::vector<std::string> const& images,
                     std::vector<std::string> const& cameras);
};

inline
SfmMainWindow::~SfmMainWindow (void)
{
}

#endif /* SFM_MAIN_WINDOW_HEADER */
