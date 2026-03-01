/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef UMVE_MAIN_WINDOW_HEADER
#define UMVE_MAIN_WINDOW_HEADER

#include "ogl/opengl.h"

#include <string>
#include <QMainWindow>

#include "mve/view.h"

#include "scene_inspect/scene_inspect.h"
#include "sceneoverview.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

private:
    QDockWidget* dock_scene;

    QStatusBar* statusbar;
    QLabel* memory_label;
    QTimer* update_timer;

    SceneOverview* scene_overview;
    SceneInspect* tab_sceneinspect;

    QAction* action_exit;
    QAction* action_about;

    QMenu* menu_scene;
    QMenu* menu_help;

private:
    void create_actions (void);
    void create_menus (void);
    bool perform_close_scene (void);

private slots:
    void on_about (void);
    void on_update_memory (void);

    void closeEvent (QCloseEvent* event);

public:
    MainWindow (int width, int height);
    ~MainWindow (void);

    void load_scene (std::string const& path);
    // load images and tsai camera files
    void load_scene (std::vector<std::string> const& images,
                     std::vector<std::string> const& cameras);

    void open_scene_inspect (void);
};

inline
MainWindow::~MainWindow (void)
{
}

#endif /* UMVE_MAIN_WINDOW_HEADER */
