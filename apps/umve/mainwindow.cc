/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <iostream>
#include <cstdlib>

#include <QApplication>
#include <QDockWidget>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QStatusBar>

#include "util/file_system.h"

#include "scenemanager.h"
#include "mainwindow.h"

MainWindow::MainWindow (int width, int height) {
    this->scene_overview = new SceneOverview(this);

    this->tab_sceneinspect = new SceneInspect(this);

    /* Create dock widgets. */
    this->dock_scene = new QDockWidget(tr("Scene"));
    this->dock_scene->setWidget(this->scene_overview);

    this->memory_label = new QLabel("Memory: <unknown>");
    this->statusbar = new QStatusBar();
    this->statusbar->addWidget(this->memory_label);
    this->setStatusBar(this->statusbar);

    /* Create actions and menus. */
    this->create_actions();
    this->create_menus();

    /* Configure window. */
    QWidget* central_widget(new QWidget(this));
    QLayout* central_layout(new QVBoxLayout(central_widget));
    central_layout->addWidget(this->tab_sceneinspect);

    this->setWindowTitle(tr("sfm_view"));
    this->setCentralWidget(central_widget);
    this->addDockWidget(Qt::LeftDockWidgetArea, this->dock_scene);
    this->enable_scene_actions(false);
    this->resize(width, height);

    /* Start update timer and init. */
    this->update_timer = new QTimer(this);
    this->update_timer->start(2000);
    this->on_update_memory();

    /* Connect signals. */
    this->connect(this->update_timer, SIGNAL(timeout()),
        this, SLOT(on_update_memory()));

    this->show();
}

void MainWindow::load_scene (std::string const& path) {
    mve::Scene::Ptr scene;
    try {
        scene = mve::Scene::create(path);
    } catch (std::exception& e) {
        QMessageBox::information(this, tr("Error loading scene"),
            tr("Scene could not be loaded.\n"
            "Directory: %1\nError: %2").arg(QString(path.c_str()), e.what()));
        return;
    }

    SceneManager::get().select_scene(scene);
    this->enable_scene_actions(true);
}

// Load images and tsai camera files
void MainWindow::load_scene (std::vector<std::string> const& images,
    std::vector<std::string> const& cameras) {
    mve::Scene::Ptr scene;
    try
    { scene = mve::Scene::create(images, cameras); }
    catch (std::exception& e)
    {
        QMessageBox::information(this, tr("Error loading scene"),
            tr("Scene could not be loaded.\n"
            "Error: %1").arg(e.what()));
        return;
    }

    SceneManager::get().select_scene(scene);
    this->enable_scene_actions(true);
}

void
MainWindow::open_scene_inspect (void)
{
}

void
MainWindow::create_actions (void)
{
    this->action_close_scene = new QAction(
        tr("Close scene"), this);
    this->connect(this->action_close_scene, SIGNAL(triggered()),
        this, SLOT(on_close_scene()));

    this->action_cache_cleanup = new QAction(
        tr("Cache cleanup"), this);
    this->connect(this->action_cache_cleanup, SIGNAL(triggered()),
        this, SLOT(on_cache_cleanup()));

    this->action_refresh_scene = new QAction(
        tr("Refresh scene"), this);
    this->connect(this->action_refresh_scene, SIGNAL(triggered()),
        this, SLOT(on_refresh_scene()));

    this->action_exit = new QAction(tr("E&xit"), this);
    this->action_exit->setShortcut(tr("Ctrl+Q"));
    this->connect(this->action_exit, SIGNAL(triggered()),
        this, SLOT(close()));

    this->action_about = new QAction(tr("&About"), this);
    this->connect(this->action_about, SIGNAL(triggered()),
        this, SLOT(on_about()));
}

void
MainWindow::create_menus (void)
{
    this->menu_scene = new QMenu(tr("&Scene"), this);
    this->menu_scene->addAction(this->action_close_scene);
    this->menu_scene->addSeparator();
    this->menu_scene->addAction(this->action_cache_cleanup);
    this->menu_scene->addAction(this->action_refresh_scene);
    this->menu_scene->addSeparator();
    this->menu_scene->addAction(this->action_exit);

    this->menu_help = new QMenu(tr("&Help"), this);
    this->menu_help->addAction(this->action_about);

    this->menuBar()->addMenu(this->menu_scene);
    this->menuBar()->addMenu(this->menu_help);
    this->menuBar()->show();

    this->scene_overview->add_toolbar_action(this->action_close_scene);
    this->scene_overview->add_toolbar_action(this->action_cache_cleanup);
    this->scene_overview->add_toolbar_action(this->action_refresh_scene);
}

bool
MainWindow::perform_close_scene (void)
{
    mve::Scene::Ptr scene = SceneManager::get().get_scene();

    if (scene == nullptr)
        return true;

    if (scene->is_dirty())
    {
        QMessageBox::StandardButton answer = QMessageBox::question(this,
            tr("Close scene?"), tr("Really close scene?\n"
            "Unsaved changes get lost, this cannot be undone."),
            QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Yes);

        if (answer != QMessageBox::Yes)
            return false;
    }

    /* Remove references to the scene. */
    SceneManager::get().reset_image();
    SceneManager::get().reset_view();
    SceneManager::get().reset_scene();
    this->tab_sceneinspect->reset();
    this->enable_scene_actions(false);

    return true;
}

void
MainWindow::enable_scene_actions (bool value)
{
    this->action_close_scene->setEnabled(value);
    this->action_cache_cleanup->setEnabled(value);
    this->action_refresh_scene->setEnabled(value);
}

void
MainWindow::on_close_scene (void)
{
    this->perform_close_scene();
}

void
MainWindow::on_refresh_scene (void)
{
    SceneManager::get().refresh_scene();
}

void
MainWindow::on_about (void)
{
    QMessageBox::about(this, tr("About sfm_view"),
        tr("sfm_view - camera position viewer for ASP."));
}

void
MainWindow::on_update_memory (void)
{
    mve::Scene::Ptr scene = SceneManager::get().get_scene();
    std::size_t mem = 0;
    if (scene != nullptr)
        mem = scene->get_total_mem_usage();

    std::string memstr = util::string::get_size_string(mem);
    this->memory_label->setText(tr("Memory: %1").arg(memstr.c_str()));
}

void
MainWindow::on_cache_cleanup (void)
{
    mve::Scene::Ptr scene = SceneManager::get().get_scene();
    if (scene == nullptr)
        return;

    scene->cache_cleanup();
    this->on_update_memory();
}

void
MainWindow::closeEvent (QCloseEvent* event)
{
    if (!this->perform_close_scene())
        event->ignore();
    else
        event->accept();
}
