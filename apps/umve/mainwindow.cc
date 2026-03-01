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
#include <QBoxLayout>
#include <QDockWidget>
#include <QMenuBar>
#include <QMessageBox>

#include "scenemanager.h"
#include "mainwindow.h"

MainWindow::MainWindow (int width, int height) {
    this->scene_overview = new SceneOverview(this);

    this->tab_sceneinspect = new SceneInspect(this);

    /* Create dock widgets. */
    this->dock_scene = new QDockWidget(tr("Scene"));
    this->dock_scene->setWidget(this->scene_overview);
    this->dock_scene->setFeatures(QDockWidget::NoDockWidgetFeatures);

    /* Use in-window menu bar to avoid macOS auto-injected menus
       (Hide, Services, Window, etc.). */
    this->menuBar()->setNativeMenuBar(false);

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
    this->resize(width, height);

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
}

void
MainWindow::open_scene_inspect (void)
{
}

void
MainWindow::create_actions (void)
{
    this->action_quit = new QAction(tr("&Quit"), this);
    this->action_quit->setShortcut(tr("Ctrl+Q"));
    this->connect(this->action_quit, SIGNAL(triggered()),
        this, SLOT(close()));

    this->action_about = new QAction(tr("&About"), this);
    this->connect(this->action_about, SIGNAL(triggered()),
        this, SLOT(on_about()));
}

void
MainWindow::create_menus (void)
{
    this->menu_file = new QMenu(tr("&File"), this);
    this->menu_file->addAction(this->action_quit);

    AddinFrustaSceneRenderer* fr =
        this->tab_sceneinspect->get_addin_manager()->get_frusta_renderer();
    this->menu_view = new QMenu(tr("&View"), this);
    this->menu_view->addAction(fr->get_action_frusta());
    this->menu_view->addAction(fr->get_action_viewdir());

    this->menu_help = new QMenu(tr("&Help"), this);
    this->menu_help->addAction(this->action_about);

    this->menuBar()->addMenu(this->menu_file);
    this->menuBar()->addMenu(this->menu_view);
    this->menuBar()->addMenu(this->menu_help);
    this->menuBar()->show();
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

    return true;
}

void
MainWindow::on_about (void)
{
    QMessageBox::about(this, tr("About sfm_view"),
        tr("Camera position viewer for ASP. Based on "
           "<a href=\"https://github.com/simonfuhrmann/mve\">MVE</a>."));
}

void
MainWindow::closeEvent (QCloseEvent* event)
{
    if (!this->perform_close_scene())
        event->ignore();
    else
        event->accept();
}
