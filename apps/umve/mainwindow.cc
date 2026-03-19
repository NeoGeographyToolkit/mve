/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <QApplication>
#include <QBoxLayout>
#include <QDialog>
#include <QDockWidget>
#include <QMenuBar>
#include <QMessageBox>
#include <QSlider>

#include "scenemanager.h"
#include "mainwindow.h"

MainWindow::MainWindow (int width, int height) {
    this->scene_overview = new SceneOverview(this);

    // Create GL widget and scene renderer
    this->gl_widget = new GLWidget();
    this->scene_renderer = new SceneRenderer(this->gl_widget);
    this->gl_widget->set_context(this->scene_renderer);

    this->connect(&SceneManager::get(), SIGNAL(scene_selected(sfm::Scene::Ptr)),
        this, SLOT(on_scene_selected(sfm::Scene::Ptr)));
    this->connect(&SceneManager::get(), SIGNAL(view_selected(sfm::View::Ptr)),
        this, SLOT(on_view_selected(sfm::View::Ptr)));

    // Create dock widgets
    this->dock_scene = new QDockWidget(tr("Cameras"));
    this->dock_scene->setWidget(this->scene_overview);
    this->dock_scene->setFeatures(QDockWidget::NoDockWidgetFeatures);

    // Use in-window menu bar to avoid macOS auto-injected menus
    this->menuBar()->setNativeMenuBar(false);

    this->create_actions();
    this->create_menus();

    QWidget* central_widget(new QWidget(this));
    QLayout* central_layout(new QVBoxLayout(central_widget));
    central_layout->addWidget(this->gl_widget);

    this->setWindowTitle(tr("sfm_view"));
    this->setCentralWidget(central_widget);
    this->addDockWidget(Qt::LeftDockWidgetArea, this->dock_scene);
    this->resize(width, height);

    this->show();
}

void MainWindow::load_scene (std::vector<std::string> const& images,
    std::vector<std::string> const& cameras) {
    sfm::Scene::Ptr scene;
    try
    { scene = sfm::Scene::create(images, cameras); }
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

    this->menu_view = new QMenu(tr("&View"), this);
    this->menu_view->addAction(this->scene_renderer->get_action_frusta());
    this->menu_view->addAction(this->scene_renderer->get_action_ground());
    this->menu_view->addAction(this->scene_renderer->get_action_viewdir());
    QAction* action_frusta_size = new QAction(tr("Set frusta size"), this);
    this->connect(action_frusta_size, SIGNAL(triggered()),
        this, SLOT(on_frusta_size()));
    this->menu_view->addAction(action_frusta_size);

    this->menu_help = new QMenu(tr("&Help"), this);
    this->menu_help->addAction(this->action_about);

    this->menuBar()->addMenu(this->menu_file);
    this->menuBar()->addMenu(this->menu_view);
    this->menuBar()->addMenu(this->menu_help);
    this->menuBar()->show();
}

void
MainWindow::on_scene_selected (sfm::Scene::Ptr scene)
{
    this->scene_renderer->set_scene(scene);
}

void
MainWindow::on_view_selected (sfm::View::Ptr view)
{
    this->scene_renderer->set_view(view);
}

void
MainWindow::perform_close_scene (void)
{
    SceneManager::get().reset_view();
    SceneManager::get().reset_scene();
    this->scene_renderer->reset_scene();
}

void
MainWindow::on_about (void)
{
    QMessageBox::about(this, tr("About sfm_view"),
        tr("Camera pose viewer for ASP. Based on "
           "<a href=\"https://github.com/simonfuhrmann/mve\">MVE</a>."));
}

void
MainWindow::on_frusta_size (void)
{
    QSlider* slider = this->scene_renderer->get_frusta_size_slider();

    QDialog dlg(this);
    dlg.setWindowTitle(tr("Frusta size"));
    QVBoxLayout* layout = new QVBoxLayout(&dlg);
    layout->addWidget(slider);
    dlg.resize(300, 60);
    dlg.exec();

    // Reparent slider back so it survives the dialog destruction
    slider->setParent(nullptr);
}

void
MainWindow::closeEvent (QCloseEvent* event)
{
    this->perform_close_scene();
    event->accept();
}
