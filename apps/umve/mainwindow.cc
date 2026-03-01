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
#include <QFileDialog>
#include <QDockWidget>
#include <QMenuBar>
#include <QMessageBox>
#include <QStatusBar>

#include "util/exception.h"
#include "util/file_system.h"

#include "guihelpers.h"
#include "scenemanager.h"
#include "mainwindow.h"

MainWindow::MainWindow (int width, int height) {
    this->scene_overview = new SceneOverview(this);

    /* Populate notebook. */
    this->tab_sceneinspect = new SceneInspect(this);

    this->tabs = new QTabWidget(this);
    this->tabs->addTab(this->tab_sceneinspect, this->tab_sceneinspect->get_title());

    this->memory_label = new QLabel("Memory: <unknown>");
    this->statusbar = new QStatusBar();
    this->statusbar->addWidget(this->memory_label);
    this->setStatusBar(this->statusbar);

    /* Create dock widgets. */
    this->dock_scene = new QDockWidget(tr("Scene"));
    this->dock_scene->setWidget(this->scene_overview);

    /* Create actions and menus. */
    this->create_actions();
    this->create_menus();

    /* Configure window. */
    QWidget* central_widget(new QWidget(this));
    QLayout* central_layout(new QVBoxLayout(central_widget));
    central_layout->addWidget(this->tabs);

    this->setWindowTitle(tr("UMVE - Ultimate Multi-View Environment"));
    this->setWindowIcon(QIcon(":/images/icon_window.png"));
    this->setCentralWidget(central_widget);
    this->addDockWidget(Qt::LeftDockWidgetArea, this->dock_scene);
    this->enable_scene_actions(false);
    this->resize(width, height);
    
    /* Start update timer and init. */
    this->update_timer = new QTimer(this);
    this->update_timer->start(2000); // Every sec
    this->on_update_memory();

    /* Connect signals. */
    this->connect(this->update_timer, SIGNAL(timeout()),
        this, SLOT(on_update_memory()));
    this->connect(this->tabs, SIGNAL(currentChanged(int)),
        this, SLOT(on_switch_tabs(int)));

    this->show();

    this->on_switch_tabs(0);
}

/* ---------------------------------------------------------------- */

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

/* ---------------------------------------------------------------- */

void
MainWindow::open_scene_inspect (void)
{
    this->tabs->setCurrentIndex(0);
}

/* ---------------------------------------------------------------- */

void
MainWindow::create_actions (void)
{
    this->action_new_scene = new QAction(QIcon(":/images/icon_new_dir.svg"),
        tr("&New scene..."), this);
    this->connect(this->action_new_scene, SIGNAL(triggered()),
        this, SLOT(on_new_scene()));

    this->action_open_scene = new QAction(QIcon(":/images/icon_open_file.svg"),
        tr("&Open scene..."), this);
    //this->action_open_scene->setShortcut(tr("Ctrl+O"));
    this->connect(this->action_open_scene, SIGNAL(triggered()),
        this, SLOT(raise_open_scene_dialog()));

    this->action_reload_scene = new QAction(QIcon(":/images/icon_revert.svg"),
        tr("&Reload scene"), this);
    //this->action_reload_scene->setShortcut(tr("Ctrl+R"));
    this->connect(this->action_reload_scene, SIGNAL(triggered()),
        this, SLOT(on_reload_scene()));

    this->action_save_scene = new QAction(QIcon(":/images/icon_save.svg"),
        tr("Save scene"), this);
    this->action_save_scene->setShortcut(tr("Ctrl+S"));
    this->connect(this->action_save_scene, SIGNAL(triggered()),
        this, SLOT(on_save_scene()));

    this->action_close_scene = new QAction(QIcon(":/images/icon_close.svg"),
        tr("Close scene"), this);
    this->connect(this->action_close_scene, SIGNAL(triggered()),
        this, SLOT(on_close_scene()));

    this->action_import_images = new QAction(
        QIcon(":/images/icon_new_file.svg"), tr("Import Images..."), this);
    this->connect(this->action_import_images, SIGNAL(triggered()),
        this, SLOT(on_import_images()));

    this->action_recon_export = new QAction(QIcon(":/images/icon_export.svg"),
        tr("Export reconstruction..."), this);
    this->connect(this->action_recon_export, SIGNAL(triggered()),
        this, SLOT(on_recon_export()));

    this->action_batch_delete = new QAction(QIcon(":/images/icon_delete.svg"),
        tr("Delete embeddings..."), this);
    this->connect(this->action_batch_delete, SIGNAL(triggered()),
        this, SLOT(on_batch_delete()));

    this->action_generate_thumbs = new QAction(
        QIcon(":/images/icon_image_inspect.svg"),
        tr("Generate thumbmails..."), this);
    this->connect(this->action_generate_thumbs, SIGNAL(triggered()),
        this, SLOT(on_generate_thumbs()));

    this->action_cache_cleanup = new QAction(QIcon(":/images/icon_clean.svg"),
        tr("Cache cleanup"), this);
    this->connect(this->action_cache_cleanup, SIGNAL(triggered()),
        this, SLOT(on_cache_cleanup()));

    this->action_refresh_scene = new QAction(QIcon(":/images/icon_refresh.svg"),
        tr("Refresh scene"), this);
    this->connect(this->action_refresh_scene, SIGNAL(triggered()),
        this, SLOT(on_refresh_scene()));

    this->action_exit = new QAction(QIcon(":/images/icon_exit.svg"),
        tr("E&xit"), this);
    this->action_exit->setShortcut(tr("Ctrl+Q"));
    this->connect(this->action_exit, SIGNAL(triggered()),
        this, SLOT(close()));

    this->action_about = new QAction(QIcon(":/images/icon_about.svg"),
        tr("&About"), this);
    this->connect(this->action_about, SIGNAL(triggered()),
        this, SLOT(on_about()));

    this->action_about_qt = new QAction(QIcon(":/images/icon_about.svg"),
        tr("About &Qt"), this);
    this->connect(this->action_about_qt, SIGNAL(triggered()),
        qApp, SLOT(aboutQt()));
}

/* ---------------------------------------------------------------- */

void
MainWindow::create_menus (void)
{
    this->menu_scene = new QMenu(tr("&Scene"), this);
    this->menu_scene->addAction(this->action_new_scene);
    this->menu_scene->addAction(this->action_open_scene);
    this->menu_scene->addAction(this->action_reload_scene);
    this->menu_scene->addAction(this->action_save_scene);
    this->menu_scene->addAction(this->action_close_scene);
    this->menu_scene->addSeparator();
    this->menu_scene->addAction(this->action_import_images);
    this->menu_scene->addAction(this->action_recon_export);
    this->menu_scene->addAction(this->action_batch_delete);
    this->menu_scene->addAction(this->action_generate_thumbs);
    this->menu_scene->addAction(this->action_cache_cleanup);
    this->menu_scene->addSeparator();
    this->menu_scene->addAction(this->action_exit);

    this->menu_help = new QMenu(tr("&Help"), this);
    this->menu_help->addAction(this->action_about);
    this->menu_help->addAction(this->action_about_qt);

    this->menuBar()->addMenu(this->menu_scene);
    this->menuBar()->addMenu(this->menu_help);
    this->menuBar()->show();

    this->scene_overview->add_toolbar_action(this->action_open_scene);
    this->scene_overview->add_toolbar_action(this->action_reload_scene);
    this->scene_overview->add_toolbar_action(this->action_save_scene);
    this->scene_overview->add_toolbar_action(this->action_close_scene);
    this->scene_overview->add_toolbar_action(this->action_cache_cleanup);
    this->scene_overview->add_toolbar_action(this->action_refresh_scene);
}

/* ---------------------------------------------------------------- */

void
MainWindow::raise_open_scene_dialog (void)
{
    QString dirname = QFileDialog::getExistingDirectory(this,
        tr("Open scene"), QDir::currentPath());
    if (dirname.isEmpty())
        return;
    if (!this->perform_close_scene())
        return;

    this->load_scene(dirname.toStdString());
}

/* ---------------------------------------------------------------- */

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

/* ---------------------------------------------------------------- */

void
MainWindow::enable_scene_actions (bool value)
{
    this->action_reload_scene->setEnabled(value);
    this->action_save_scene->setEnabled(value);
    this->action_close_scene->setEnabled(value);
    this->action_import_images->setEnabled(value);
    this->action_recon_export->setEnabled(value);
    this->action_batch_delete->setEnabled(value);
    this->action_generate_thumbs->setEnabled(value);
    this->action_cache_cleanup->setEnabled(value);
    this->action_refresh_scene->setEnabled(value);
}

/* ---------------------------------------------------------------- */

void
MainWindow::on_new_scene (void)
{
    if (!this->perform_close_scene())
        return;

    QString dirname = QFileDialog::getExistingDirectory(this,
        "Select scene directory", QDir::currentPath());
    if (dirname.isEmpty())
        return;

    std::string scene_path = dirname.toStdString();
    std::string views_path = scene_path + "/views";
    if (util::fs::dir_exists(views_path.c_str())
        || util::fs::file_exists(views_path.c_str()))
    {
        QMessageBox::information(this, "Error creating scene",
            "Another <i>views/</i> directory or file already exists!");
        return;
    }
    if (!util::fs::mkdir(views_path.c_str()))
    {
        QMessageBox::information(this, "Error creating scene",
            "The <i>views/</i> directory could not be created!");
        return;
    }

    this->load_scene(scene_path);
    QMessageBox::information(this, "Scene created!",
        "The scene has been created! Now import some images...");
}

/* ---------------------------------------------------------------- */

void
MainWindow::on_reload_scene (void)
{
    mve::Scene::Ptr scene = SceneManager::get().get_scene();

    if (scene == nullptr || scene->get_path().empty())
    {
        QMessageBox::information(this, "Error reloading scene!",
            "There is nothing to reload, rookie.");
        return;
    }

    std::string scene_path = scene->get_path();
    if (!this->perform_close_scene())
        return;

    this->load_scene(scene_path);
}

/* ---------------------------------------------------------------- */

void
MainWindow::on_save_scene (void)
{
    mve::Scene::Ptr scene = SceneManager::get().get_scene();

    if (scene == nullptr || scene->get_path().empty())
    {
        QMessageBox::information(this, "Error saving scene!",
            "There is nothing to save, rookie.");
        return;
    }

    try
    {
        scene->save_scene();
    }
    catch (std::exception& e)
    {
        std::cout << "Error saving scene: " << e.what() << std::endl;
        QMessageBox::critical(this, "Error saving scene!",
            tr("Error saving scene:\n%1").arg(e.what()));
    }
}

/* ---------------------------------------------------------------- */

void
MainWindow::on_close_scene (void)
{
    this->perform_close_scene();
}

/* ---------------------------------------------------------------- */

void
MainWindow::on_refresh_scene (void)
{
    SceneManager::get().refresh_scene();
}

/* ---------------------------------------------------------------- */

void
MainWindow::on_about (void)
{
    QMessageBox::about(this, tr("About UMVE"),
        tr("UMVE is the Ultimate Multi-View Environment."));
}

/* ---------------------------------------------------------------- */

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

/* ---------------------------------------------------------------- */

// Batch operations stubs - to be removed with menu cleanup
void MainWindow::on_import_images (void) {}
void MainWindow::on_recon_export (void) {}
void MainWindow::on_batch_delete (void) {}
void MainWindow::on_generate_thumbs (void) {}

/* ---------------------------------------------------------------- */

void
MainWindow::on_cache_cleanup (void)
{
    mve::Scene::Ptr scene = SceneManager::get().get_scene();
    if (scene == nullptr)
        return;

    scene->cache_cleanup();
    this->on_update_memory();
}

/* ---------------------------------------------------------------- */

void
MainWindow::on_switch_tabs (int tab_id)
{
    for (int i = 0; i < this->tabs->count(); ++i)
    {
        QWidget *tab = this->tabs->widget(i);
        dynamic_cast<MainWindowTab&>(*tab).set_tab_active(i == tab_id);
    }
}

/* ---------------------------------------------------------------- */

void
MainWindow::closeEvent (QCloseEvent* event)
{

    if (!this->perform_close_scene())
        event->ignore();
    else
        event->accept();
}
