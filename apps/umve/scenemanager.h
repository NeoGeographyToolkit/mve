/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef UMVE_SCENEMANAGER_HEADER
#define UMVE_SCENEMANAGER_HEADER

#include <QObject>

#include "mve/scene.h"
#include "mve/view.h"

/**
 * The currently active scene as well as the selected view are requried
 * throughout the application. In order to reduce passing the scene
 * through a hierarchy of aggregate objects, the scene, the selected
 * view as well as the selected embedding are managed here. Whenever
 * one of the events happens, the appropriate signal is fired.
 *
 * The selection of the scene is typically handled by the main window.
 * The selection of views is handled by the SceneOverview class.
 * The selection of images is handled by the ViewInspect class.
 */
class SceneManager : public QObject
{
    Q_OBJECT

private:
    mve::Scene::Ptr scene;
    mve::View::Ptr view;

signals:
    void scene_selected (mve::Scene::Ptr scene);
    void view_selected (mve::View::Ptr view);

public:
    SceneManager (void);
    ~SceneManager (void);
    static SceneManager& get (void);

    void select_scene (mve::Scene::Ptr scene);
    void select_view (mve::View::Ptr view);

    mve::Scene::Ptr get_scene (void);
    mve::View::Ptr get_view (void);

    void reset_scene (void);
    void reset_view (void);
};

/* ---------------------------------------------------------------- */

inline void
SceneManager::select_scene (mve::Scene::Ptr scene)
{
    this->scene = scene;
    emit this->scene_selected(scene);
}

inline void
SceneManager::select_view (mve::View::Ptr view)
{
    this->view = view;
    emit this->view_selected(view);
}

inline mve::Scene::Ptr
SceneManager::get_scene (void)
{
    return this->scene;
}

inline mve::View::Ptr
SceneManager::get_view (void)
{
    return this->view;
}

inline void
SceneManager::reset_scene (void)
{
    this->select_scene(mve::Scene::Ptr());
}

inline void
SceneManager::reset_view (void)
{
    this->select_view(mve::View::Ptr());
}

#endif // UMVE_SCENEMANAGER_HEADER
