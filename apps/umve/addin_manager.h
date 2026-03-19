/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef UMVE_SCENE_ADDIN_MANAGER_HEADER
#define UMVE_SCENE_ADDIN_MANAGER_HEADER

#include "ogl_common.h"

#include <QAction>
#include <QSlider>

#include "sfm_view_utils.h"
#include "trackball_context.h"
#include "mesh_renderer.h"

#include "glwidget.h"
#include "addin_state.h"

class AddinManager : public QWidget, public ogl::Context
{
    Q_OBJECT

public:
    AddinManager (GLWidget* gl_widget);
    void set_scene (sfm::Scene::Ptr scene);
    void set_view (sfm::View::Ptr view);
    void reset_scene (void);

    QAction* get_action_frusta (void);
    QAction* get_action_viewdir (void);
    QAction* get_action_ground (void);
    QSlider* get_frusta_size_slider (void);

protected:
    void init_impl (void);
    void resize_impl (int old_width, int old_height);
    void paint_impl (void);

private slots:
    void reset_viewdir_renderer (void);
    void reset_frusta_renderer (void);
    void reset_ground_renderer (void);
    void on_scene_changed (void);

private:
    void create_frusta_renderer (void);
    void create_ground_renderer (void);
    void create_viewdir_renderer (void);

    AddinState state;
    QAction* action_frusta;
    QAction* action_viewdir;
    QAction* action_ground;
    QSlider* frusta_size_slider;
    ogl::MeshRenderer::Ptr frusta_renderer;
    ogl::MeshRenderer::Ptr ground_renderer;
    ogl::MeshRenderer::Ptr viewdir_renderer;

    // Cached original poses (before GL transformation).
    // Cleared on scene change, preserved across slider changes.
    std::vector<math::Vec3d> orig_cam_centers;
    std::vector<math::Matrix3d> orig_cam2world_vec;
};

#endif /* UMVE_SCENE_ADDIN_MANAGER_HEADER */
