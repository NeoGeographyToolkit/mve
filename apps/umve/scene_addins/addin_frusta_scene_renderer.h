/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef UMVE_ADDIN_FRUSTA_SCENE_RENDERER_HEADER
#define UMVE_ADDIN_FRUSTA_SCENE_RENDERER_HEADER

#include <QWidget>
#include <QCheckBox>
#include <QSlider>
#include <QFormLayout>

#include "math/vector.h"
#include "math/matrix.h"
#include "mve/view.h"
#include "ogl/mesh_renderer.h"

#include "scene_addins/addin_base.h"

class AddinFrustaSceneRenderer : public AddinBase
{
    Q_OBJECT

public:
    AddinFrustaSceneRenderer (void);
    QWidget* get_sidebar_widget (void);

protected:
    void create_frusta_renderer (void);
    void create_viewdir_renderer (void);
    void paint_impl (void);

protected slots:
    void reset_viewdir_renderer (void);
    void reset_frusta_renderer (void);
    void on_scene_changed (void);

private:
    QFormLayout* render_frusta_form;
    QCheckBox* render_frusta_cb;
    QCheckBox* render_viewdir_cb;
    QSlider* frusta_size_slider;
    ogl::MeshRenderer::Ptr frusta_renderer;
    ogl::MeshRenderer::Ptr viewdir_renderer;

    // Cached original poses (before GL transformation).
    // Cleared on scene change, preserved across slider changes.
    std::vector<math::Vec3d> orig_cam_centers;
    std::vector<math::Matrix3d> orig_cam2world_vec;
};

#endif /* UMVE_ADDIN_FRUSTA_SCENE_RENDERER_HEADER */
