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

#include "ogl/opengl.h"

#include <string>
#include <vector>

#include "mve/scene.h"
#include "mve/view.h"
#include "ogl/context.h"
#include "ogl/camera_trackball.h"

#include "glwidget.h"
#include "scene_addins/addin_base.h"
#include "scene_addins/addin_frusta_scene_renderer.h"

/*
 * The addin manager sets up the basic OpenGL context, creates the shaders
 * and refers rendering to a set of addins.
 */
class AddinManager : public QWidget, public ogl::CameraTrackballContext
{
    Q_OBJECT

public:
    AddinManager (GLWidget* gl_widget, QWidget* sidebar);
    virtual ~AddinManager (void) {}
    void set_scene (mve::Scene::Ptr scene);
    void set_view (mve::View::Ptr view);

    void reset_scene (void);

    virtual bool keyboard_event (ogl::KeyboardEvent const& event);
    virtual bool mouse_event (ogl::MouseEvent const& event);

protected:
    void init_impl (void);
    void resize_impl (int old_width, int old_height);
    void paint_impl (void);

private:
    AddinState state;
    std::vector<AddinBase*> addins;

    /* Addins. */
    AddinFrustaSceneRenderer* frusta_renderer;
};

#endif /* UMVE_SCENE_ADDIN_MANAGER_HEADER */
