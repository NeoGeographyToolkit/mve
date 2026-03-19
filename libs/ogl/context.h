/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef OGL_CONTEXT_HEADER
#define OGL_CONTEXT_HEADER

#include <algorithm>

#include "ogl/ogl_common.h"
#include "ogl/camera.h"
#include "ogl/camera_trackball.h"

OGL_NAMESPACE_BEGIN

// Rendering context with trackball camera control.
// Subclass and override init_impl, resize_impl, paint_impl.
class Context
{
public:
    virtual ~Context (void) {}

    void init (void);
    void resize (int new_width, int new_height);
    void paint (void);
    bool mouse_event (MouseEvent const& event);

    int get_width (void) const;
    int get_height (void) const;

protected:
    virtual void init_impl (void) = 0;
    virtual void resize_impl (int old_width, int old_height);
    virtual void paint_impl (void) = 0;
    void update_camera (void);

protected:
    Camera camera;
    CamTrackball controller;
    int width = 0;
    int height = 0;
};

/* ---------------------------------------------------------------- */

inline void
Context::init (void)
{
    this->controller.set_camera(&this->camera);
    this->init_impl();
}

inline void
Context::resize (int new_width, int new_height)
{
    std::swap(new_width, this->width);
    std::swap(new_height, this->height);
    this->resize_impl(new_width, new_height);
}

inline void
Context::paint (void)
{
    this->paint_impl();
}

inline bool
Context::mouse_event (MouseEvent const& event)
{
    bool is_handled = this->controller.consume_event(event);
    this->update_camera();
    return is_handled;
}

inline int
Context::get_width (void) const
{
    return this->width;
}

inline int
Context::get_height (void) const
{
    return this->height;
}

inline void
Context::resize_impl (int /*old_width*/, int /*old_height*/)
{
    glViewport(0, 0, this->width, this->height);
    this->camera.width = this->width;
    this->camera.height = this->height;

    float aspect = (float)this->width / (float)this->height;
    float minside = 0.05f;
    if (this->width > this->height) {
        this->camera.top = minside;
        this->camera.right = minside * aspect;
    } else {
        this->camera.top = minside / aspect;
        this->camera.right = minside;
    }

    this->camera.update_proj_mat();
}

inline void
Context::update_camera (void)
{
    this->camera.pos = this->controller.get_campos();
    this->camera.viewing_dir = this->controller.get_viewdir();
    this->camera.up_vec = this->controller.get_upvec();
    this->camera.update_view_mat();
}

OGL_NAMESPACE_END

#endif /* OGL_CONTEXT_HEADER */
