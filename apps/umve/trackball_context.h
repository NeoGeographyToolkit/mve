/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef OGL_TRACKBALL_CONTEXT_HEADER
#define OGL_TRACKBALL_CONTEXT_HEADER

#include <algorithm>

#include "sfm_math.h"
#include "ogl_common.h"

OGL_NAMESPACE_BEGIN

/* ---- Camera ---- */

// Camera with viewing and projection matrices for OpenGL rendering.
class Camera
{
public:
    math::Vec3f pos;
    math::Vec3f viewing_dir;
    math::Vec3f up_vec;

    float z_near;
    float z_far;
    float top;
    float right;

    int width;
    int height;

    math::Matrix4f view;
    math::Matrix4f proj;

public:
    Camera (void);
    void update_view_mat (void);
    void update_proj_mat (void);
};

inline
Camera::Camera (void)
    : pos(0.0f, 0.0f, 5.0f)
    , viewing_dir(0.0f, 0.0f, -1.0f)
    , up_vec(0.0f, 1.0f, 0.0f)
    , z_near(0.1f)
    , z_far(500.0f)
    , top(0.1f)
    , right(0.1f)
    , width(0)
    , height(0)
    , view(0.0f)
    , proj(0.0f)
{
}

inline void
Camera::update_view_mat (void)
{
    this->view = math::matrix_viewtrans(this->pos,
        this->viewing_dir, this->up_vec);
}

inline void
Camera::update_proj_mat (void)
{
    this->proj = math::matrix_gl_projection(this->z_near,
        this->z_far, this->top, this->right);
}

/* ---- Trackball ---- */

// Trackball camera control that consumes mouse events.
class CamTrackball
{
public:
    CamTrackball (void);

    void set_camera (Camera* camera);
    bool consume_event (MouseEvent const& event);

    math::Vec3f get_campos (void) const;
    math::Vec3f get_viewdir (void) const;
    math::Vec3f const& get_upvec (void) const;

private:
    void handle_tb_rotation (int x, int y);
    math::Vec3f get_ball_normal (int x, int y);

private:
    Camera* cam;

    float tb_radius;
    math::Vec3f tb_center;
    math::Vec3f tb_tocam;
    math::Vec3f tb_upvec;

    int rot_mouse_x;
    int rot_mouse_y;
    math::Vec3f rot_tb_tocam;
    math::Vec3f rot_tb_upvec;

    float zoom_tb_radius;
    int zoom_mouse_y;
};

inline void
CamTrackball::set_camera (Camera* camera)
{
    this->cam = camera;
}

inline math::Vec3f
CamTrackball::get_campos (void) const
{
    return this->tb_center + this->tb_tocam * this->tb_radius;
}

inline math::Vec3f
CamTrackball::get_viewdir (void) const
{
    return -this->tb_tocam;
}

inline math::Vec3f const&
CamTrackball::get_upvec (void) const
{
    return this->tb_upvec;
}

/* ---- Context ---- */

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

#endif /* OGL_TRACKBALL_CONTEXT_HEADER */
