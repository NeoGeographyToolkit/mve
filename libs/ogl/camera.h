/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef OGL_CAMERA_HEADER
#define OGL_CAMERA_HEADER

#include "math/vector.h"
#include "math/matrix.h"
#include "math/matrix_tools.h"
#include "ogl/ogl_common.h"

OGL_NAMESPACE_BEGIN

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

/* ---------------------------------------------------------------- */

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

OGL_NAMESPACE_END

#endif /* OGL_CAMERA_HEADER */
