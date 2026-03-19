// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#include <iostream>

#include "GlContext.h"

GL_NAMESPACE_BEGIN

CamTrackball::CamTrackball (void)
{
    this->cam = nullptr;
    this->tb_radius = 1.0f;
    this->tb_center = math::Vec3f(0.0f);
    this->tb_tocam = math::Vec3f(0.0f, 0.0f, 1.0f);
    this->tb_upvec = math::Vec3f(0.0f, 1.0f, 0.0f);
}

/* ---------------------------------------------------------------- */

bool
CamTrackball::consume_event (MouseEvent const& event)
{
    bool is_handled = false;

    if (event.type == MOUSE_EVENT_PRESS)
    {
        if (event.button == MOUSE_BUTTON_LEFT)
        {
            this->rot_mouse_x = event.x;
            this->rot_mouse_y = event.y;
            this->rot_tb_tocam = this->tb_tocam;
            this->rot_tb_upvec = this->tb_upvec;
        }
        else if (event.button == MOUSE_BUTTON_MIDDLE)
        {
            this->zoom_mouse_y = event.y;
            this->zoom_tb_radius = this->tb_radius;
        }
        is_handled = true;
    }
    else if (event.type == MOUSE_EVENT_MOVE)
    {
        if (event.button_mask & MOUSE_BUTTON_LEFT)
        {
            if (event.x == this->rot_mouse_x && event.y == this->rot_mouse_y)
            {
                this->tb_tocam = this->rot_tb_tocam;
                this->tb_upvec = this->rot_tb_upvec;
            }
            else
            {
                this->handle_tb_rotation(event.x, event.y);
            }
            is_handled = true;
        }

        if (event.button_mask & MOUSE_BUTTON_MIDDLE)
        {
            int mouse_diff = this->zoom_mouse_y - event.y;
            float zoom_speed = this->zoom_tb_radius / 100.0f;
            float cam_diff = (float)mouse_diff * zoom_speed;
            float new_radius = this->zoom_tb_radius + cam_diff;
            this->tb_radius = math::clamp(new_radius, this->cam->z_near, this->cam->z_far);
            is_handled = true;
        }
    }
    else if (event.type == MOUSE_EVENT_WHEEL_UP)
    {
        this->tb_radius = this->tb_radius + this->tb_radius / 10.0f;
        this->tb_radius = std::min(this->cam->z_far, this->tb_radius);
        is_handled = true;
    }
    else if (event.type == MOUSE_EVENT_WHEEL_DOWN)
    {
        this->tb_radius = this->tb_radius - this->tb_radius / 10.0f;
        this->tb_radius = std::max(this->cam->z_near, this->tb_radius);
        is_handled = true;
    }

    return is_handled;
}

/* ---------------------------------------------------------------- */

void
CamTrackball::handle_tb_rotation (int x, int y)
{
    /* Get ball normals. */
    math::Vec3f bn_start = this->get_ball_normal
        (this->rot_mouse_x, this->rot_mouse_y);
    math::Vec3f bn_now = this->get_ball_normal(x, y);

    /* Rotation axis and angle. */
    math::Vec3f axis = bn_now.cross(bn_start);
    float angle = std::acos(bn_now.dot(bn_start));

    /* Rotate axis to world coords. Build inverse viewing matrix from
     * values stored at the time of mouse click.
     */
    math::Matrix4f cam_to_world;
    math::Vec3f campos = this->tb_center + this->rot_tb_tocam * this->tb_radius;
    math::Vec3f viewdir = -this->rot_tb_tocam;
    cam_to_world = math::matrix_inverse_viewtrans(
        campos, viewdir, this->rot_tb_upvec);
    axis = cam_to_world.mult(axis, 0.0f);
    axis.normalize();

    /* Rotate camera and up vector around axis. */
    math::Matrix3f rot = matrix_rotation_from_axis_angle(axis, angle);
    this->tb_tocam = rot * this->rot_tb_tocam;
    this->tb_upvec = rot * this->rot_tb_upvec;
}

/* ---------------------------------------------------------------- */

math::Vec3f
CamTrackball::get_ball_normal (int x, int y)
{
    /* Calculate normal on unit sphere. */
    math::Vec3f sn;
    sn[0] = 2.0f * (float)x / (float)(this->cam->width - 1) - 1.0f;
    sn[1] = 1.0f - 2.0f * (float)y / (float)(this->cam->height - 1);
    float z2 = 1.0f - sn[0] * sn[0] - sn[1] * sn[1];
    sn[2] = z2 > 0.0f ? std::sqrt(z2) : 0.0f;

    return sn.normalize();
}

/* ---------------------------------------------------------------- */

GL_NAMESPACE_END
