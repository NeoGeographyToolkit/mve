// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#ifndef GL_CONTEXT_HEADER
#define GL_CONTEXT_HEADER

#include <algorithm>

#include <QOpenGLFunctions_3_3_Core>

#include "SfmMath.h"
#include "GlCommon.h"

GL_NAMESPACE_BEGIN

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
class GlContext
{
public:
    virtual ~GlContext (void) {}

    void init (void);
    void resize (int new_width, int new_height);
    void paint (void);
    bool mouse_event (MouseEvent const& event);
    void set_gl_functions (QOpenGLFunctions_3_3_Core* f);

protected:
    virtual void init_impl (void) = 0;
    virtual void resize_impl (int old_width, int old_height);
    virtual void paint_impl (void) = 0;
    void update_camera (void);

protected:
    Camera camera;
    CamTrackball controller;
    QOpenGLFunctions_3_3_Core* gl = nullptr;
    int width = 0;
    int height = 0;
};

inline void
GlContext::init (void)
{
    this->controller.set_camera(&this->camera);
    this->init_impl();
}

inline void
GlContext::resize (int new_width, int new_height)
{
    std::swap(new_width, this->width);
    std::swap(new_height, this->height);
    this->resize_impl(new_width, new_height);
}

inline void
GlContext::paint (void)
{
    this->paint_impl();
}

inline bool
GlContext::mouse_event (MouseEvent const& event)
{
    bool is_handled = this->controller.consume_event(event);
    this->update_camera();
    return is_handled;
}

inline void
GlContext::resize_impl (int /*old_width*/, int /*old_height*/)
{
    this->gl->glViewport(0, 0, this->width, this->height);
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
GlContext::set_gl_functions (QOpenGLFunctions_3_3_Core* f)
{
    this->gl = f;
}

inline void
GlContext::update_camera (void)
{
    this->camera.pos = this->controller.get_campos();
    this->camera.viewing_dir = this->controller.get_viewdir();
    this->camera.up_vec = this->controller.get_upvec();
    this->camera.update_view_mat();
}

GL_NAMESPACE_END

#endif /* GL_CONTEXT_HEADER */
