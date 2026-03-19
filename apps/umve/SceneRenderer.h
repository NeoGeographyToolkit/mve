// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#ifndef SFM_SCENE_RENDERER_HEADER
#define SFM_SCENE_RENDERER_HEADER

#include "GlCommon.h"

#include <QAction>
#include <QSlider>

#include "SfmUtils.h"
#include "GlContext.h"
#include "MeshRenderer.h"

#include "GlWidget.h"

// 3D renderer: draws camera frusta, ground plane, viewing direction.
class SceneRenderer : public QWidget, public gl::GlContext
{
    Q_OBJECT

public:
    SceneRenderer (GlWidget* gl_widget);
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
    void load_shaders (void);
    void send_uniform (gl::Camera const& cam);
    void create_frusta_renderer (void);
    void create_ground_renderer (void);
    void create_viewdir_renderer (void);

    GlWidget* gl_widget;
    QOpenGLShaderProgram* wireframe_shader = nullptr;
    sfm::Scene::Ptr scene;
    sfm::View::Ptr view;

    QAction* action_frusta;
    QAction* action_viewdir;
    QAction* action_ground;
    QSlider* frusta_size_slider;
    gl::MeshRenderer::Ptr frusta_renderer;
    gl::MeshRenderer::Ptr ground_renderer;
    gl::MeshRenderer::Ptr viewdir_renderer;

    // Cached original poses (before GL transformation).
    // Cleared on scene change, preserved across slider changes.
    std::vector<math::Vec3d> orig_cam_centers;
    std::vector<math::Matrix3d> orig_cam2world_vec;
};

#endif /* SFM_SCENE_RENDERER_HEADER */
