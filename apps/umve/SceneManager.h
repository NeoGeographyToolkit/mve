// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#ifndef SFM_SCENE_MANAGER_HEADER
#define SFM_SCENE_MANAGER_HEADER

#include <QObject>

#include "SfmUtils.h"

// Singleton signal hub for scene and view selection.
// SfmMainWindow selects the scene, SceneOverview selects views.
class SceneManager: public QObject {
  Q_OBJECT

private:
  sfm::Scene::Ptr scene;
  sfm::View::Ptr view;

signals:
  void scene_selected(sfm::Scene::Ptr scene);
  void view_selected(sfm::View::Ptr view);

public:
  SceneManager(void);
  ~SceneManager(void);
  static SceneManager& get(void);

  void select_scene(sfm::Scene::Ptr scene);
  void select_view(sfm::View::Ptr view);
  sfm::Scene::Ptr get_scene(void);
  void reset_scene(void);
  void reset_view(void);
};

#endif // SFM_SCENE_MANAGER_HEADER
