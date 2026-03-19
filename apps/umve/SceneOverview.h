// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#ifndef SFM_SCENE_OVERVIEW_HEADER
#define SFM_SCENE_OVERVIEW_HEADER

#include <QListWidget>

#include "SfmUtils.h"

class SceneOverview: public QWidget {
  Q_OBJECT

protected slots:
  void on_scene_changed(sfm::Scene::Ptr scene);
  void on_row_changed(int id);

private:
  void add_view_to_layout(std::size_t id, sfm::View::Ptr view);
  QListWidget* viewlist;

public:
  SceneOverview(QWidget* parent);
  QSize sizeHint(void) const;
};

#endif // SFM_SCENE_OVERVIEW_HEADER
