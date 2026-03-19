/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef SFM_SCENE_OVERVIEW_HEADER
#define SFM_SCENE_OVERVIEW_HEADER

#include <QListWidget>

#include "SfmUtils.h"

class SceneOverview : public QWidget
{
    Q_OBJECT

protected slots:
    void on_scene_changed (sfm::Scene::Ptr scene);
    void on_row_changed (int id);

private:
    void add_view_to_layout (std::size_t id, sfm::View::Ptr view);

private:
    QListWidget* viewlist;

public:
    SceneOverview (QWidget* parent);
    QSize sizeHint (void) const;
};

inline QSize
SceneOverview::sizeHint (void) const
{
    return QSize(175, 0);
}

#endif /* SFM_SCENE_OVERVIEW_HEADER */
