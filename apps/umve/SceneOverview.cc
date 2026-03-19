/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <QVBoxLayout>

#include "SceneManager.h"
#include "SceneOverview.h"

SceneOverview::SceneOverview (QWidget* parent)
    : QWidget(parent)
{
    this->viewlist = new QListWidget();
    this->viewlist->setEnabled(false);

    QVBoxLayout* vbox = new QVBoxLayout(this);
    vbox->setSpacing(0);
    vbox->setContentsMargins(0, 0, 0, 0);
    vbox->addWidget(this->viewlist);

    this->connect(this->viewlist, SIGNAL(currentRowChanged(int)),
        this, SLOT(on_row_changed(int)));
    this->connect(&SceneManager::get(), SIGNAL(scene_selected(sfm::Scene::Ptr)),
        this, SLOT(on_scene_changed(sfm::Scene::Ptr)));
}

void
SceneOverview::on_scene_changed (sfm::Scene::Ptr scene)
{
    this->viewlist->clear();
    this->viewlist->setEnabled(false);

    if (scene == nullptr)
        return;

    sfm::Scene::ViewList& sl(scene->get_views());
    if (sl.empty()) {
        QListWidgetItem* item = new QListWidgetItem("Scene has no views!");
        item->setData(Qt::UserRole, -1);
        this->viewlist->addItem(item);
    } else {
        this->viewlist->setEnabled(true);
    }

    for (std::size_t i = 0; i < sl.size(); ++i) {
        sfm::View::Ptr view(sl[i]);
        if (view == nullptr)
            continue;
        this->add_view_to_layout(i, view);
    }
}

void
SceneOverview::add_view_to_layout (std::size_t id, sfm::View::Ptr view)
{
    if (view == nullptr)
        return;

    std::string const& view_name = view->get_name();
    bool cam_valid = view->get_camera().flen != 0.0f;

    QString name = QString(view_name.c_str());

    QListWidgetItem* item = new QListWidgetItem(name);
    if (!cam_valid)
        item->setBackgroundColor(QColor(255, 221, 221));
    item->setData(Qt::UserRole, (int)id);
    this->viewlist->addItem(item);
}

void
SceneOverview::on_row_changed (int id)
{
    if (id < 0)
        return;

    QListWidgetItem* item = this->viewlist->item(id);
    std::size_t view_id = (std::size_t)item->data(Qt::UserRole).toInt();

    sfm::Scene::Ptr scene(SceneManager::get().get_scene());
    sfm::View::Ptr view(scene->get_view_by_id(view_id));
    SceneManager::get().select_view(view);
}
