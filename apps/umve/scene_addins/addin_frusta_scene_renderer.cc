/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include "guihelpers.h"

#include "scenemanager.h"
#include "scene_addins/addin_frusta_base.h"
#include "scene_addins/addin_frusta_scene_renderer.h"

#include <iostream>

AddinFrustaSceneRenderer::AddinFrustaSceneRenderer (void)
{
    this->render_frusta_cb = new QCheckBox("Draw camera frusta");
    this->render_viewdir_cb = new QCheckBox("Draw viewing direction");
    this->render_frusta_cb->setChecked(true);
    this->render_viewdir_cb->setChecked(true);

    this->frusta_size_slider = new QSlider();
    this->frusta_size_slider->setMinimum(1);
    this->frusta_size_slider->setMaximum(100);
    this->frusta_size_slider->setValue(10);
    this->frusta_size_slider->setOrientation(Qt::Horizontal);

    /* Create frusta rendering layout. */
    this->render_frusta_form = new QFormLayout();
    this->render_frusta_form->setVerticalSpacing(0);
    this->render_frusta_form->addRow(this->render_frusta_cb);
    this->render_frusta_form->addRow(this->render_viewdir_cb);
    this->render_frusta_form->addRow("Frusta Size:", this->frusta_size_slider);

    this->connect(&SceneManager::get(), SIGNAL(scene_bundle_changed()),
        this, SLOT(reset_frusta_renderer()));
    this->connect(&SceneManager::get(), SIGNAL(scene_selected(mve::Scene::Ptr)),
        this, SLOT(reset_frusta_renderer()));
    this->connect(&SceneManager::get(), SIGNAL(view_selected(mve::View::Ptr)),
        this, SLOT(reset_viewdir_renderer()));
    this->connect(this->frusta_size_slider, SIGNAL(valueChanged(int)),
        this, SLOT(reset_frusta_renderer()));
    this->connect(this->frusta_size_slider, SIGNAL(valueChanged(int)),
        this, SLOT(repaint()));
    this->connect(this->render_frusta_cb, SIGNAL(clicked()),
        this, SLOT(repaint()));
    this->connect(this->render_viewdir_cb, SIGNAL(clicked()),
        this, SLOT(repaint()));
}

QWidget*
AddinFrustaSceneRenderer::get_sidebar_widget (void)
{
    return get_wrapper(this->render_frusta_form);
}

void
AddinFrustaSceneRenderer::reset_frusta_renderer (void)
{
    this->frusta_renderer.reset();
}

void
AddinFrustaSceneRenderer::reset_viewdir_renderer (void)
{
    this->viewdir_renderer.reset();
}

void
AddinFrustaSceneRenderer::paint_impl (void)
{
    if (this->render_frusta_cb->isChecked())
    {
        if (this->frusta_renderer == nullptr)
            this->create_frusta_renderer();
        if (this->frusta_renderer != nullptr)
            this->frusta_renderer->draw();
    }

    if (this->render_viewdir_cb->isChecked())
    {
        if (this->viewdir_renderer == nullptr)
            this->create_viewdir_renderer();
        if (this->viewdir_renderer != nullptr)
            this->viewdir_renderer->draw();
    }
}

// TODO(oalexan1): Move this function to utils somewhere.

// Find shortest distance from camera center to the origin
double find_shortest_distance(std::vector<math::Vec3d> centers) {
    // If the vector is empty, return 1.0
    if (centers.size() == 0)
        return 1.0;
    
    double shortest = centers[0].norm();
    for (std::size_t i = 1; i < centers.size(); i++) {
        double dist = centers[i].norm();
        if (dist < shortest)
            shortest = dist;
    }
    return shortest;
}

// Function to find the mean camera position
math::Vec3d find_mean_camera_pos(std::vector<math::Vec3d> const& centers) {
    // If the vector is empty, return 0,0,0
    if (centers.size() == 0)
        return math::Vec3d(0,0,0);
    
    math::Vec3d mean(0,0,0);
    for (std::size_t i = 0; i < centers.size(); i++) {
        mean += centers[i];
    }
    mean /= centers.size();
    return mean;
}

// Write a function to collect the cam2world matrices and camera centers
// in vectors.
void extractCameraPoses(mve::Scene::ViewList const & views,
    // Outputs
    std::vector<math::Vec3d> & cam_centers,
    std::vector<math::Matrix3d> & cam2world_vec) {

    // First wipe the outputs
    cam_centers.clear();
    cam2world_vec.clear();

    for (std::size_t i = 0; i < views.size(); i++) {
    
        if (views[i].get() == nullptr) {
            std::cerr << "Error: Empty camera.\n";
            continue;
        }
        
        //std::cout << "Will adjust the views!" << std::endl;
        mve::CameraInfo const& cam = views[i]->get_camera(); // alias

        // The cameras store trans = -inverse(camera2world) * camera_center
        // and rot = inverse(camera2world).
        // We need to invert the camera2world matrix to get the camera center.
        // So, need to find:
        // and camera2world = transpose(rot)
        // camera_center = -camera2world * trans

        math::Matrix3d world2cam(cam.rot);
        math::Vec3d t(cam.trans);
        math::Matrix3d cam2world = world2cam.transposed();
        math::Vec3d ctr = -cam2world.mult(t);

        // Append to the vector of centers and rotations
        cam_centers.push_back(ctr);
        cam2world_vec.push_back(cam2world);
    }

    return;
}

// Divide all camera centers by shortest distance to planet center
// TODO(oalexan1): This must be a switch. When not having a planet,
// don't do this.
void
AddinFrustaSceneRenderer::create_frusta_renderer (void)
{

std::cout << "Now in create_frusta_renderer" << std::endl;

    if (this->state->scene == nullptr) {
        std::cerr << "Error: No scene loaded.\n";
        return;
    }

    // Find the camera centers and camera2world matrices
    std::vector<math::Vec3d> cam_centers;
    std::vector<math::Matrix3d> cam2world_vec;
    mve::Scene::ViewList & views(this->state->scene->get_views());
    extractCameraPoses(views, cam_centers, cam2world_vec);

    // Find shortest distance from camera center to the origin
    double shortest_dist = find_shortest_distance(cam_centers);
    std::cout << "Shortest distance is " << shortest_dist << std::endl;

    // Divide by the shortest distance, if at least 10.0
    // TODO(oalexan1): Make this into a function.
    // Use a switch to control if to divide or not.
    for (std::size_t i = 0; i < views.size(); i++) {
        if (views[i].get() == nullptr) {
            std::cerr << "Error: Empty camera.\n";
            continue;
        }
        
        //std::cout << "Will adjust the views!" << std::endl;
        mve::CameraInfo cam = views[i]->get_camera(); // make a copy of the camera

        if (shortest_dist > 10.0) 
            cam_centers[i] = cam_centers[i] / shortest_dist;
        
        std::cout << "Camera center is " << cam_centers[i] << std::endl;
        std::cout << "norm - 1.0 is " << cam_centers[i].norm() - 1.0 << std::endl;

        // Update the trans field of the camera
        math::Vec3d t = -cam2world_vec[i].transposed().mult(cam_centers[i]);
        for (int coord = 0; coord < 3; coord++) 
            cam.trans[coord] = t[coord];
        views[i]->set_camera(cam);
        views[i]->set_dirty(false); // so it does not ask to save the scene on quitting
    }

    // Plot the cameras as meshes
    float size = this->frusta_size_slider->value() / 100.0f;
    mve::TriangleMesh::Ptr mesh = mve::TriangleMesh::create();
    for (std::size_t i = 0; i < views.size(); i++) {
        if (views[i].get() == nullptr) {
            std::cerr << "Error: Empty camera.\n";
            continue;
        }

        mve::CameraInfo const& cam = views[i]->get_camera();

        if (cam.flen == 0.0f) {
            std::cerr << "Error: Camera focal length is 0.\n";
            continue;
        }

        add_camera_to_mesh(cam, size, mesh);
    }

    // TODO(oalexan1): Make this into a function.
    mve::TriangleMesh::VertexList& verts(mesh->get_vertices());
    mve::TriangleMesh::FaceList& faces(mesh->get_faces());
    mve::TriangleMesh::ColorList& colors(mesh->get_vertex_colors());
#if 1 // Ground plane
    // TODO(oalexan1): Add a switch for turning this off. This must not
    // show up when a mesh is loaded.
    // Set up a color
    math::Vec4f color;
    color[0] = 0.0f;
    color[1] = 1.0f;
    color[2] = 0.0f;
    color[3] = 1.0f;

    // Find the mean camera center by calling the function defined above
    math::Vec3d mean_cam_center = find_mean_camera_pos(cam_centers);
    std::cout << "Mean camera center is " << mean_cam_center << std::endl;
    std::cout << "Normal - 1.0 is " << mean_cam_center.norm() - 1.0 << std::endl;

    math::Vec3d z = mean_cam_center / mean_cam_center.norm();
    std::cout << "Plane normal is " << z << std::endl;
    // Draw a tiled plane to signify the ground It is perpendicular to the
    // mean camera center and goes through it
    // TODO(oalexan1): Make this into a function

    // Find the index of the largest component of the normal
    int largest_comp = 0;
    for (int i = 1; i < 3; i++) {
        if (std::abs(z[i]) > std::abs(z[largest_comp]))
            largest_comp = i;
    }
    std::cout << "Largest component is " << largest_comp << std::endl;
    // print the largest value
    std::cout << "Largest value is " << z[largest_comp] << std::endl;
    int j = largest_comp + 1;
    if (j == 3) j = 0;
    std::cout << "j is " << j << std::endl;
    math::Vec3d x(0.0, 0.0, 0.0);
    x[j] = z[largest_comp];
    x[largest_comp] = -z[j];
    x = x / x.norm();
    // Swap the j and largest_comp components
    std::cout << "x is " << x << std::endl;
    // print the dot product
    std::cout << "Dot product is " << x.dot(z) << std::endl;
    // Find y as the cross product of plane normal and x
    math::Vec3d y = z.cross(x);
    y = y / y.norm();
    std::cout << "y is " << y << std::endl;
    std::cout << "z is " << z << std::endl;
    // print the dot product of x and y
    std::cout << "Dot product is " << x.dot(y) << std::endl;
    // print the dot product of y and z
    std::cout << "Dot product is " << y.dot(z) << std::endl;
    // print the dot product of x and z
    std::cout << "Dot product is " << x.dot(z) << std::endl;
    // Find z2 as cross product of x and y
    math::Vec3d z2 = x.cross(y);
    std::cout << "z2 is " << z2 << std::endl;
    std::cout << "Dot product is2 " << x.dot(y) << std::endl;
    // print the dot product of y and z
    std::cout << "Dot product is2 " << y.dot(z2) << std::endl;
    // print the dot product of x and z
    std::cout << "Dot product is2 " << x.dot(z2) << std::endl;

    // Find the matrix with x, y, z as columns
    math::Matrix3d R;
    for (int row = 0; row < 3; row++) {
        R(row, 0) = x[row];
        R(row, 1) = y[row];
        R(row, 2) = z[row];
    }
    std::cout << "R is " << R << std::endl;


    // TODO(oalexan1): Make this plane filled and in different color.
    double d = 10.0;
    for (size_t i = 1; i <= 22; i++) {
        double x[2], y[2];
        if (i <= 11) {
            y[0] = 2.0*(i - 6.0)/d; y[1] = y[0];
            x[0] = -1.0; x[1] = 1.0;
        } else {
            x[0] = 2.0*(i - 11 - 6.0)/d; x[1] = x[0];
            y[0] = -1.0; y[1] = 1.0;
        }
    
        // TODO(oalexan1): Replace here by double
        math::Vec3d v1(x[0], y[0], 1.0);
        math::Vec3d v2(x[1], y[1], 1.0);

        verts.push_back(R.mult(v1));
        verts.push_back(R.mult(v2));
        faces.push_back(verts.size() - 2);
        faces.push_back(verts.size() - 1);
        colors.push_back(color);
        colors.push_back(color);
    }
#endif

    this->frusta_renderer = ogl::MeshRenderer::create(mesh);
    this->frusta_renderer->set_shader(this->state->wireframe_shader);
    this->frusta_renderer->set_primitive(GL_LINES);
}

void
AddinFrustaSceneRenderer::create_viewdir_renderer (void)
{
    if (this->state->view == nullptr)
        return;

    math::Vec3f campos, viewdir;
    mve::CameraInfo const& cam(this->state->view->get_camera());
    cam.fill_camera_pos(*campos);
    cam.fill_viewing_direction(*viewdir);

    mve::TriangleMesh::Ptr mesh = mve::TriangleMesh::create();
    mve::TriangleMesh::VertexList& verts = mesh->get_vertices();
    mve::TriangleMesh::ColorList& colors = mesh->get_vertex_colors();
    verts.push_back(campos);
    verts.push_back(campos + viewdir * 100.0f);
    colors.push_back(math::Vec4f(1.0f, 1.0f, 0.0f, 1.0f));
    colors.push_back(math::Vec4f(1.0f, 1.0f, 0.0f, 1.0f));

    this->viewdir_renderer = ogl::MeshRenderer::create(mesh);
    this->viewdir_renderer->set_shader(this->state->wireframe_shader);
    this->viewdir_renderer->set_primitive(GL_LINES);
}
