/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>

#include "ogl/opengl.h"

#include <QApplication>

#include "util/file_system.h"
#include "util/arguments.h"

#include "mainwindow.h"
#include "guihelpers.h"

struct AppSettings
{
    bool gl_mode;
    bool open_dialog;
    std::vector<std::string> filenames;
};

void
print_help_and_exit (util::Arguments const& args)
{
    args.generate_helptext(std::cerr);
    std::exit(EXIT_FAILURE);
}

int main (int argc, char** argv) {
    /* Parse arguments. */
    util::Arguments args;
    args.set_usage("Syntax: umve [ OPTIONS ] [ FILES | SCENEDIR ]");
    args.set_helptext_indent(14);
    args.set_exit_on_error(true);
    args.add_option('h', "help", false, "Prints this help text and exits");
    args.add_option('o', "open-dialog", "Raises scene open dialog on startup");
    args.add_option('\0', "gl", false, "Switches to GL window on startup");
    args.parse(argc, argv);

    /* Set default startup config. */
    AppSettings conf;
    conf.gl_mode = false;
    conf.open_dialog = false;

    /* Handle arguments.*/
    std::vector<std::string> images, cameras;
    util::ArgResult const* arg;
    while ((arg = args.next_result())) {
        if (arg->opt == nullptr) {
            // Keep track of images and camera files in .tsai format
            std::string file = arg->arg; 
            std::string ext4 = util::string::right(file, 4);
            std::string ext5 = util::string::right(file, 5);
            ext4 = util::string::lowercase(ext4);
            ext5 = util::string::lowercase(ext5);
            if (ext4 == ".png" || ext4 == ".jpg" || ext4 == ".tif" ||
                ext5 == ".jpeg")
                images.push_back(file);
            else if (ext5 == ".tsai")
                cameras.push_back(file);

            // This will also cover the case of input images with no .tsai cameras
            conf.filenames.push_back(arg->arg);
            continue;
        }

        if (arg->opt->lopt == "gl")
            conf.gl_mode = true;
        else if (arg->opt->lopt == "open-dialog")
            conf.open_dialog = true;
        else
            print_help_and_exit(args);
    }

    /* Check if we have as many images as cameras. */
    if (images.size() != cameras.size()) {
        std::cerr << "Number of images and cameras does not match!" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    
    /* Set OpenGL version that Qt should use when creating a context.*/
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
#if defined(_WIN32)
    fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
#else
    fmt.setProfile(QSurfaceFormat::CoreProfile);
#endif
    QSurfaceFormat::setDefaultFormat(fmt);

    /* Create application. */
    set_qt_style("Cleanlooks");
#if defined(_WIN32)
    QCoreApplication::addLibraryPath(QString::fromStdString(
        util::fs::join_path(util::fs::dirname(util::fs::get_binary_path()),
        "qt_plugins")));
#endif
    QApplication app(argc, argv);

    /* Create main window. */
    MainWindow win;

    /* Apply app config. */
    if (conf.gl_mode)
        win.open_scene_inspect();

    bool scene_opened = false;
    
    // The case when we have input images and cameras with .tsai extension
    if (images.size() > 0 && cameras.size() > 0) {
        win.load_scene(images, cameras);
        scene_opened = true;
    } else {
        for (std::size_t i = 0; i < conf.filenames.size(); i++) {
            std::string const& filename = conf.filenames[i];
            if (util::fs::file_exists(filename.c_str())) {
                win.load_file(filename);
            } else if (!scene_opened) {
                win.load_scene(filename);
                scene_opened = true;
            } else {
                std::cerr << "Ignoring extra directory argument: "
                    << filename << std::endl;
                continue;
            }
        }
    }

    if (!scene_opened && conf.open_dialog)
        win.raise_open_scene_dialog();

    return app.exec();
}
