/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * Customized for use with ASP by Oleg Alexandrov (2023)
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
    std::vector<std::string> filenames;
};

void
print_help_and_exit (util::Arguments const& args)
{
    args.generate_helptext(std::cerr);
    std::exit(EXIT_FAILURE);
}

int main (int argc, char** argv) {
    util::Arguments args;
    args.set_usage("Syntax: sfm_view [ OPTIONS ] [ FILES | SCENEDIR ]");
    args.set_helptext_indent(14);
    args.set_exit_on_error(true);
    args.add_option('h', "help", false, "Prints this help text and exits");
    args.add_option('\0', "width", true, "Window width in pixels");
    args.add_option('\0', "height", true, "Window height in pixels");
    args.parse(argc, argv);

    // Must set this before any GUI is created, for OpenGL to work.
    // This variable must not go out of scope till this program exits.
    char MESA_GL_ENV_STR[5012];
    putenv(strcpy(MESA_GL_ENV_STR, "MESA_GL_VERSION_OVERRIDE=3.3"));

    /* Set default startup config. */
    AppSettings conf;
    /* Handle arguments.*/
    std::vector<std::string> images, cameras;
    int width = 1400, height = 1200; // default window size
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
                ext5 == ".jpeg" || ext5 == ".tiff")
                images.push_back(file);
            else if (ext5 == ".tsai")
                cameras.push_back(file);

            // This will also cover the case of input images with no .tsai cameras
            conf.filenames.push_back(arg->arg);
            continue;
        }

        if (arg->opt->lopt == "width")
            width = atoi(arg->arg.c_str());
        else if (arg->opt->lopt == "height")
            height = atoi(arg->arg.c_str());
        else
            print_help_and_exit(args);
    }

    /* Check if we have as many images as cameras. */
    if (images.size() != cameras.size()) {
        std::cerr << "Number of images and cameras do not match." << "\n";
        std::exit(EXIT_FAILURE);
    }

    // Sanity check for width and height
    if (width < 10 || height < 10) {
      std::cerr << "Invalid width or height. Must be at least 10." << "\n";
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
    MainWindow win(width, height);

    // Load images and cameras with .tsai extension
    if (images.size() > 0 && cameras.size() > 0)
        win.load_scene(images, cameras);
    else if (!conf.filenames.empty())
        win.load_scene(conf.filenames[0]);

    return app.exec();
}
