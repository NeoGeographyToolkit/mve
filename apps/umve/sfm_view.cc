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

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "ogl/opengl.h"

#include <QApplication>

#include "mainwindow.h"
#include "guihelpers.h"

void print_usage_and_exit () {
    std::cerr << "Usage: sfm_view [OPTIONS] [FILES | SCENEDIR]\n"
              << "Options:\n"
              << "  -h, --help      Print this help and exit\n"
              << "  --width VALUE   Window width in pixels\n"
              << "  --height VALUE  Window height in pixels\n";
    std::exit(EXIT_FAILURE);
}

// Return lowercase file extension including the dot (e.g. ".tsai").
std::string get_file_extension (std::string const& path) {
    std::string::size_type pos = path.rfind('.');
    if (pos == std::string::npos)
        return "";
    std::string ext = path.substr(pos);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return ext;
}

int main (int argc, char** argv) {

    // Must set this before any GUI is created, for OpenGL to work.
    // This variable must not go out of scope till this program exits.
    char MESA_GL_ENV_STR[5012];
    putenv(strcpy(MESA_GL_ENV_STR, "MESA_GL_VERSION_OVERRIDE=3.3"));

    std::vector<std::string> images, cameras;
    int width = 1400, height = 1200; // default window size

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage_and_exit();
        } else if (arg == "--width" && i + 1 < argc) {
            width = std::atoi(argv[++i]);
        } else if (arg == "--height" && i + 1 < argc) {
            height = std::atoi(argv[++i]);
        } else {
            // Classify positional arg as image, camera, or scene dir
            std::string ext = get_file_extension(arg);
            if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" ||
                ext == ".tif" || ext == ".tiff")
                images.push_back(arg);
            else if (ext == ".tsai")
                cameras.push_back(arg);
            // Other files are ignored (only .tsai + image pairs are used)
        }
    }

    if (images.size() != cameras.size()) {
        std::cerr << "Number of images and cameras do not match.\n";
        std::exit(EXIT_FAILURE);
    }

    if (width < 10 || height < 10) {
        std::cerr << "Invalid width or height. Must be at least 10.\n";
        std::exit(EXIT_FAILURE);
    }

    // Set OpenGL version that Qt should use when creating a context.
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(fmt);

    // Create application.
    set_qt_style("Cleanlooks");
    QApplication app(argc, argv);

    // Create main window.
    MainWindow win(width, height);

    // Load images and cameras with .tsai extension
    if (!images.empty() && !cameras.empty())
        win.load_scene(images, cameras);

    return app.exec();
}
