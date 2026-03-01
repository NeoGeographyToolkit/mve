/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef GUI_HELPERS_HEADER
#define GUI_HELPERS_HEADER

#include <QLayout>
#include <QPushButton>

QWidget* get_separator (void);
QWidget* get_expander (void);
QWidget* get_wrapper (QLayout* layout, int margin = 0);

void set_qt_style (char const* style_name);

/** Widget that displays a header and collapsible content. */
class QCollapsible : public QWidget
{
    Q_OBJECT

private:
    QPushButton* collapse_but;
    QWidget* content;
    QWidget* content_indent;
    QWidget* content_wrapper;

private slots:
    void on_toggle_collapse (void);

public:
    QCollapsible (QString title, QWidget* content);
    void set_collapsed (bool value);
    void set_collapsible (bool value);
    void set_content_indent (int pixels);
};

#endif /* GUI_HELPERS_HEADER */
