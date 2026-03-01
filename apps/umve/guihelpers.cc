/*
 * Copyright (C) 2015, Simon Fuhrmann
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#include <QApplication>
#include <QFrame>
#include <QLabel>
#include <QStyleFactory>

#include "guihelpers.h"

QWidget*
get_separator (void)
{
    QFrame* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    return line;
}

QWidget*
get_expander (void)
{
    QWidget* expander = new QWidget();
    expander->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    return expander;
}

void
set_qt_style (char const* style_name)
{
    QStyle* style = QStyleFactory::create(style_name);
    if (style != nullptr)
        QApplication::setStyle(style);
}

QWidget*
get_wrapper (QLayout* layout, int margin)
{
    layout->setContentsMargins(margin, margin, margin, margin);
    QWidget* ret = new QWidget();
    ret->setLayout(layout);
    return ret;
}

QCollapsible::QCollapsible (QString title, QWidget* content)
    : content(content)
{
    QLabel* label = new QLabel(title);
    this->collapse_but = new QPushButton("-");
    this->collapse_but->setFlat(true);
    this->collapse_but->setMinimumSize(17, 17);
    this->collapse_but->setMaximumSize(17, 17);

    QHBoxLayout* header_layout = new QHBoxLayout();
    header_layout->setContentsMargins(QMargins(0, 0, 0, 0));
    header_layout->setSpacing(5);
    header_layout->addWidget(collapse_but, 0);
    header_layout->addWidget(get_separator(), 1);
    header_layout->addWidget(label, 0);

    this->content_indent = new QWidget();
    this->content_indent->setFixedSize(0, 0);
    QHBoxLayout* content_layout = new QHBoxLayout();
    content_layout->setContentsMargins(QMargins(0, 0, 0, 0));
    content_layout->setSpacing(0);
    content_layout->addWidget(this->content_indent);
    content_layout->addWidget(this->content);
    this->content_wrapper = new QWidget();
    this->content_wrapper->setLayout(content_layout);

    QVBoxLayout* main_layout = new QVBoxLayout();
    main_layout->setContentsMargins(QMargins(0, 0, 0, 0));
    main_layout->setSpacing(5);
    main_layout->addLayout(header_layout);
    main_layout->addWidget(this->content_wrapper);

    this->setLayout(main_layout);
    this->connect(collapse_but, SIGNAL(clicked()),
        this, SLOT(on_toggle_collapse()));
}

void
QCollapsible::on_toggle_collapse (void)
{
    this->set_collapsed(this->content_wrapper->isVisible() ? true : false);
}

void
QCollapsible::set_collapsed (bool value)
{
    this->content_wrapper->setVisible(!value);
    this->collapse_but->setText(value ? "+" : "-");
}

void
QCollapsible::set_collapsible (bool value)
{
    if (!value)
        this->set_collapsed(false);
    this->collapse_but->setEnabled(value);
}

void
QCollapsible::set_content_indent(int pixels)
{
    this->content_indent->setFixedSize(pixels, 1);
}
