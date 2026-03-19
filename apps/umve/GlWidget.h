// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#ifndef SFM_GL_WIDGET_HEADER
#define SFM_GL_WIDGET_HEADER

#include <set>
#include <QOpenGLWidget>
#include <QMouseEvent>
#include <QTimer>

#include "GlContext.h"

class GlWidget : public QOpenGLWidget
{
    Q_OBJECT

public:
    GlWidget(QWidget* parent = nullptr);
    ~GlWidget();

    void set_context (gl::GlContext* context);

    QSize minimumSizeHint() const;
    QSize sizeHint() const;

public slots:
    void repaint_async (void);

protected:
    void initializeGL (void);
    void paintGL (void);
    void resizeGL (int width, int height);

    void mousePressEvent (QMouseEvent *event);
    void mouseReleaseEvent (QMouseEvent *event);
    void mouseMoveEvent (QMouseEvent *event);
    void wheelEvent (QWheelEvent* event);

private:
    gl::GlContext* context;
    int gl_width;
    int gl_height;
    qreal device_pixel_ratio;
    bool cx_init;
    std::set<gl::GlContext*> init_set;
    QTimer* repaint_timer;
};

/* ---------------------------------------------------------------- */

inline QSize
GlWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

inline QSize
GlWidget::sizeHint() const
{
    return QSize(400, 400);
}

inline void
GlWidget::repaint_async (void)
{
    /* Don't issue an immediate repaint but let the timer trigger
     * a repaint after all events have been processed. */

    if (this->repaint_timer->isActive())
        return;

    this->repaint_timer->start();
}

#endif /* SFM_GL_WIDGET_HEADER */
