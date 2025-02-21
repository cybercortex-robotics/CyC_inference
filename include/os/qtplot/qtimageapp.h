
#pragma once

#include <QtWidgets/QWidget>
#include <QtWidgets/QBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGraphicsPixmapItem>
#include <QKeyEvent>
#include <os/qtplot/qtplot.h>
#include <queue>
#include <mutex>

class QTImageAppCustomView : public QGraphicsView
{
    Q_OBJECT

public:
    QTImageAppCustomView(QGraphicsScene* parent = nullptr);
    ~QTImageAppCustomView();
    int get_last_key();

    std::atomic<bool> is_running = true;

protected:
    void keyPressEvent(QKeyEvent* ev) override;
    void closeEvent(QCloseEvent* ev) override;

private:
    std::mutex m_keypress_queue_mtx;
    std::queue<int> m_keypress_queue;
};

class CCcrQTImageApp : public QWidget
{
    Q_OBJECT

public:
    CCcrQTImageApp();
    ~CCcrQTImageApp();

    int get_last_key();
    bool is_running();

signals:
    void should_start(const std::string titlwe);
    void should_set_rgba_pixels(void* data, int width, int height);
    void should_stop();

public slots:
    void start(const std::string title);
    void set_rgba_pixels(void* data, int width, int height);
    void stop();

private:
    QGraphicsScene* m_scene = nullptr;
    QTImageAppCustomView* m_view = nullptr;
    QGraphicsPixmapItem* m_item = nullptr;
    QGraphicsTextItem* m_placeholder_text = nullptr;
};
