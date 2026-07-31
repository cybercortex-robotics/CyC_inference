// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include <os/qtplot/qtimageapp.h>

QTImageAppCustomView::QTImageAppCustomView(QGraphicsScene* parent):
    QGraphicsView(parent)
{
    this->is_running = true;
}

QTImageAppCustomView::~QTImageAppCustomView()
{

}

void QTImageAppCustomView::keyPressEvent(QKeyEvent* ev)
{
    // A key that carries text (letters, digits, return, escape, ...) is queued as that
    // character, which is what callers switch on. The keys that carry none -- the
    // arrows, home/end, the function keys -- have an empty text() and would all queue
    // as the same 0, so they are queued as their Qt key code instead. The two cannot be
    // confused: every Qt::Key_* of a textless key is >= 0x01000000.
    const std::string text = ev->text().toStdString();
    const int key = text.empty() ? ev->key() : (int)text[0];

    m_keypress_queue_mtx.lock();
    m_keypress_queue.push(key);
    m_keypress_queue_mtx.unlock();
}

void QTImageAppCustomView::closeEvent(QCloseEvent* ev)
{
    this->is_running = false;
    ev->accept();
}

int QTImageAppCustomView::get_last_key()
{
    m_keypress_queue_mtx.lock();
    int c = -1;
    if(m_keypress_queue.size())
    {
        c = m_keypress_queue.front();
        m_keypress_queue.pop();
    }
    m_keypress_queue_mtx.unlock();
    return c;
}

CCycQTImageApp::CCycQTImageApp():
    QWidget()
{

}

CCycQTImageApp::~CCycQTImageApp()
{

}

int CCycQTImageApp::get_last_key()
{
    return m_view->get_last_key();
}

bool CCycQTImageApp::is_running()
{
    if(!m_view)
        return false;
    return m_view->is_running;
}

void CCycQTImageApp::start(const std::string title)
{
    m_scene = new QGraphicsScene(this);
    m_view = new QTImageAppCustomView(m_scene);
    m_view->setWindowTitle(title.c_str());

    m_placeholder_text = new QGraphicsTextItem();
    m_placeholder_text->setPlainText("No image has yet been displayed...");
    m_scene->addItem(m_placeholder_text);

    m_item = new QGraphicsPixmapItem();
    m_scene->addItem(m_item);
    
    m_view->show();
}

void CCycQTImageApp::set_rgba_pixels(void* data, int width, int height)
{
    if(!m_item)
        return;

    QImage image((uchar*)data, width, height, QImage::Format_RGBA8888);
    m_item->setPixmap(QPixmap::fromImage(image));
    m_view->adjustSize();
}

void CCycQTImageApp::stop()
{
    m_view->hide();

    if(m_item)
    {
        m_scene->removeItem(m_item);
        delete m_item;
        m_item = nullptr;
    }

    if(m_placeholder_text)
    {
        m_scene->removeItem(m_placeholder_text);
        delete m_placeholder_text;
        m_placeholder_text = nullptr;
    }

    if(m_view)
    {
        delete m_view;
        m_view = nullptr;
    }

    if(m_scene)
    {
        delete m_scene;
        m_scene = nullptr;
    }
}
