// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include <os/qtplot/qtplot.h>
#include <os/qtplot/qtimage.h>
#include <os/qtplot/qtplotsegment.h>
#include <os/qtplot/qtplotapp.h>
#include <os/qtplot/qtimageapp.h>
#include <QtWidgets/QApplication>
#include <QtCore/QEvent>

static int qtplot_qtapp_argc = 1;
const char* qtplot_qtapp_argv[] = { "qtplot" };

struct RegisteredImageDisplayApp
{
    std::unique_ptr<CCycQTImageApp> app;
    std::string title;
    bool running = false;
};

CCycQTSkeleton::CCycQTSkeleton(CSingletonRegistry* _singleton_registry) :
    m_qtapp(nullptr)
{
    init();
}

CCycQTSkeleton::~CCycQTSkeleton()
{
    destroy();
}

//CCycQTSkeleton& CCycQTSkeleton::instance()
//{
//    static CCycQTSkeleton inst;
//    return inst;
//}

CCycQTSkeleton::CCcrQTPlotAppPtr CCycQTSkeleton::createPlotApp()
{
    return createQtObject<CCcrQTPlotApp>(this);
}

CCycQTSkeleton::RegisteredImageDisplayAppPtr CCycQTSkeleton::createDisplayApp()
{
    auto ptr = std::make_unique<RegisteredImageDisplayApp>();
    ptr->app = createQtObject<CCycQTImageApp>(this);

    return std::move(ptr);
}

void CCycQTSkeleton::init()
{
    std::call_once(m_init_flag, [this]() {
        std::promise<void> initialized_promise;
        std::future<void> initialized_future = initialized_promise.get_future();
        m_qtplot_thread = std::thread([this, &initialized_promise]() {
            m_qtapp = std::make_unique<QApplication>(qtplot_qtapp_argc, const_cast<char**>(qtplot_qtapp_argv));
            QCoreApplication::setApplicationName("QtPlot");

            // For passing arguments to signals.
            qRegisterMetaType<std::vector<float>>("std::vector<float>");
            qRegisterMetaType<std::vector<CCcrQTPlotSegmentTemp>>("std::vector<CCcrQTPlotSegmentTemp>");
            qRegisterMetaType<std::string>("std::string");

            m_qtapp->setQuitOnLastWindowClosed(false);

            initialized_promise.set_value();
            m_qtapp->exec();

            m_qtapp.reset(nullptr);
        });

        initialized_future.wait();
        m_qt_initialized = true;
    });
}

void CCycQTSkeleton::destroy()
{
    if (!m_qt_initialized.exchange(false))
        return;

    {
        std::scoped_lock lock{m_plotapp_mutex};
        for (auto& app : m_plotapps) {
            if (app)
                emit app->should_stop();
        }
        m_plotapps.clear();
    }

    {
        std::scoped_lock lock{m_displayapp_mutex};
        for (auto& disp : m_display_apps) {
            if (disp && disp->app)
                emit disp->app->should_stop();
        }
        m_display_apps.clear();
    }

    if (auto* app = QCoreApplication::instance()) {
        QMetaObject::invokeMethod(
            app,
            [] {
                QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
                QCoreApplication::processEvents();
                QCoreApplication::quit();
            },
            Qt::QueuedConnection
        );
    }

    if (m_qtplot_thread.joinable())
    {
        m_qtplot_thread.join();
    }
}

//void CCycQTSkeleton::assert_running()
//{
//    if (!m_qt_initialized.load())
//    {
//        throw std::runtime_error{ "QtPlot is not initialized. Call CCycQTSkeleton::instance().init() in the main thread before plotting. "};
//    }
//}

CCcrQTPlotApp* CCycQTSkeleton::allocate_plotapp()
{
    if (!this->is_running())
        return nullptr;

    std::scoped_lock lock{ m_plotapp_mutex };
    m_plotapps.emplace_back(createPlotApp());

    auto* ptr = m_plotapps.back().get();

    QObject::connect(ptr, &CCcrQTPlotApp::should_start, ptr, &CCcrQTPlotApp::start);
    QObject::connect(ptr, &CCcrQTPlotApp::should_plot_signals, ptr, &CCcrQTPlotApp::plot_signals);
    QObject::connect(ptr, &CCcrQTPlotApp::should_stop, ptr, &CCcrQTPlotApp::stop);

    return ptr;
}

void CCycQTSkeleton::deallocate_plotapp(CCcrQTPlotApp* plotapp)
{
    std::scoped_lock lock{ m_plotapp_mutex };
    for (auto it = m_plotapps.begin(); it != m_plotapps.end(); ++it)
    {
        if (it->get() == plotapp)
        {
            m_plotapps.erase(it);
            break;
        }
    }
}

RegisteredImageDisplayApp* CCycQTSkeleton::find_display_app(const std::string& title) const
{
    std::scoped_lock lock{ m_displayapp_mutex };
    for (const auto& app : m_display_apps)
    {
        if (app->title == title && app->running)
        {
            return app.get();
        }
    }

    return nullptr;
}

RegisteredImageDisplayApp* CCycQTSkeleton::allocate_display_app(const std::string& title)
{
    if (!this->is_running())
        return nullptr;

    std::scoped_lock lock{ m_displayapp_mutex };
    m_display_apps.emplace_back(createDisplayApp());

    auto* ptr = m_display_apps.back().get();

    CCycQTImageApp* disp_app = ptr->app.get();
    QObject::connect(disp_app, &CCycQTImageApp::should_start, disp_app, &CCycQTImageApp::start, Qt::BlockingQueuedConnection);
    QObject::connect(disp_app, &CCycQTImageApp::should_set_rgba_pixels, disp_app, &CCycQTImageApp::set_rgba_pixels, Qt::BlockingQueuedConnection);
    QObject::connect(disp_app, &CCycQTImageApp::should_stop, disp_app, &CCycQTImageApp::stop, Qt::BlockingQueuedConnection);

    ptr->title = title;
    ptr->running = true;

    emit ptr->app->should_start(title);
    return ptr;
}

CCcrQTPlot::CCcrQTPlot(const std::string& _title, CCycQTSkeleton* _qt_skeleton) :
    m_title(_title),
    m_CcrQTSkeleton(_qt_skeleton)
{
    //CCycQTSkeleton::instance().init();
}

CCcrQTPlot::~CCcrQTPlot()
{
}

int CCcrQTPlot::run()
{
    //CCycQTSkeleton::instance().assert_running();
    if (!m_CcrQTSkeleton->is_running())
        return -1;

    m_plotapp = m_CcrQTSkeleton->allocate_plotapp();
    if (m_plotapp == nullptr)
        return -1;

    emit m_plotapp->should_start(m_temp_segments, m_title);
    return 0;
}

bool CCcrQTPlot::stop()
{
    if(m_plotapp == nullptr)
        return false;

    emit m_plotapp->should_stop();

    m_CcrQTSkeleton.get()->deallocate_plotapp(m_plotapp);
    m_plotapp = nullptr;
    return true;
}

void CCcrQTPlot::add_segment(const std::string& name, const std::vector<std::string> &sigs, const std::string& unit)
{
    m_temp_segments.emplace_back(name, sigs, unit);
}

void CCcrQTPlot::plot_signals(const std::vector<float>& sigs)
{
    if(m_plotapp == nullptr)
        return;

    emit m_plotapp->should_plot_signals(sigs);
}


CCycQTImage::CCycQTImage(CCycQTSkeleton* _qt_skeleton) :
    m_CcrQTSkeleton(_qt_skeleton)
{}

CCycQTImage::~CCycQTImage()
{}

int CCycQTImage::display_rgba(
    void* data, 
    int width, 
    int height, 
    const std::string& title)
{
    //CCycQTSkeleton::instance().assert_running();
    if (!m_CcrQTSkeleton->is_running())
        return -3;

    RegisteredImageDisplayApp* thisapp = m_CcrQTSkeleton->find_display_app(title);
    if(thisapp == nullptr)
    {
        // there's no app registered yet
        thisapp = m_CcrQTSkeleton->allocate_display_app(title);
        if (thisapp == nullptr)
        {
            // give up, there's no way to do it
            return -1;
        }
    }

    if (!thisapp->app->is_running())
    {
        thisapp->running = false;
        return -2;
    }

    /* WARNING We pass a pointer to some memory which might be freed later. The only reason this is 
    safe is because this emit is made to be blocking using BlockingQueuedConnection when the signals 
    are connected. Another solution would be to allocate and copy the data here, and make this signal
    non blocking. */
    emit thisapp->app->should_set_rgba_pixels(data, width, height);

    return 0;
}

int CCycQTImage::get_last_key(const std::string& title)
{
    //CCycQTSkeleton::instance().assert_running();
    if (!m_CcrQTSkeleton->is_running())
        return -1;

    RegisteredImageDisplayApp* thisapp = m_CcrQTSkeleton->find_display_app(title);
    if(thisapp == nullptr)
        return -2;

    return thisapp->app->get_last_key();
}
