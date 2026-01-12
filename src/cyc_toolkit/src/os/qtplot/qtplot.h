#pragma once

#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <future>
#include "os/CSingletonBase.h"
#include "os/CSingletonRegistry.h"

#ifndef WIN32
#include <QtCore/QEvent>
#include <QtWidgets/QApplication>
#endif

#define QTPLOT_HAS_SKELETON

class QApplication;
class QWidget;
class QEvent;

class CCcrQTPlotApp;
class CCycQTSkeleton;
struct RegisteredImageDisplayApp;

struct CCcrQTPlotSegmentTemp
{
    std::string name;
    std::vector<std::string> sigs;
    std::string unit;

    CCcrQTPlotSegmentTemp(const std::string& name, const std::vector<std::string>& sigsl, const std::string& unit):
        name(name), sigs(sigs), unit(unit)
    {}
};

class CCcrQTPlot
{
public:
    CCcrQTPlot(const std::string& _title, CCycQTSkeleton* _qt_skeleton);
    ~CCcrQTPlot();

    int run();
    bool stop();

    // Add new segment values. Should only be called before CCcrQTPlot::run_on_new_thread 
    void add_segment(const std::string& name, const std::vector<std::string> &sigs, const std::string& unit = "");

    // Add new signal values. Safe to be called from another thread.
    void plot_signals(const std::vector<float>& sigs);

private:
    std::string m_title;
    std::vector<CCcrQTPlotSegmentTemp> m_temp_segments;

    CCcrQTPlotApp* m_plotapp = nullptr;

    std::shared_ptr<CCycQTSkeleton> m_CcrQTSkeleton = nullptr;
    //CCycQTSkeleton* m_QtSkeleton = nullptr;
};

class CCycQTSkeleton : public CSingletonBase
{
public:
    friend class CSingletonRegistry;

    using CCcrQTPlotAppPtr = std::unique_ptr<CCcrQTPlotApp>;
    using RegisteredImageDisplayAppPtr = std::unique_ptr<RegisteredImageDisplayApp>;

    CCycQTSkeleton(const CCycQTSkeleton&) = delete;
    CCycQTSkeleton(CCycQTSkeleton&&) = delete;
    CCycQTSkeleton& operator=(const CCycQTSkeleton&) = delete;
    CCycQTSkeleton& operator=(CCycQTSkeleton&&) = delete;
    ~CCycQTSkeleton();

    //static CCycQTSkeleton& instance();

    // Singleton type
    static const char* getType()
    {
        return "CyC_QT_SINGLETON_TYPE";
    }

    // Singleton registry function
    static std::unique_ptr<CCycQTSkeleton> create_instance(CSingletonRegistry* _singleton_registry)
    {
        return std::unique_ptr<CCycQTSkeleton>{new CCycQTSkeleton(_singleton_registry)};
    }

    void init();
    void destroy();

    //void assert_running();
    bool is_running() const { return m_qt_initialized.load(); }

    CCcrQTPlotApp* allocate_plotapp();
    void deallocate_plotapp(CCcrQTPlotApp* plotapp);

    RegisteredImageDisplayApp* find_display_app(const std::string& title) const;
    RegisteredImageDisplayApp* allocate_display_app(const std::string& title);

private:
    CCycQTSkeleton(CSingletonRegistry* _singleton_registry);

    CCcrQTPlotAppPtr createPlotApp();
    RegisteredImageDisplayAppPtr createDisplayApp();

    std::vector<CCcrQTPlotAppPtr> m_plotapps;
    std::vector<RegisteredImageDisplayAppPtr> m_display_apps;

    std::atomic<bool> m_qt_initialized = false;
    std::unique_ptr<QApplication> m_qtapp;
    std::thread m_qtplot_thread;

    std::once_flag m_init_flag;
    mutable std::mutex m_plotapp_mutex;
    mutable std::mutex m_displayapp_mutex;
};

template <typename T>
class GenericQtCreator : public QObject
{
public:
    GenericQtCreator(std::promise<std::unique_ptr<T>>& p)
        : m_promise(p)
    {
        //CCycQTSkeleton::instance().init();
    }

    bool event(QEvent* ev) override
    {
        if (ev->type() == QEvent::User)
        {
            m_promise.set_value(std::make_unique<T>());
            return true;
        }

        return false;
    }

private:
    std::promise<std::unique_ptr<T>>& m_promise;
};

template <typename T>
std::unique_ptr<T> createQtObject(CCycQTSkeleton* _qt_skeleton)
{
    //if (!CCycQTSkeleton::instance().is_running())
    if (!_qt_skeleton->is_running())
        return nullptr;

    using CreatorType = GenericQtCreator<T>;

    std::promise<std::unique_ptr<T>> promise;
    auto future = promise.get_future();

    CreatorType creator{ promise };
    creator.moveToThread(QApplication::instance()->thread());

    QApplication::postEvent(&creator, new QEvent(QEvent::User));

    return future.get();
}
