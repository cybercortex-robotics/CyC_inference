
#pragma once

#include <QtWidgets/QWidget>
#include <QtWidgets/QBoxLayout>
#include <os/qtplot/qtplot.h>
#include <os/qtplot/qtplotsegment.h>

class CCcrQTPlotApp : public QWidget
{
    Q_OBJECT

public:
    CCcrQTPlotApp();
    ~CCcrQTPlotApp();

signals:
    void should_start(std::vector<CCcrQTPlotSegmentTemp> segments, const std::string title);
    void should_plot_signals(const std::vector<float> sigs);
    void should_stop();

public slots:
    void start(std::vector<CCcrQTPlotSegmentTemp> segments, const std::string title);
    void plot_signals(const std::vector<float> sigs);
    void stop();

private:
    QWidget* m_root_widget = nullptr;
    QBoxLayout* m_root_layout = nullptr;
    std::vector<CCcrQTPlotSegment*> m_segments;
};
