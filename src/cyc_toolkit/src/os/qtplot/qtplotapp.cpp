// Copyright (c) 2026 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include <os/qtplot/qtplotapp.h>
#include <os/qtplot/qtplotsegment.h>
#include <QtWidgets/QBoxLayout>
#include <spdlog/spdlog.h>

static QColor g_qtplot_colors[] = {
    QColor(0xFF, 0, 0),
    QColor(0, 0, 0xFF),
    QColor(0xFF, 0, 0xFF),
    QColor(0, 0xFF, 0),
    QColor(0, 0, 0),
};

CCcrQTPlotApp::CCcrQTPlotApp():
    QWidget()
{

}

CCcrQTPlotApp::~CCcrQTPlotApp()
{
}

void CCcrQTPlotApp::start(std::vector<CCcrQTPlotSegmentTemp> segments, const std::string title)
{
    m_root_widget = new QWidget();
    m_root_widget->resize(800, 100);

    m_root_layout = new QBoxLayout(QBoxLayout::Direction::Down, m_root_widget);
    m_root_layout->setMargin(0);
    m_root_layout->setSpacing(0);

    for(const auto& seg: segments)
    {
        size_t segment_count = m_segments.size();
        size_t colors = sizeof(g_qtplot_colors) / sizeof(g_qtplot_colors[0]);
        const QColor& color = g_qtplot_colors[segment_count % colors];

        QColor bgcolor;
        if(segment_count % 2 == 0)
            bgcolor = QColor(0xEE, 0xEE, 0xEE);
        else
            bgcolor = QColor(0xFF, 0xFF, 0xFF);

        CCcrQTPlotSegment* s = new CCcrQTPlotSegment(seg.name, seg.unit, seg.sigs, color, bgcolor);
        m_segments.push_back(s);
        QObject::connect(s, &CCcrQTPlotSegment::new_values_added, s, &CCcrQTPlotSegment::add_new_values);
        m_root_layout->addWidget(s);
    }

    m_root_widget->show();

    size_t width = 0;
    for(CCcrQTPlotSegment* segment: m_segments)
    {
        size_t w = segment->get_name_width();
        if(w > width)
            width = w;
    }

    for(CCcrQTPlotSegment* segment: m_segments)
        segment->align_signals(width);
}

void CCcrQTPlotApp::plot_signals(const std::vector<float> sigs)
{
    size_t total_signals = 0;
    for(auto* segment: m_segments)
        total_signals += segment->get_signal_count();

    if(sigs.size() != total_signals)
    {
        spdlog::error("qtplot: Tried to plot {} signals, when only {} were registered!", sigs.size(), total_signals);
        return;
    }

    for (size_t i = 0; i < total_signals;)
    {
        CCcrQTPlotSegment* segment = m_segments[i];
        size_t signal_count = segment->get_signal_count();

        std::vector<float> segment_values;
        segment_values.reserve(signal_count);
        for(size_t k = 0; k < signal_count; ++k)
            segment_values.push_back(sigs[i + k]);
        
        i += signal_count;

        emit segment->add_new_values(segment_values);
    }
}

void CCcrQTPlotApp::stop()
{
    m_root_widget->hide();
    for(size_t i = 0; i < m_segments.size(); ++i)
    {
        m_root_layout->removeWidget(m_segments[i]);
        delete m_segments[i];
    }
    m_segments.clear();
    delete m_root_layout;
    m_root_layout = nullptr;
    delete m_root_widget;
    m_root_widget = nullptr;
}
