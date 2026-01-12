
#pragma once

#include <vector>
#include <string>
#include <QtWidgets/QWidget>
#include <QtWidgets/QApplication>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QBoxLayout>
#include <QtWidgets/QLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>
#include <os/qtplot/qcustomplot.h>

class CCcrQTPlotSegment : public QWidget
{
    Q_OBJECT

public:
    CCcrQTPlotSegment(
        const std::string& name, 
        const std::string& unit, 
        const std::vector<std::string>& sigs,
        const QColor& plotcolor,
        const QColor& bgcolor,
        QWidget* parent = nullptr
    );

    ~CCcrQTPlotSegment();

    void add_signal_name(const std::string& name);

    size_t get_signal_count();
    size_t get_name_width();
    void align_signals(size_t width);

signals:
    void new_values_added(const std::vector<float> values);

public slots:
    void add_new_values(const std::vector<float> values);

private:
    void create_signal_names_layout(const std::vector<std::string>& sigs);

    void create_signal_values_layout();

    void create_signal_graph_layout(const QColor& color, const QColor& bgcolor);

    float x = 0;
    const std::string m_unit;
    const size_t m_signal_count;
    QLabel m_segment_name;
    QHBoxLayout* m_root_layout;
    QBoxLayout m_signal_names_layout;
    QBoxLayout m_signal_values_layout;
    QBoxLayout m_signal_graph_layout;
    std::vector<QLabel*> m_signal_names;
    std::vector<QLabel*> m_signal_values;
    std::vector<QCustomPlot*> m_signal_plots;
    double m_lower_y = 0;
    double m_upper_y = 0;
    double plot_min_width;
};
