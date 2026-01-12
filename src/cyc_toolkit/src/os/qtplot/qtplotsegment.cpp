
#include <QtWidgets/QWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QBoxLayout>
#include <QtWidgets/QLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>
#include <os/qtplot/qcustomplot.h>
#include <os/qtplot/qtplotsegment.h>

CCcrQTPlotSegment::CCcrQTPlotSegment(
    const std::string& name, 
    const std::string& unit, 
    const std::vector<std::string>& sigs,
    const QColor& plotcolor,
    const QColor& bgcolor,
    QWidget* parent
):
    QWidget(parent), 
    m_unit(unit),
    m_signal_count(sigs.size() == 0 ? 1 : sigs.size()),
    m_signal_names_layout(QBoxLayout::Direction::Down),
    m_signal_values_layout(QBoxLayout::Direction::Down),
    m_signal_graph_layout(QBoxLayout::Down)
{
    this->setContentsMargins(0, 0, 0, 0);
    /* Only way I found that works... */
    this->setStyleSheet("background-color: rgb(" + QString::number(bgcolor.red()) + ", " + QString::number(bgcolor.green()) + "," + QString::number(bgcolor.blue()) + ")");

    m_root_layout = new QHBoxLayout(this);
    m_root_layout->setSpacing(0);
    m_root_layout->setMargin(0);
    // m_root_layout->setAlignment(Qt::AlignLeft);
    
    /* Init tht segment name */
    m_segment_name.setText(name.c_str());
    m_segment_name.setAlignment(Qt::AlignVCenter);
    m_segment_name.setContentsMargins(10, 0, 0, 0);

    create_signal_names_layout(sigs);
    create_signal_values_layout();
    create_signal_graph_layout(plotcolor, bgcolor);

    /* Add everythiong to the root layout */
    m_root_layout->addWidget(&m_segment_name);
    m_root_layout->addLayout(&m_signal_names_layout);
    m_root_layout->addLayout(&m_signal_values_layout);
    m_root_layout->addLayout(&m_signal_graph_layout, 1);
}

CCcrQTPlotSegment::~CCcrQTPlotSegment()
{
    m_root_layout->removeWidget(&m_segment_name);
    m_root_layout->removeItem(&m_signal_names_layout);
    m_root_layout->removeItem(&m_signal_values_layout);
    m_root_layout->removeItem(&m_signal_graph_layout);
    delete m_root_layout;

    for(size_t i = 0; i < m_signal_names.size(); ++i)
    {
        m_signal_names_layout.removeWidget(m_signal_names[i]);
        delete m_signal_names[i];
    }

    for(size_t i = 0; i < m_signal_values.size(); ++i)
    {
        m_signal_values_layout.removeWidget(m_signal_values[i]);
        delete m_signal_values[i];
    }

    for(size_t i = 0; i < m_signal_plots.size(); ++i)
    {
        m_signal_graph_layout.removeWidget(m_signal_plots[i]);
        delete m_signal_plots[i];
    }
}

void CCcrQTPlotSegment::add_new_values(const std::vector<float> values)
{
    assert(values.size() == m_signal_count);

    /* Set bounds if we're adding values for the first time. */
    if(m_lower_y == 0 && m_upper_y == 0 && m_signal_count)
    {
        m_lower_y = values[0] - 2;
        m_upper_y = values[0] + 2;
    }

    for(size_t i = 0; i < m_signal_count; ++i)
    {
        float value = values[i];
        std::string str = std::to_string(value);
        // str.erase(str.find_last_not_of('0') + 1, std::string::npos);
        size_t idx = str.find_first_of('.');
        str.erase(idx + 5, std::string::npos);
        str += " " + m_unit;
        m_signal_values[i]->setText(str.c_str());

        QCustomPlot* plot = m_signal_plots[i];
        plot->graph(0)->addData(x, values[i]);

        /* Make sure we are in bounds */
        if(value > m_upper_y)
            m_upper_y = value + 2;
        else if(value < m_lower_y)
            m_lower_y = value - 2;

        plot->yAxis->setRange(m_lower_y, m_upper_y);
        /* Scroll x axis */
        plot->xAxis->setRange(x, 8, Qt::AlignRight);
        x += 0.1f;
        plot->replot();
    }
}

void CCcrQTPlotSegment::add_signal_name(const std::string& name)
{
    QLabel* label = new QLabel(nullptr);
    label->setText(name.c_str());
    label->setContentsMargins(0, 0, 10, 0);
    m_signal_names.emplace_back(label);
}

size_t CCcrQTPlotSegment::get_signal_count()
{
    return m_signal_count;
}

size_t CCcrQTPlotSegment::get_name_width()
{
    return m_segment_name.geometry().width();
}

void CCcrQTPlotSegment::align_signals(size_t width)
{
    size_t segment_width = m_segment_name.geometry().width();
    m_segment_name.setContentsMargins(10, 0, width - segment_width, 0);
}

void CCcrQTPlotSegment::create_signal_names_layout(const std::vector<std::string>& sigs)
{
    if(sigs.size() == 0)
    {
        add_signal_name("");
        m_signal_names_layout.addWidget(m_signal_names[0]);
        return;
    }

    for (const auto& sig_name: sigs)
        add_signal_name(sig_name);

    for(auto* signal_name: m_signal_names)
        m_signal_names_layout.addWidget(signal_name);
}

void CCcrQTPlotSegment::create_signal_values_layout()
{
    for(size_t i = 0; i < m_signal_count; ++i)
        m_signal_values.emplace_back(new QLabel());

    for(auto& signal_value: m_signal_values)
    {
        signal_value->setContentsMargins(0, 0, 20, 0);
        m_signal_values_layout.addWidget(signal_value);
    }
}

void CCcrQTPlotSegment::create_signal_graph_layout(const QColor& color, const QColor& bgcolor)
{
    m_signal_plots.push_back(new QCustomPlot());

    QCustomPlot* plot = m_signal_plots[0];
    m_signal_graph_layout.addWidget(plot);

    plot->setBackground(bgcolor);
    plot->addGraph();
    plot->xAxis->setRangeReversed(true);
    plot->xAxis->setVisible(false);
    plot->yAxis->setVisible(false);
    plot->axisRect()->setAutoMargins(QCP::msNone);
    plot->axisRect()->setMargins(QMargins(0,0,0,0));
    plot->graph(0)->setPen(QPen(color));
    plot->replot();
}
