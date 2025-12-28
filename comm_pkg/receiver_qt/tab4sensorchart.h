#ifndef TAB4SENSORCHART_H
#define TAB4SENSORCHART_H

#include <QWidget>
#include <QChartView>
#include <QLineSeries>
#include <QDateTimeAxis>
#include <QValueAxis>
#include <QDate>
#include <QTime>
#include <QDebug>

namespace Ui {
class Tab4SensorChart;
}

class Tab4SensorChart : public QWidget
{
    Q_OBJECT

public:
    explicit Tab4SensorChart(QWidget *parent = nullptr);
    ~Tab4SensorChart();
    void  updateLastDateTime(bool);

private slots:
    void on_pPBChartClear_clicked();
    void tab4RecvDataSlot(QString);
private:
    Ui::Tab4SensorChart *ui;
    QLineSeries *illuLine;
    QChart *pQChart;
    QChartView *pQChartView;
    QDateTimeAxis *pQDateTimeAixs;
    QDateTime firstDateTime;
    QDateTime lastDateTime;
};

#endif // TAB4SENSORCHART_H
