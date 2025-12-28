#include "tab4sensorchart.h"
#include "ui_tab4sensorchart.h"

Tab4SensorChart::Tab4SensorChart(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab4SensorChart)
{
    ui->setupUi(this);
    illuLine = new QLineSeries(this);
    illuLine->setName("조도");

    QPen pen;
    pen.setWidth(2);
    pen.setBrush(Qt::red);
    pen.setCapStyle(Qt::FlatCap);
    pen.setJoinStyle(Qt::MiterJoin);
    illuLine->setPen(pen);

    pQChart = new QChart();
    pQChart->addSeries(illuLine);

    pQChart->createDefaultAxes();
    pQChart->axes(Qt::Vertical).constFirst()->setRange(0,100);

    pQChartView = new QChartView(pQChart);
    pQChartView->setRenderHint(QPainter::Antialiasing);

    pQDateTimeAixs = new QDateTimeAxis;
    pQDateTimeAixs->setFormat("hh:mm");

    updateLastDateTime(0);

    ui->pChartViewLayout->layout()->addWidget(pQChartView);
    pQChartView->chart()->setAxisX(pQDateTimeAixs, illuLine);

    // illuLine->append(0.25,50.0);
    // illuLine->append(0.50,75.0);
    // illuLine->append(0.75,0.0);
}

void  Tab4SensorChart::updateLastDateTime(bool bCheck) {

    QDate date = QDate::currentDate();
    QTime time = QTime::currentTime();
    firstDateTime.setDate(date);
    firstDateTime.setTime(time);
    lastDateTime.setDate(date);
    lastDateTime.setTime(time.addSecs(60*10));   //60초 * 10분
    pQDateTimeAixs->setRange(firstDateTime, lastDateTime);
}

void Tab4SensorChart::tab4RecvDataSlot(QString strData) {
    QStringList strList = strData.split("@");  //strList[0]= "";  @10@SENSOR@조도@온도@습도
    QDateTime dateTime = QDateTime::currentDateTime();

    QString strIllu = strList[3] ;    //조도
    illuLine->append(dateTime.toMSecsSinceEpoch(), strIllu.toInt());
}

Tab4SensorChart::~Tab4SensorChart()
{
    delete ui;
}

void Tab4SensorChart::on_pPBChartClear_clicked()
{
    illuLine->clear();
    updateLastDateTime(0);
}
