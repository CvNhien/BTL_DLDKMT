#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QSerialPort>
#include <QSerialPortInfo>
#include "QTimer"


QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

private slots:
    void on_pushButton_2_clicked();
    void receiveMessage();
    void on_pushButton_3_clicked();
    void on_pushButton_4_clicked();
    void on_pushButton_5_clicked();
    void on_pushButton_7_clicked();
    void on_pushButton_6_clicked();
    void on_pushButton_8_clicked();
    void on_pushButton_9_clicked();
    void on_pushButton_10_clicked();
    void on_radioButton_Timer_clicked();

    void timer_timeout();

    void on_radioButton_Event_clicked();
    void ProcessData(QString data);
    bool String_Compare(char* str1, char* str2);

private:
    Ui::Widget *ui;
    QSerialPort serialPort;
    QSerialPortInfo info;
    QString buffer;
    QString code;
    QTimer *timer;
    int codeSize;

    bool ADC_Enable = false;
    bool DAC_Enable = false;
    bool DI_Enable = false;
    bool DO_Enable = false;
};
#endif // WIDGET_H
