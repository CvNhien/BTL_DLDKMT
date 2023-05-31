#include "widget.h"
#include "ui_widget.h"

char bSTX[] = {0x02 };
char bADC[] = {0x41, 0x44, 0x43};
char bDAC[] = {0x44, 0x41, 0x43};
char bGDI[] = {0x47, 0x44, 0x49};
char bGDO[] = {0x47, 0x44, 0x4F};
char bSTP[] = {0x53, 0x54, 0x50};
char bTIM[] = {0x54, 0x49, 0x4D};
char bOPT[] = {0x30 ,0x30, 0x30 };
char bDATA[8] = { 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38};
char bDATA_Ori[8] = { 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30};
char bSYNC[] = { 0x16 };
char bACK[] = { 0x06 };
char bETX[] = { 0x03 };
char bProtocolDataBuffer[17] = {};
char bProtocolData[8] = {};



Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);

    timer = new QTimer(this);
    timer->setInterval(1000);
    connect(timer, SIGNAL(timeout()),this,SLOT(timer_timeout()));
    // Disable maximizing
    setFixedSize(width(), height());

    // Adding title for widget
    QWidget::setWindowTitle("Serial Port Example");

    // Ports
    QList<QSerialPortInfo> ports = info.availablePorts();
    QList<QString> stringPorts;
    for(int i = 0 ; i < ports.size() ; i++){
        stringPorts.append(ports.at(i).portName());
    }
    ui->comboBox->addItems(stringPorts);

    // Baud Rate Ratios
    QList<qint32> baudRates = info.standardBaudRates(); // What baudrates does my computer support ?
    QList<QString> stringBaudRates;
    for(int i = 0 ; i < baudRates.size() ; i++){
        stringBaudRates.append(QString::number(baudRates.at(i)));
    }
    ui->comboBox_2->addItems(stringBaudRates);

    // Data Bits
    ui->comboBox_3->addItem("5");
    ui->comboBox_3->addItem("6");
    ui->comboBox_3->addItem("7");
    ui->comboBox_3->addItem("8");

    // Stop Bits
    ui->comboBox_4->addItem("1 Bit");
    ui->comboBox_4->addItem("1,5 Bits");
    ui->comboBox_4->addItem("2 Bits");

    // Parities
    ui->comboBox_5->addItem("No Parity");
    ui->comboBox_5->addItem("Even Parity");
    ui->comboBox_5->addItem("Odd Parity");
    ui->comboBox_5->addItem("Mark Parity");
    ui->comboBox_5->addItem("Space Parity");

    //Flow Controls
    ui->comboBox_6->addItem("No Flow Control");
    ui->comboBox_6->addItem("Hardware Flow Control");
    ui->comboBox_6->addItem("Software Flow Control");

    QFont font("Arial", 20, QFont::Bold);
    ui->lineEdit_3->setFont(font);
    ui->radioButton_Event->setChecked(true);

}

Widget::~Widget()
{
    delete ui;
}


void Widget::on_pushButton_2_clicked()
{
    //  Ấn Nút CONNECT
    QString portName = ui->comboBox->currentText();
    serialPort.setPortName(portName);

    serialPort.open(QIODevice::ReadWrite);

    if(!serialPort.isOpen()){
        ui->textBrowser->setTextColor(Qt::red);
        ui->textBrowser->append("!!!! Something went Wrong !!!!");
    }
    else {

        QString stringbaudRate = ui->comboBox_2->currentText();
        int intbaudRate = stringbaudRate.toInt();
        serialPort.setBaudRate(intbaudRate);

        QString dataBits = ui->comboBox_3->currentText();
        if(dataBits == "5 Bits") {
           serialPort.setDataBits(QSerialPort::Data5);
        }
        else if((dataBits == "6 Bits")) {
           serialPort.setDataBits(QSerialPort::Data6);
        }
        else if(dataBits == "7 Bits") {
           serialPort.setDataBits(QSerialPort::Data7);
        }
        else if(dataBits == "8 Bits"){
           serialPort.setDataBits(QSerialPort::Data8);
        }

        QString stopBits = ui->comboBox_4->currentText();
        if(stopBits == "1 Bit") {
         serialPort.setStopBits(QSerialPort::OneStop);
        }
        else if(stopBits == "1,5 Bits") {
         serialPort.setStopBits(QSerialPort::OneAndHalfStop);
        }
        else if(stopBits == "2 Bits") {
         serialPort.setStopBits(QSerialPort::TwoStop);
        }

        QString parity = ui->comboBox_5->currentText();
        if(parity == "No Parity"){
          serialPort.setParity(QSerialPort::NoParity);
        }
        else if(parity == "Even Parity"){
          serialPort.setParity(QSerialPort::EvenParity);
        }
        else if(parity == "Odd Parity"){
          serialPort.setParity(QSerialPort::OddParity);
        }
        else if(parity == "Mark Parity"){
          serialPort.setParity(QSerialPort::MarkParity);
        }
        else if(parity == "Space Parity") {
          serialPort.setParity(QSerialPort::SpaceParity);
        }


        QString flowControl = ui->comboBox_6->currentText();
        if(flowControl == "No Flow Control") {
          serialPort.setFlowControl(QSerialPort::NoFlowControl);
        }
        else if(flowControl == "Hardware Flow Control") {
          serialPort.setFlowControl(QSerialPort::HardwareControl);
        }
        else if(flowControl == "Software Flow Control") {
          serialPort.setFlowControl(QSerialPort::SoftwareControl);
        }

        code ="*";
        codeSize = code.size();
        connect(&serialPort,SIGNAL(readyRead()),this,SLOT(receiveMessage()));
    }


}

void Widget::receiveMessage()
{
    QByteArray dataBA = serialPort.readAll();  //đọc tất cả dữ liệu đang chờ trong bộ đệm đầu vào của cổng nối tiếp
    QString data(dataBA);   //chuyển đổi dữ liệu đang có kiểu QByteArray (dataBA) sang chuỗi QString (data).
    buffer.append(data);    //nối một chuỗi vào cuối chuỗi hiện tại.
    int index = buffer.indexOf(code); //tìm vị trí xuất hiện đầu tiên của một chuỗi con trong chuỗi hiện tại. Nếu không tìm thấy, phương thức sẽ trả về giá trị -1.
    if(index != -1){ // khi tìm thấy ký tự đặc biệt
       QString message = buffer.mid(0,index);
       ui->textBrowser->setTextColor(Qt::blue); // Receieved message's color is blue.
       ui->textBrowser->append(message);
       buffer.remove(0,index+codeSize);
       ProcessData(message);
    }
}

// Button of Disconnect
void Widget::on_pushButton_3_clicked()
{
    serialPort.close();
}

// Button of Refresh Ports
void Widget::on_pushButton_4_clicked()
{
    ui->comboBox->clear();
    QList<QSerialPortInfo> ports = info.availablePorts();
    QList<QString> stringPorts;
    for(int i = 0 ; i < ports.size() ; i++){
        stringPorts.append(ports.at(i).portName());
    }
    ui->comboBox->addItems(stringPorts);
}

// Button of Clear
void Widget::on_pushButton_5_clicked()
{
    ui->textBrowser->clear();
}


// BUTTON OF ADC
void Widget::on_pushButton_6_clicked()
{
    ADC_Enable = true;
    DAC_Enable = false;
    DO_Enable = false;
    DI_Enable = false;

    char ProtocolFrame[18] = {};
    int index = 0;
    memcpy(ProtocolFrame+index, bSTX, sizeof(bSTX));
    index += sizeof(bSTX);
    memcpy(ProtocolFrame+index, bADC, sizeof(bADC));
    index += sizeof(bADC);
    memcpy(ProtocolFrame+index, bOPT, sizeof(bOPT));
    index += sizeof(bOPT);
    QString bDATA1 = ui->lineEdit_2->text();
    if(!ui->lineEdit_2->text().isEmpty())
    {
        char bDATA2[8];
        strcpy(bDATA2, bDATA1.toUtf8().constData());
        for(int i = 0; i<8;i++)
        {
            bDATA[i] = bDATA2[i];
        }
        memcpy(ProtocolFrame+index, bDATA, sizeof(bDATA));
        index += sizeof(bDATA);
    }
    else
    {
        memcpy(ProtocolFrame+index, bDATA_Ori, sizeof(bDATA_Ori));
        index += sizeof(bDATA_Ori);
    }
    memcpy(ProtocolFrame+index, bSYNC, sizeof(bSYNC));
    index += sizeof(bSYNC);
    memcpy(ProtocolFrame+index, bETX, sizeof(bETX));
    //index += sizeof(bETX);

    ui->textBrowser->setTextColor(Qt::darkGreen); // Color of message to send is green.
    ui->textBrowser->append(ProtocolFrame);
    //serialPort.write(ProtocolFrame.toUtf8()); //cho kiểu QString
    serialPort.write(ProtocolFrame);        // cho kiểu char
}

//Button of DAC
void Widget::on_pushButton_7_clicked()
{
    ADC_Enable = false;
    DAC_Enable = true;
    DO_Enable = false;
    DI_Enable = false;
    char ProtocolFrame[18] = {};
    int index = 0;
    memcpy(ProtocolFrame+index, bSTX, sizeof(bSTX));
    index += sizeof(bSTX);
    memcpy(ProtocolFrame+index, bDAC, sizeof(bDAC));
    index += sizeof(bDAC);
    memcpy(ProtocolFrame+index, bOPT, sizeof(bOPT));
    index += sizeof(bOPT);
    QString bDATA1 = ui->lineEdit_2->text();
    if(!ui->lineEdit_2->text().isEmpty())
    {
        char bDATA2[8];
        strcpy(bDATA2, bDATA1.toUtf8().constData());
        for(int i = 0; i<8;i++)
        {
            bDATA[i] = bDATA2[i];
        }
        memcpy(ProtocolFrame+index, bDATA, sizeof(bDATA));
        index += sizeof(bDATA);
    }
    else
    {
        memcpy(ProtocolFrame+index, bDATA_Ori, sizeof(bDATA_Ori));
        index += sizeof(bDATA_Ori);
    }

    memcpy(ProtocolFrame+index, bSYNC, sizeof(bSYNC));
    index += sizeof(bSYNC);
    memcpy(ProtocolFrame+index, bETX, sizeof(bETX));
    //index += sizeof(bETX);

    ui->textBrowser->setTextColor(Qt::darkGreen); // Color of message to send is green.
    ui->textBrowser->append(ProtocolFrame);
    //serialPort.write(ProtocolFrame.toUtf8());
    serialPort.write(ProtocolFrame);
}
// Button of DI
void Widget::on_pushButton_8_clicked()
{
    ADC_Enable = false;
    DAC_Enable = false;
    DO_Enable = false;
    DI_Enable = true;
    char ProtocolFrame[18] = {};
    int index = 0;
    memcpy(ProtocolFrame+index, bSTX, sizeof(bSTX));
    index += sizeof(bSTX);
    memcpy(ProtocolFrame+index, bGDI, sizeof(bGDI));
    index += sizeof(bGDI);
    memcpy(ProtocolFrame+index, bOPT, sizeof(bOPT));
    index += sizeof(bOPT);
    QString bDATA1 = ui->lineEdit_2->text();
    if(!ui->lineEdit_2->text().isEmpty())
    {
        char bDATA2[8];
        strcpy(bDATA2, bDATA1.toUtf8().constData());
        for(int i = 0; i<8;i++)
        {
            bDATA[i] = bDATA2[i];
        }
        memcpy(ProtocolFrame+index, bDATA, sizeof(bDATA));
        index += sizeof(bDATA);
    }
    else
    {
        memcpy(ProtocolFrame+index, bDATA_Ori, sizeof(bDATA_Ori));
        index += sizeof(bDATA_Ori);
    }

    memcpy(ProtocolFrame+index, bSYNC, sizeof(bSYNC));
    index += sizeof(bSYNC);
    memcpy(ProtocolFrame+index, bETX, sizeof(bETX));
    //index += sizeof(bETX);

    ui->textBrowser->setTextColor(Qt::darkGreen); // Color of message to send is green.
    ui->textBrowser->append(ProtocolFrame);
    //serialPort.write(ProtocolFrame.toUtf8()); //cho kiểu QString
    serialPort.write(ProtocolFrame);        // cho kiểu char


}
//Button of DO
void Widget::on_pushButton_9_clicked()
{
    ADC_Enable = false;
    DAC_Enable = false;
    DO_Enable = true;
    DI_Enable = false;
    char ProtocolFrame[18] = {};
    int index = 0;
    memcpy(ProtocolFrame+index, bSTX, sizeof(bSTX));
    index += sizeof(bSTX);
    memcpy(ProtocolFrame+index, bGDO, sizeof(bGDO));
    index += sizeof(bGDO);
    memcpy(ProtocolFrame+index, bOPT, sizeof(bOPT));
    index += sizeof(bOPT);
    char bDATA_DO[8] = { 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30};
    if(ui->checkBox_LGreen->isChecked()==1)
    {
        bDATA_DO[4] = {0x31};
    }
    if(ui->checkBox_Orange->isChecked()==1)
    {
        bDATA_DO[5] = {0x31};
    }
    if(ui->checkBox_Red->isChecked()==1)
    {
        bDATA_DO[6] = {0x31};
    }
    if(ui->checkBox_Blue->isChecked()==1)
    {
        bDATA_DO[7] = {0x31};
    }
    memcpy(ProtocolFrame+index, bDATA_DO, sizeof(bDATA_DO));
    index += sizeof(bDATA_DO);
    //QString bDATA1 = ui->lineEdit_2->text();
    //memcpy(ProtocolFrame+index, bDATA1.toUtf8(), sizeof(bDATA1.toUtf8()));
    //index += sizeof(bDATA1.toUtf8());
    memcpy(ProtocolFrame+index, bSYNC, sizeof(bSYNC));
    index += sizeof(bSYNC);
    memcpy(ProtocolFrame+index, bETX, sizeof(bETX));
    //index += sizeof(bETX);

    ui->textBrowser->setTextColor(Qt::darkGreen); // Color of message to send is green.
    ui->textBrowser->append(ProtocolFrame);
    //serialPort.write(ProtocolFrame.toUtf8());
    serialPort.write(ProtocolFrame);


}

 //STOP - Dừng gửi dữ liệu
void Widget::on_pushButton_10_clicked()
{
    ADC_Enable = false;
    DAC_Enable = false;
    DO_Enable = false;
    DI_Enable = false;
    char ProtocolFrame[18] = {};
    int index = 0;
    memcpy(ProtocolFrame+index, bSTX, sizeof(bSTX));
    index += sizeof(bSTX);
    memcpy(ProtocolFrame+index, bSTP, sizeof(bSTP));
    index += sizeof(bSTP);
    memcpy(ProtocolFrame+index, bOPT, sizeof(bOPT));
    index += sizeof(bOPT);
    char bDATA_STOP[8] = { 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30};
    memcpy(ProtocolFrame+index, bDATA_STOP, sizeof(bDATA_STOP));
    index += sizeof(bDATA_STOP);
    //QString bDATA1 = ui->lineEdit_2->text();
    //memcpy(ProtocolFrame+index, bDATA1.toUtf8(), sizeof(bDATA1.toUtf8()));
    //index += sizeof(bDATA1.toUtf8());
    memcpy(ProtocolFrame+index, bSYNC, sizeof(bSYNC));
    index += sizeof(bSYNC);
    memcpy(ProtocolFrame+index, bETX, sizeof(bETX));
    //index += sizeof(bETX);

    ui->textBrowser->setTextColor(Qt::darkGreen); // Color of message to send is green.
    ui->textBrowser->append(ProtocolFrame);
    //serialPort.write(ProtocolFrame.toUtf8());
    serialPort.write(ProtocolFrame);

    ui->lineEdit_3->clear();
    ui->checkBox->setChecked(false);
    timer->stop();
    ui->radioButton_Event->setChecked(true);
}

void Widget::on_radioButton_Timer_clicked()
{
    timer->start();
    ui->lineEdit_3->clear();
    ui->checkBox->setChecked(false);
}
void Widget::timer_timeout()
{
    char ProtocolFrame[18] = {};
    int index = 0;
    memcpy(ProtocolFrame+index, bSTX, sizeof(bSTX));
    index += sizeof(bSTX);
    memcpy(ProtocolFrame+index, bTIM, sizeof(bTIM));
    index += sizeof(bTIM);
    memcpy(ProtocolFrame+index, bOPT, sizeof(bOPT));
    index += sizeof(bOPT);
    QString bDATA1 = ui->lineEdit_2->text();
    if(!ui->lineEdit_2->text().isEmpty())
    {
        char bDATA2[8];
        strcpy(bDATA2, bDATA1.toUtf8().constData());
        for(int i = 0; i<8;i++)
        {
            bDATA[i] = bDATA2[i];
        }
        memcpy(ProtocolFrame+index, bDATA, sizeof(bDATA));
        index += sizeof(bDATA);
    }
    else
    {
        memcpy(ProtocolFrame+index, bDATA_Ori, sizeof(bDATA_Ori));
        index += sizeof(bDATA_Ori);
    }

    memcpy(ProtocolFrame+index, bSYNC, sizeof(bSYNC));
    index += sizeof(bSYNC);
    memcpy(ProtocolFrame+index, bETX, sizeof(bETX));
    //index += sizeof(bETX);

    ui->textBrowser->setTextColor(Qt::darkGreen); // Color of message to send is green.
    ui->textBrowser->append(ProtocolFrame);
    //serialPort.write(ProtocolFrame.toUtf8());
    serialPort.write(ProtocolFrame);
}
void Widget::on_radioButton_Event_clicked()
{
    timer->stop();
}
void Widget::ProcessData(QString data)
{   char cmd[3];

    strcpy(bProtocolDataBuffer, data.toUtf8().constData());

    QString message1 = data.mid(1,3);
    strcpy(cmd, message1.toUtf8().constData());

    QString message2 = data.mid(7,8);
    strcpy(bProtocolData, message2.toUtf8().constData());


    if(String_Compare((char*)cmd, (char*)bDAC))
    {
    }
    else if((String_Compare(cmd,bADC)))
    {
        QString subString = data.mid(7,4);
        QString ch = ".";
        subString.insert(2, ch);
        ui->lineEdit_3->setText(subString);
    }
    else if(String_Compare(cmd,bGDO))
    {
    }
    else if(String_Compare(cmd,bGDI))
    {
        bool checked = ui->checkBox->isChecked();
        ui->checkBox->setChecked(!checked);
    }
    else if(String_Compare(cmd,bTIM))
    {
    }
    else if(String_Compare(cmd,bSTP))
    {
    }

    if(!String_Compare(cmd,bDAC))
    {
    }
    if(!String_Compare(cmd,bADC))
    {
        ui->lineEdit_3->clear();
    }
    if(!String_Compare(cmd,bGDO))
    {
        ui->checkBox_LGreen->setChecked(false);
        ui->checkBox_Orange->setChecked(false);
        ui->checkBox_Red->setChecked(false);
        ui->checkBox_Blue->setChecked(false);
    }
    if(!String_Compare(cmd,bGDI))
    {
        ui->checkBox->setChecked(false);
    }
}
bool Widget::String_Compare(char* str1, char* str2)
{
    bool checked = false;
    for(int i = 0 ; i<3;i++)
    {
        if(str1[i] == str2[i])
        checked = true;
        else
        {
        checked = false;
        break;
        }
    }
    return checked;
}
