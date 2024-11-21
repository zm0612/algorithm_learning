#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 设置串口数据显示窗口的历史最大行数
    ui->SerialPortDataTextEdit->setMaximumBlockCount(100);

    // 新建串口程序
    serial_port_ = new QSerialPort;

    InitCustomPlot();

    // connect
    connect(serial_port_, &QSerialPort::readyRead, this, &MainWindow::ReceiveSerialData);
    connect(timer_replot_, &QTimer::timeout, this,  &MainWindow::CustomReplot);

    start_timestamp_ = std::chrono::steady_clock::now();
}

void MainWindow::InitCustomPlot(){
    // new custom plot graph
    position_graph_ = ui->CustomPlot->addGraph();
    velocity_graph_ = ui->CustomPlot->addGraph();

    QSharedPointer<QCPAxisTickerFixed> ticker(new QCPAxisTickerFixed);
    ticker->setTickStep(1.0);
    ui->CustomPlot->xAxis->setTicker(ticker);

    position_graph_->setName("Position");
    velocity_graph_->setName("Velocity");

    QPen position_pen, velocity_pen;
    position_pen.setWidth(position_line_width_);
    position_pen.setColor(QColor(255, 0, 0, 255));
    velocity_pen.setWidth(velocity_line_width_);
    velocity_pen.setColor(QColor(0, 255, 0, 255));

    position_graph_->setPen(position_pen);
    velocity_graph_->setPen(velocity_pen);

    ui->CustomPlot->legend->setVisible(true);
    ui->CustomPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
                                    QCP::iSelectLegend | QCP::iSelectPlottables);

    timer_replot_ = new QTimer(this);
    timer_replot_->setInterval(time_interval_replot_);
}

void MainWindow::CustomReplot(){
    if(!is_display_position_ && !is_display_velocity_){
        return;
    }

    std::pair<QVector<double>::iterator, QVector<double>::iterator> min_max_position;
    std::pair<QVector<double>::iterator, QVector<double>::iterator> min_max_velocity;

    bool position_data_valid = false;
    bool velocity_data_valid = false;

    if(position_data_vec_.empty() && velocity_data_vec_.empty()){
        return;
    }

    if(position_data_vec_.size() >= 2){
        position_data_valid = true;
    }

    if(velocity_data_vec_.size() >= 2){
        velocity_data_valid = true;
    }

    if(position_data_valid){
        min_max_position = std::minmax_element(std::begin(position_data_vec_), std::end(position_data_vec_));
    }

    if(velocity_data_valid){
        min_max_velocity = std::minmax_element(std::begin(velocity_data_vec_), std::end(velocity_data_vec_));
    }

    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::max();
    double max_time_s = std::numeric_limits<double>::max();
    if(position_data_valid && velocity_data_valid){
        min_y = std::min(*min_max_position.first, *min_max_velocity.first);
        max_y = std::max(*min_max_position.second, *min_max_velocity.second);
        max_time_s = std::max(velocity_time_s_vec_.back(), position_time_s_vec_.back());
    } else if (position_data_valid) {
        min_y = *min_max_position.first;
        max_y = *min_max_position.second;
        max_time_s  = position_time_s_vec_.back();
    } else if(velocity_data_valid){
        min_y = *min_max_velocity.first;
        max_y = *min_max_velocity.second;
        max_time_s = position_time_s_vec_.back();
    } else {
        return;
    }

    if(is_display_position_){
        position_graph_->setData(position_time_s_vec_, position_data_vec_);
    }

    if(is_display_velocity_){
        velocity_graph_->setData(velocity_time_s_vec_, velocity_data_vec_);
    }

    if(min_y == std::numeric_limits<double>::max() || max_y == std::numeric_limits<double>::max()){
        position_graph_->rescaleAxes();;
        velocity_graph_->rescaleAxes();
    } else{
        double delta_y = (max_y - min_y) / 5.0;
        ui->CustomPlot->yAxis->setRange(min_y - delta_y, max_y + delta_y);
        ui->CustomPlot->xAxis->setRange(std::max(0.0, max_time_s - plot_data_duration_), max_time_s);
    }
    ui->CustomPlot->replot();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_OpenSerialPortButton_clicked()
{
    QList<QSerialPortInfo> serial_port_info = QSerialPortInfo::availablePorts();

    if(serial_port_info.isEmpty()){
        QMessageBox::warning(this, "Warning", "No available serial port found", QMessageBox::Close);
        return;
    }

    for (const auto& it : serial_port_info) {
        ui->SerialPortsComboBox->addItem(it.portName());
    }
}

void MainWindow::on_BaudRateComboBox_currentTextChanged(const QString &arg1)
{
    if(arg1 == "4800"){
        serial_port_baud_rate_ = QSerialPort::Baud4800;
    } else if(arg1 == "9600"){
        serial_port_baud_rate_ = QSerialPort::Baud9600;
    } else if(arg1 == "19200"){
        serial_port_baud_rate_ = QSerialPort::Baud19200;
    } else if (arg1 == "38400") {
        serial_port_baud_rate_ = QSerialPort::Baud38400;
    } else if (arg1 == "57600") {
        serial_port_baud_rate_ = QSerialPort::Baud57600;
    } else if(arg1 == "115200"){
        serial_port_baud_rate_ = QSerialPort::Baud115200;
    }
}

void MainWindow::on_DataBitsComboBox_currentTextChanged(const QString &arg1)
{
    if(arg1 == "5"){
        serial_port_data_bits_ = QSerialPort::Data5;
    } else if(arg1 == "6"){
        serial_port_data_bits_ = QSerialPort::Data6;
    } else if(arg1 == "7"){
        serial_port_data_bits_ = QSerialPort::Data7;
    } else if(arg1 == "8"){
        serial_port_data_bits_ = QSerialPort::Data8;
    }
}

void MainWindow::on_StopBitsComboBox_currentTextChanged(const QString &arg1)
{
    if(arg1 == "1"){
        serial_port_stop_bits_ = QSerialPort::StopBits::OneStop;
    } else if(arg1 == "2"){
        serial_port_stop_bits_ = QSerialPort::StopBits::TwoStop;
    } else if(arg1 == "1.5"){
        serial_port_stop_bits_ = QSerialPort::StopBits::OneAndHalfStop;
    }
}


void MainWindow::on_VelocityCheckBox_stateChanged(int arg1)
{
    if(arg1 == Qt::Unchecked){
        is_display_velocity_ = false;
    } else if (arg1 == Qt::Checked) {
        is_display_velocity_ = true;
    }
}

void MainWindow::on_PositionCheckBox_stateChanged(int arg1)
{
    if(arg1 == Qt::Unchecked){
        is_display_position_ = false;
    } else if (arg1 == Qt::Checked) {
        is_display_position_ = true;
    }
}

bool MainWindow::SetSerialPort(){
    if(serial_port_name_.isEmpty()){
        QMessageBox::warning(this, "Warning", "Please connect and select serial port", QMessageBox::Close);
        return false;
    }

    serial_port_->setPortName(serial_port_name_);
    serial_port_->setBaudRate(serial_port_baud_rate_);
    serial_port_->setDataBits(serial_port_data_bits_);
    serial_port_->setParity(serial_port_parity_);

    if(serial_port_->isOpen()){
        serial_port_->close();
    }

    if(!serial_port_->open(QIODevice::ReadOnly)){
        QMessageBox::warning(this, "Warning", "Open COM: " + serial_port_name_ + " failed", QMessageBox::Close);
        return false;
    }

    return true;
}

void MainWindow::on_StartButton_clicked()
{
    if(!SetSerialPort()){
        return;
    }

    ui->BaudRateComboBox->setEnabled(false);
    ui->DataBitsComboBox->setEnabled(false);
    ui->SerialPortsComboBox->setEnabled(false);
    ui->StopBitsComboBox->setEnabled(false);
    ui->SerialParityComboBox->setEnabled(false);
    ui->OpenSerialPortButton->setEnabled(false);

    timer_replot_->start();

    ui->StartButton->setEnabled(false);
    ui->StopButton->setEnabled(true);
}

void MainWindow::on_SerialPortsComboBox_currentTextChanged(const QString &arg1)
{
    serial_port_name_ = arg1;
}

void MainWindow::ReceiveSerialData(){
    static std::vector<char> data_segment;
    static uint8_t receive_state = 0;

    if(!serial_port_){
        return;
    }

    const QByteArray data_array = serial_port_->readAll();

    for (const auto data : data_array) {
        if(receive_state == 0){
            data_segment.clear();
            if(data == 'D'){
                receive_state = 1; // have received start char 'D'
            } else {
                receive_state = 0;
            }
        } else if (receive_state == 1) {
            if(data == 'A'){
                receive_state = 2; // have received start char 'A'
            } else {
                receive_state = 0;
            }
        } else if (receive_state == 2) {
            if(data == 'T'){
                receive_state = 3; // have received start char 'T'
            }else {
                receive_state = 0;
            }
        }else if (receive_state == 3) {
            if(data == 'A'){
                receive_state = 4; // have received start char 'A'
            }else {
                receive_state = 0;
            }
        } else if (receive_state >= 4 && receive_state <= 8) {
            data_segment.push_back(data); // receive data
            receive_state++;
        } else if (receive_state == 9){
            if(data == 'E'){
                receive_state = 10; // have received stop char 'E'
            } else {
                data_segment.clear();
                receive_state = 0;
            }
        } else if (receive_state == 10) {
            if(data == 'N'){
                receive_state = 11; // have received stop char 'N'
            } else {
                data_segment.clear();
                receive_state = 0;
            }
        } else if (receive_state == 11) {
            if(data == 'D'){
                receive_state = 12; // have received stop char 'D'
            } else {
                data_segment.clear();
                receive_state = 0;
            }
        }

        if(receive_state < 12){
            continue;
        }

        if(data_segment.size() != 5){
            data_segment.clear();
            continue;
        }

        receive_state = 0; // reset receive status

        std::pair<MainWindow::SerialPortDataType, float> result{MainWindow::SerialPortDataType::InValid, 0.0f};

        result = ParseSerialPortData(data_segment);
        data_segment.clear();

        if(result.first == SerialPortDataType::InValid){
            continue;
        }

        const auto end = std::chrono::steady_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_timestamp_);
        const double duration_s = static_cast<double>(duration.count()) / 1.0e6;

        if(result.first == SerialPortDataType::Position){
            position_time_s_vec_.push_back(duration_s);
            position_data_vec_.push_back(result.second);

            while (position_time_s_vec_.back() - position_time_s_vec_.front() > retain_data_duration_) {
                position_data_vec_.pop_front();
                position_time_s_vec_.pop_front();
            }

            if(is_display_position_){
                ui->SerialPortDataTextEdit->appendPlainText(QString("pos: ") + QString::number(result.second, 'f', 3));
            }
        } else if (result.first == SerialPortDataType::Velocity) {
            velocity_time_s_vec_.push_back(duration_s);
            velocity_data_vec_.push_back(result.second);

            while (velocity_time_s_vec_.back() - velocity_time_s_vec_.front() > retain_data_duration_) {
                velocity_data_vec_.pop_front();
                velocity_time_s_vec_.pop_front();
            }

            if(is_display_velocity_){
                ui->SerialPortDataTextEdit->appendPlainText(QString("vel: ") + QString::number(result.second, 'f', 3));
            }
        }
    }
}

std::pair<MainWindow::SerialPortDataType, float> MainWindow::ParseSerialPortData(const std::vector<char>& data){
    if(data.size() != 5){
        return {SerialPortDataType::InValid, 0.0};
    }

    float result;
    if((uint8_t)data[0] == (uint8_t)SerialPortDataType::Position){
        result=*((float *)(&data[1]));
        return {SerialPortDataType::Position, result};
    } else if ((uint8_t)data[0] == (uint8_t)SerialPortDataType::Velocity) {
        result=*((float *)(&data[1]));
        return {SerialPortDataType::Velocity, result};
    } else {
        return {SerialPortDataType::InValid, 0.0};
    }
}

void MainWindow::on_StopButton_clicked()
{
    timer_replot_->stop();

    if(serial_port_->isOpen()){
        serial_port_->close();
    }

    // 释放串口相关按钮的状态
    ui->BaudRateComboBox->setEnabled(true);
    ui->DataBitsComboBox->setEnabled(true);
    ui->SerialPortsComboBox->setEnabled(true);
    ui->StopBitsComboBox->setEnabled(true);
    ui->SerialParityComboBox->setEnabled(true);
    ui->OpenSerialPortButton->setEnabled(true);

    ui->StartButton->setEnabled(true);
    ui->StopButton->setEnabled(false);
}

void MainWindow::on_ClearSerialPortDataButton_clicked()
{
    ui->SerialPortDataTextEdit->clear();
}

void MainWindow::on_SerialParityComboBox_currentTextChanged(const QString &arg1)
{
    if(arg1 == "No"){
        serial_port_parity_ = QSerialPort::Parity::NoParity;
    } else if(arg1 == "Even"){
        serial_port_parity_ = QSerialPort::Parity::EvenParity;
    } else if(arg1 == "Odd"){
        serial_port_parity_ = QSerialPort::Parity::OddParity;
    }
}
