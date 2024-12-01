#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "qcustomplot.h"

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <chrono>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void InitCustomPlot();

    void CustomReplot();

    void on_OpenSerialPortButton_clicked();

    void on_BaudRateComboBox_currentTextChanged(const QString &arg1);

    void on_DataBitsComboBox_currentTextChanged(const QString &arg1);

    void on_StopBitsComboBox_currentTextChanged(const QString &arg1);

    void on_VelocityCheckBox_stateChanged(int arg1);

    void on_PositionCheckBox_stateChanged(int arg1);

    void on_StartButton_clicked();

    void on_SerialPortsComboBox_currentTextChanged(const QString &arg1);

    void on_StopButton_clicked();

    void on_ClearSerialPortDataButton_clicked();

    void on_SerialParityComboBox_currentTextChanged(const QString &arg1);

private:
    enum SerialPortDataType : int8_t{
        Position = '0',
        Velocity = '1',
        InValid = -1
    };

private:
    bool SetSerialPort();

    void ReceiveSerialData();

    std::pair<SerialPortDataType, float> ParseSerialPortData(const std::vector<char>& data);

private:
    Ui::MainWindow *ui;

    QTimer* timer_replot_{nullptr};

    QCPGraph* position_graph_{nullptr};
    QCPGraph* velocity_graph_{nullptr};

    QSerialPort::BaudRate serial_port_baud_rate_ = QSerialPort::Baud115200;
    QSerialPort::DataBits serial_port_data_bits_ = QSerialPort::Data8;
    QSerialPort::StopBits serial_port_stop_bits_ = QSerialPort::OneStop;
    QSerialPort::Parity serial_port_parity_ = QSerialPort::Parity::NoParity;

    QSerialPort* serial_port_{nullptr};

    QString serial_port_name_;

    QVector<double> position_time_s_vec_;
    QVector<double> velocity_time_s_vec_;
    QVector<double> position_data_vec_;
    QVector<double> velocity_data_vec_;

    bool is_display_position_{false};
    bool is_display_velocity_{false};

    int time_interval_replot_{20};
    int position_line_width_{2};
    int velocity_line_width_{2};
    double retain_data_duration_{50};
    double plot_data_duration_{15};

    uint8_t receive_state_ = 0;

    std::chrono::steady_clock::time_point start_timestamp_;
};
#endif // MAINWINDOW_H
