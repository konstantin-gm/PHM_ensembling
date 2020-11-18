#ifndef VCH314CONTROLLER_H
#define VCH314CONTROLLER_H

#include <QThread>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimerEvent>
#include <QDebug>
#include <QFile>
#include <QDateTime>
#include <iostream>
#include <iomanip>
#include <QEventLoop>
#include <QTimer>
#include <QTcpServer>
#include <QTcpSocket>
#include <QSettings>

using namespace std;
#define GETDEVICE 0x46 //'F'
#define TEST      0x36 //'6'
#define REQCFG    0x55 //'U'
#define GETVER    0x37 //'7'
#define CONFIG    0x53 //'S'
#define SENFIFO24 0x69 //'i'
#define BEGIN     0x57 //'W'
#define SYNCHR    0x33 //'3'

class SocketPHM : public QObject
{
    Q_OBJECT
public:
    explicit SocketPHM(QObject *parent = nullptr);
    void init(const QString & hostName, quint16 port);
    quint16 _port;
    QHostAddress _IP;
    QTcpSocket socket;
    bool writeToSocket(const QByteArray &text);
    int fcode;
    void connectToHost();
private:
    QByteArray cmd;
public slots:
    void onReadyRead();
    void connected();
};

class Controller : public QObject
{
    Q_OBJECT

    //Device serial/usb port
    QSerialPort             *_Port;
    //Device data buffer
    QByteArray              _DeviceData;
public:
    //Constructor
    explicit Controller(QObject *parent = nullptr);
    ~Controller();
    //Timer function to poll the device every second
    //void timerEvent(QTimerEvent* event);
    //Thread method    
    //Configure serial port
    int initialize();
    void start();
private:
    //Flag to decide clearance to poll
    bool                    _bSendCommand;
    bool waitForBytes(int N, int maxTime);
    void ltoi(unsigned long uval, char *buf);
    void sleep(int);
    bool sendCmd(char cmd, char par1, char par2);
    bool getResp(char cmd);
    double str0x30toDbl(QString sr);
    void glue3(int k, int i, double *sam, double wid);    
    double ki;
    double kp;
    double delta;
    int dt_ctrl;
    int transient_T;
    int medianFilter(QList<double> &dat);
    SocketPHM *phm1;
    SocketPHM *phm2;
    bool is_started;
    QThread *thread;
signals:
    void write_x(const double &, const double &);
    void write_x2(const double &, const double &);
    void write_y(const double &, const double &);
    void write_u(const double &, const int &);
    void finished();
public slots:
    void handleData();
    void process();
};

#endif // VCH315CONTROLLER_H
