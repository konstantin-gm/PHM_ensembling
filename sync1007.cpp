#include "sync1007.h"
#include <math.h>

Controller::Controller(QObject *parent) : QObject(parent)
{
    _bSendCommand = true;    
    bool ok;
    QSettings settings("parameters.ini", QSettings::IniFormat);
    settings.beginGroup("set1");
    ki = settings.value("KI", 0.05).toDouble(&ok);
    if (!ok)
        ki = 0.05;
    kp = settings.value("KP", 0.01).toDouble(&ok);
    if (!ok)
        kp = 0.01;
    dt_ctrl = settings.value("DT", 10).toInt(&ok);
    if (!ok)
        dt_ctrl = 10;
    settings.endGroup();

    delta = 0;
    transient_T = 30;    
    is_started = false;
    thread = new QThread;
    this->moveToThread(thread);
    QObject::connect(thread, SIGNAL(started()), SLOT(process()));
    QObject::connect(this, SIGNAL(finished()), thread, SLOT(quit()));
}

Controller::~Controller()
{
    is_started = false;
    emit finished();
    thread->deleteLater();
    delete phm1;
    delete phm2;
    delete _Port;
}
/*void Controller::timerEvent(QTimerEvent *event)
{
    static bool lock = false;
    static int cntNoResponse = 0;
    static int cntRepeatPoll = 0;
    if (!lock)
    {
        //K: Lock this function to exclude double entry
        lock = true;
        handleData();
        //if message is not read yet, do not poll
        if(_bSendCommand)
        {
            _bSendCommand = false;
            //sendCmd(0x69,0x30,0x30);
            //_Port->flush();
            cntNoResponse = 0;
            cntRepeatPoll = 0;
        }
        //K: No response after 10 cycles - repeat request
        else if (cntNoResponse++ > 10)
        {
            sendCmd(0x69,0x30,0x30);
            _Port->flush();
            printf("%20.3f\tMeasurement poll repeat\n", QDateTime::currentMSecsSinceEpoch()/1000.0);
//            fprintf(_file,"%20.3f\tMeasurement poll repeat\n", QDateTime::currentMSecsSinceEpoch()/1000.0);
            cntNoResponse = 0;
            cntRepeatPoll ++;
        }
        //K: 10 repeated requests didn't help, then reconnect
        if (cntRepeatPoll > 10)
        {
            _DeviceData.clear();
            cntRepeatPoll = 0;
            _Port->close();
            sleep(100);
            printf("%20.3f\tReconnecting\n", QDateTime::currentMSecsSinceEpoch()/1000.0);
//            fprintf(_file,"%20.3f\tReconnecting\n", QDateTime::currentMSecsSinceEpoch()/1000.0);
            bool bResult = _Port->open(QSerialPort::ReadWrite);
            if (bResult == false)
            {
                qDebug()<<_Port->error();
//                fprintf(_file,"%20.3f\tFailed to reconnect\n", QDateTime::currentMSecsSinceEpoch()/1000.0);
            }
        }
        lock = false;
    }
}*/

//Thread method
void Controller::process()
{
    //Start event loop
    //exec();
    initialize();
    while (is_started) {
        handleData();
        sleep(100);
    }
    _Port->close();
}

int Controller::initialize()
{
    _Port = new QSerialPort;
    phm1 = new SocketPHM;
    phm2 = new SocketPHM;
    phm1->init("192.168.126.235", 5000);
    phm2->init("192.168.126.234", 5000);
    bool bResult = false;
    _Port->setPortName(QString("COM2"));
    bResult = _Port->setBaudRate(QSerialPort::Baud19200);
    if(bResult == false)
    {
        qDebug()<<"Error";
    }
    bResult = _Port->setDataBits(QSerialPort::Data8);
    if(bResult == false)
    {
        qDebug()<<"Error";
    }
    bResult = _Port->setFlowControl(QSerialPort::NoFlowControl);
    if(bResult == false)
    {
        qDebug()<<"Error";
    }
    bResult = _Port->setStopBits(QSerialPort::OneStop);
    if(bResult == false)
    {
        qDebug()<<"Error";
    }
    bResult = _Port->setParity(QSerialPort::NoParity);
    if(bResult == false)
    {
        qDebug()<<"Error";
    }
    bResult = _Port->open(QSerialPort::ReadWrite);
    if(bResult == false)
    {
        qDebug()<<_Port->error();
        return -1;
    }
    char p1, p2;
    p2 = '0';
    p1 = '0';
    p2 += 4;
    p1 += 4;
    p1 += 8;
    if (sendCmd(CONFIG, p1, p2))
    {
        sleep(500);
        getResp(CONFIG);
        if (sendCmd(SYNCHR,'1','0'))
        {
            sleep(2000);
            getResp(SYNCHR);
        }
        if (sendCmd(BEGIN,'1','0'))
        {
            sleep(500);
            if (getResp(BEGIN))
            {
                sleep(100);
                //Start the measurement poll timer(every second)
                //startTimer(200);
                is_started = true;
                printf("Started!\n");
            }
            else
                return -1;
        }
    }
    phm1->connectToHost();
    phm2->connectToHost();
    return 0;
}

bool Controller::sendCmd(char cmd, char par1, char par2)
{
    bool ret = false;
    if (_Port->isOpen())
    {
        _Port->setDataTerminalReady(true);
        _Port->setRequestToSend(false);
        sleep(50);
        _Port->flush();
        sleep(50);
        _Port->setRequestToSend(true);
        char to_send[4];
        to_send[0] = 0x01;
        to_send[1] = cmd;
        to_send[2] = par1;
        to_send[3] = par2;
        qint64 bytes_written = _Port->write(to_send, 4);
        if (bytes_written == 4)
            ret = true;
    }
    return ret;
}

bool Controller::getResp(char cmd)
{
    bool ret = false;
    int len = 61;
    int tmax;
    switch (cmd)
    {
    case CONFIG:
        len = 4;
        tmax = 100;
        break;
    case SYNCHR:
        len = 5;
        tmax = 26000;
        break;
    case BEGIN:
        len = 4;
        tmax = 10000;
        break;
    default:
        tmax = 500;
        break;
    }
    if (waitForBytes(len, tmax))
    {
        QByteArray getdata = _Port->readAll();
        len = getdata.length();
        if (getdata[0] == char(0x01) && getdata[1] == cmd)
        {
            ret = true;
        }
    }
    return ret;
}

//Slot/Callback to handle data available on device port
void Controller::handleData()
{
    //int iStartIndex = 0;
    QString timestr, phasestr;
    double t, x;
    int u;
    static double xprev = 0;
    static double y = 0, yprev = 0;
    static double tprev1, tprev2;
    static int totnum1 = 0, totnum2 = 0;
    static int cnt = 0;
    QByteArray data_rcvd;
    static QList<double> ylist;
    static char par = 0x30;
    static int rep_cnt = 0;

    sendCmd(0x69,par,0x30);
    //_Port->flush();
    //sleep(400);
    waitForBytes(61, 500);

    _DeviceData = _Port->readAll();
    if (_DeviceData.length() >= 61) {
        par = 0x30;
        rep_cnt = 0;
        //if (_Port->bytesAvailable() >= 61) {
        //data_rcvd = _Port->readAll();
        //_DeviceData = _Port->readAll();

        //Append the new data to the processing buffer
        //_DeviceData.append(data_rcvd);

        //As long as there are headers in the processing buffer, to ensure no message is missed
        //while((iStartIndex = _DeviceData.indexOf(0x01)) >= 0)
        //if ((iStartIndex = _DeviceData.indexOf(0x01)) >= 0)
        if (_DeviceData[0] == char(0x01))
        {

            //Clear the processing buffer contents till the header is  aligned at the start of the buffer
            //_DeviceData.remove(0, iStartIndex + 1);
            _DeviceData.remove(0, 1);
            //Look for the message trailer
            //iEndIndex = _DeviceData.indexOf(0x1a, iStartIndex);
            //If trailer exists, there is a complete message in the buffer, parse now
            //if(iEndIndex > 0)
            if (_DeviceData.length() >= 60)
            {
                if(_DeviceData[0] == char(0x69))
                {
                    //Measurement response
                    if(_DeviceData[2] != '2')
                    {
                        timestr = _DeviceData.mid(13, 6); //14//43
                        phasestr = _DeviceData.mid(4, 8);//5//34
                        t = str0x30toDbl(timestr);
                        if (t > 4000000 || t < 0) {
                            par = 0x31;
                            printf("REPEAT t = %f\n", t);
                            return;
                        }
                        if (t - tprev1 > 0.5)
                        {
                            tprev1 = t;
                            x = str0x30toDbl(phasestr);
                            x /= 1.e8;
                            glue3(0, totnum1, &x, 1.);
                            emit write_x(t, x);
                            ylist.append(x - xprev);
                            printf("x = %f, y = %.4e\n", x, (x - xprev)*1e-6);
                            if (totnum1 == transient_T)
                                printf("Start steering\n");
                            totnum1 ++;

                            if (cnt++ == dt_ctrl)
                            {
                                yprev = y;
                                medianFilter(ylist);
                                printf("ylist.length() = %d, ", ylist.length());
                                //                            if (ylist.length() > 0)
                                //                                y = std::accumulate(ylist.begin(), ylist.end(), 0)/static_cast<double>(ylist.length());
                                int len = ylist.length();
                                for (int i = 0; i < len; ++i)
                                    y += ylist[i];
                                if (len > 0) {
                                    y /= static_cast<double>(len);
                                    ylist.clear();
                                    emit write_y(t, y);
                                }
                                cnt = 1;
                                if (totnum1 >= transient_T && len > 0)
                                {
                                    //u = static_cast<int>((ki*(x - xprev) + kp*(x - 2*xprev + xpprev))*1e9/static_cast<double>(dt_ctrl));
                                    u = static_cast<int>((ki*y + kp*(y - yprev))*1e9);
                                    printf("u = %d ", u);
                                    emit write_u(t, u);
                                    if (u < 500 && u > -500 && u != 0 && 1)
                                    {
                                        phm1->fcode += u; //290(.234) "-", 287(.235) "+"
                                        phm2->fcode -= u;
                                        QByteArray cmd = "SET_SYNTH " + QByteArray::number(phm1->fcode);
                                        phm1->writeToSocket(cmd);
                                        cmd = "SET_SYNTH " + QByteArray::number(phm2->fcode);
                                        phm2->writeToSocket(cmd);
                                    }
                                }
                            }
                            xprev = x;
                        }
                    }
                    if(_DeviceData[31] != '2')
                    {
                        timestr = _DeviceData.mid(42, 6); //14//43
                        phasestr = _DeviceData.mid(33, 8);//5//34
                        t = str0x30toDbl(timestr);
                        if (t - tprev2 > 0.5)
                        {
                            tprev2 = t;
                            x = str0x30toDbl(phasestr);
                            x /= 1.e8;
                            glue3(1, totnum2, &x, 1.);
                            emit write_x2(t, x);
                            //printf("chan2 %f, %.8f\n", t, x);
                            totnum2 ++;
                        }
                    }
                }
                _DeviceData.remove(0, 60);
                _bSendCommand = true;
            }
            //Complete message not available yet. Break and wait for more data
            //        else
            //        {
            //            break;
            //        }
        }
    }
    else {
        par = 0x31;
        printf("REPEAT\n");
        if (rep_cnt ++ > 5) {
            printf("RECONNECT\n");
            _Port->close();
            sleep(100);
            _Port->open(QSerialPort::ReadWrite);
            par = 0x30;
        }
    }

}

void Controller::ltoi(unsigned long uval, char *buf)
{
    buf[0] = static_cast<char>(uval) & static_cast<char>(0xff);
    buf[1] = static_cast<char>(uval>>8) & static_cast<char>(0xff);
    buf[2] = static_cast<char>(uval>>16) & static_cast<char>(0xff);
    buf[3] = static_cast<char>(uval>>24) & static_cast<char>(0xff);
}

bool Controller::waitForBytes(int N, int maxTime)
{
    int dtime = 0;
    while (_Port->bytesAvailable() < N)
    {
        sleep(50);
        dtime += 50;
        if (dtime > maxTime)
            return false;
    }
    return true;
}
//K: instead of msleep. This allows events processing
void Controller::sleep(int ms)
{
    QEventLoop loop;
    QTimer::singleShot(ms, &loop, SLOT(quit()));
    loop.exec();
}

double Controller::str0x30toDbl(QString sr)
{
    double lin = 0;
    int j;
    double mult = 1.;
    wchar_t c;
    for (j = 0; j < sr.length(); j++)
    {
        sr.mid(j,1).toWCharArray(&c);
        lin += static_cast<double>(c - 0x30) * mult;
        mult *= 16.;
    }
    return lin;
}

void Controller::glue3(int k, int i, double *sam, double wid)
{
    static double s0[2], base[2];
    if (i > 0)
    {
        if (*sam - s0[k] > wid / 2.)
        {
            base[k] -= wid;
        }
        else if (*sam - s0[k] < -wid / 2.)
        {
            base[k] += wid;
        }
    }
    else
    {
        base[k] = 0;
    }
    s0[k] = *sam;
    *sam += base[k];
}

int Controller::medianFilter(QList<double> &dat)
{
    QList<double> absDevs;
    int i;
    int n = dat.length();
    double mediana = 0;
    double mediana_of_devs = 0;
    double finalLimit = 1e-4;

    if (n > 1)
    {
        QList<double> d;
        d = dat;
        //qSort(d.begin(), d.end());
        std::sort(d.begin(), d.end());
        int n_med = static_cast<int>(static_cast<double>(n)/2.);
        int n_med2 = static_cast<int>(static_cast<double>(n-1)/2.);
        mediana = (n % 2 == 0) ? (0.5*(d.at(n_med) + d.at(n_med - 1))) : d.at(n_med2);
        QList<double> absDevs;
        for(i = 0; i < n; ++i)
        {
            absDevs.append(fabs(d.at(i) - mediana));
        }
        //qSort(absDevs.begin(),absDevs.end());
        std::sort(absDevs.begin(),absDevs.end());
        int absLenght = absDevs.length();
        int absLength_med = static_cast<int>(static_cast<double>(absLenght)/2.);
        int absLength_med2 = static_cast<int>(static_cast<double>(absLenght-1.)/2.);
        mediana_of_devs = (absLenght % 2 == 0) ?
                    (0.5*(absDevs.at(absLength_med) + absDevs.at(absLength_med - 1)))
                  : absDevs.at(absLength_med2);
        finalLimit = mediana_of_devs*6;                  //*3                  // finalLimit = mediana_of_devs*paramMedianFilter.limitCoef;
    }
    else if (n == 1)
        mediana = dat[0];

    for (i = 0; i < n; ++i)
    {
        if (fabs(mediana - dat.at(i)) > finalLimit)
        {
            dat.removeAt(i);
            --n;
            --i;
        }
    }
    return n;
}

SocketPHM::SocketPHM(QObject *parent) : QObject(parent)
{
    connect(&socket, SIGNAL(readyRead()), this, SLOT(onReadyRead()));
    connect(&socket, SIGNAL(connected()), this, SLOT(connected()));
    cmd = "?SYNTH";
    fcode = 100000;
}

void SocketPHM::init(const QString & hostName, quint16 port)
{
    this->_IP.setAddress(hostName);
    this->_port = port;
}


void SocketPHM::connectToHost()
{
    if (socket.state() == QAbstractSocket::UnconnectedState)
        socket.connectToHost(_IP, _port);
}

bool SocketPHM::writeToSocket(const QByteArray &text)
{
    if (socket.state() == QAbstractSocket::ConnectedState)
    {
        cmd = text;
        printf(text);
        printf("\n");
        socket.write(text);
        return true;
    }
    else
    {
        printf("TRY TO RECONNECT\n");
        socket.connectToHost(_IP, _port);
    }
    return false;
}

void SocketPHM::onReadyRead()
{
    QByteArray response = socket.readAll();
    printf("RECEIVED\n");
    printf(response);
    printf("\n");
    if (cmd == "?SYNTH")
    {
        bool ok;
        int tmp;
        tmp = response.toInt(&ok);
        if (ok)
            fcode = tmp;
        printf("fcode = %d\n", fcode);
    }
}
void SocketPHM::connected()
{
    printf("CONNECTED\n");
    //writeToSocket(client_socket, "SET_SYNTH "+QString::number(-22700).toLatin1());
    //sleep(100);
    //fcode = -22280; //290
//    fcode = -20500; //287
//    writeToSocket(client_socket1, "SET_SYNTH " + QByteArray::number(fcode));
//    sleep(100);
    cmd = "?SYNTH";
    writeToSocket(cmd);
}
void Controller::start()
{
    thread->start(QThread::NormalPriority);
}
