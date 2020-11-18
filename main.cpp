#include <QCoreApplication>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QDebug>
#include <QFile>
#include <QFileInfo>
#include <QThread>
#include "filewriter.h"
#include "sync1007.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    Controller *v = new Controller ();
    FileWriter *f = new FileWriter ();
    QObject::connect(v, SIGNAL(write_x(const double &, const double &)), f, SLOT(on_write_x(const double &, const double &)));
    QObject::connect(v, SIGNAL(write_x2(const double &, const double &)), f, SLOT(on_write_x2(const double &, const double &)));
    QObject::connect(v, SIGNAL(write_y(const double &, const double &)), f, SLOT(on_write_y(const double &, const double &)));
    QObject::connect(v, SIGNAL(write_u(const double &, const int &)), f, SLOT(on_write_u(const double &, const int &)));
    v->start();
    f->start();
    //Timer timer;
    //timer.init();
    a.exec();
    delete v;
    delete f;
    return 0;
}
