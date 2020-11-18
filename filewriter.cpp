#include "filewriter.h"

FileWriter::FileWriter(QObject *parent) : QObject(parent)
{    
    _file_x = fopen("x.txt","w");
    fclose(_file_x);
    _file_x2 = fopen("x_chan2.txt","w");
    fclose(_file_x2);
    _file_y = fopen("y.txt","w");
    fclose(_file_y);
    _file_u = fopen("u.txt","w");
    fclose(_file_u);
    thread = new QThread;
    this->moveToThread(thread);
    QObject::connect(this, SIGNAL(finished()), thread, SLOT(quit()));
}

FileWriter::~FileWriter()
{
    emit finished();
    thread->deleteLater();
}

void FileWriter::start()
{
    thread->start(QThread::NormalPriority);
}

void FileWriter::on_write_x(const double &t, const double &x)
{
    _file_x = fopen("x.txt", "a+");
    if (_file_x != nullptr) {
        fprintf(_file_x, "%f %.8e\n", t, x*1e-6);
        fclose(_file_x);
        _file_x = nullptr;
    }
}

void FileWriter::on_write_x2(const double &t, const double &x)
{
    _file_x2 = fopen("x_chan2.txt", "a+");
    if (_file_x2 != nullptr) {
        fprintf(_file_x2, "%f %.8e\n", t, x*1e-6);
        fclose(_file_x2);
        _file_x2 = nullptr;
    }
}

void FileWriter::on_write_y(const double &t, const double &y)
{
    _file_y = fopen("y.txt", "a+");
    if (_file_y != nullptr) {
        fprintf(_file_y, "%f %.8e\n", t, y*1e-6);
        fclose(_file_y);
        _file_y = nullptr;
    }
}

void FileWriter::on_write_u(const double &t, const int &u)
{
    _file_u = fopen("u.txt", "a+");
    if (_file_u != nullptr) {
        fprintf(_file_u, "%f %d\n", t, u);
        fclose(_file_u);
        _file_u = nullptr;
    }
}
