#ifndef FILEWRITER_H
#define FILEWRITER_H

#include <QObject>
#include <QThread>

class FileWriter : public QObject
{
    Q_OBJECT
private:
    QThread *thread;
    FILE *_file_x;
    FILE *_file_x2;
    FILE *_file_y;
    FILE *_file_u;
public:
    explicit FileWriter(QObject *parent = nullptr);
    ~FileWriter();
    void start();
signals:
    void finished();
public slots:
    void on_write_x(const double &, const double &);
    void on_write_x2(const double &, const double &);
    void on_write_y(const double &, const double &);
    void on_write_u(const double &, const int &);
};

#endif // FILEWRITER_H

