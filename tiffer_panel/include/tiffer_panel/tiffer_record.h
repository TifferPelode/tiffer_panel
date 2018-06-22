///////////////////////////////////////////////////////////////////////////////
//      Title     : tiffer panel
//      Project   : Tiffer
//      Created   : 6/22/2018
//      Author    : Tiffer Pelode
//      Platforms : Ubuntu 64-bit           
//
////////////////////////////////////////////////////////////////////////////////

#ifndef TIFFER_RECORD_H
#define TIFFER_RECORD_H

#include <QThread>
#include <QObject>
#include <QDebug>

class Record_thread : public QThread
{
    Q_OBJECT

public:
    Record_thread() {}
private:
    void run()
    {
        for(int i = 0; i < 5; i++)
        {
            qDebug() << "aaa";
            QThread::sleep(5);
        }
    }
};

#endif