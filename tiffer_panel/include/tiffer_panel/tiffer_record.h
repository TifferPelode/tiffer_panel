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
#include <QProcess>
#include <QDebug>

class Record_thread : public QThread
{
    //Q_OBJECT

public:
    Record_thread() {}
private:
    void run()
    {
        // for(int i = 0; i < 5; i++)
        // {
        //     qDebug() << "aaa";
        //     QThread::sleep(5);
        // }

        std::string cur_path = ros::package::getPath("tiffer_panel");
        std::string rec_com = "arecord -c 1 -t wav -f S16_LE -r 16000 -d 4 " + cur_path + "/file/l";    
        std::string asr_com = "python3 " + cur_path + "/python/asr.py";

        QProcess* record_exec = new QProcess();
        record_exec->setEnvironment(record_exec->environment());

        QString recCom = QString::fromStdString(rec_com);
        QString asrCom = QString::fromStdString(asr_com);

        //record_exec->execute(recCom);
        record_exec->execute(asrCom);

        qDebug() << "Thread finished";

    }
};

#endif // TIFFER_RECORD_H