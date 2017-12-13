#ifndef ARDUINOTALKER_H
#define ARDUINOTALKER_H

#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QVector>

class ArduinoTalker : public QObject
{
    Q_OBJECT

private:

    QSerialPort *arduino;

    float errorLimit;

    QByteArray inBuffer, inWord;

    QVector <double> *outData;

    QVector <double> *inData;

    void ParseIncomingWord(QByteArray);

public:
    explicit ArduinoTalker(QObject *parent = 0);

    ~ArduinoTalker();

    void ConnectToArduino(QString, float);

    void DisconnectFromArduino();

    void ClearArduinoIncomingVector();

    void AddDataToOutBuffer(double);

    void ClearOutBuffer();

    void SendDataToArduino();

signals:
    void NewDataArrivedFromArduino(QVector <double>*);
    
public slots:
    
    void ReadDataFromArduino();
    
        
};

#endif // ARDUINOTALKER_H
