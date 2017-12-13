#include "arduinotalker.h"
#include <QDebug>

ArduinoTalker::ArduinoTalker(QObject *parent) :
    QObject(parent)
{
    //create the port
    arduino = new QSerialPort(this);

    //initialise pointers to comm variables

    inData = new QVector <double>;

    outData = new QVector <double>;

    inWord ="";

    //connect arduino's readyRead() signal to read data..

    connect(arduino,SIGNAL(readyRead()),this,SLOT(ReadDataFromArduino()));
}

ArduinoTalker::~ArduinoTalker()
{
    arduino->close();
}

void ArduinoTalker::ReadDataFromArduino()
{
    inBuffer = arduino->readAll();

    //qDebug() << "Reading  Raw Data: " << inBuffer;

    if (inBuffer.contains("&"))
    {
        int loc = inBuffer.indexOf("&", 0);

        //qDebug() << "EndMarker found at:" << loc;

//        inWord+=inBuffer.left(loc -1);

        inWord+=inBuffer.left(loc);

        ParseIncomingWord(inWord);

        //Emit the DATA RECEIVED SIGNAL

        //qDebug() << "Data Received...";

        for (int i = 0; i< inData->size(); i++)
        {
            //qDebug() << inData->at(i);
        }

        NewDataArrivedFromArduino(inData);

        inData->clear();

        inWord.clear();

        inWord+=inBuffer.right(inBuffer.size() - loc - 1);

     }
    else
    {
        //qDebug() << "No end marker.";

        inWord+=inBuffer;

    }

    arduino->clear(QSerialPort::AllDirections);

}

void ArduinoTalker::ParseIncomingWord(QByteArray inWord)
{
    QByteArray numWord;

    //qDebug() << "Parsing word:" << inWord;

    //Parse the word only if the end padding is present

    if(inWord.startsWith("*") && inWord.endsWith("*"))
    {
        //Remove the padding

        inWord = inWord.right(inWord.size() - 1);

        inWord = inWord.left(inWord.size() - 2);

        //qDebug() << "Word after padding stripped:" << inWord;

        while (inWord.contains(","))
        {
            int cutPoint = inWord.indexOf(",",0);

            //qDebug() << "Parsing...";

            //qDebug() << inWord;

            //qDebug() <<"Cutting before:" << cutPoint;

            numWord = inWord.left(cutPoint);

            //qDebug() << "Extracting.." << numWord;

            inData->append(numWord.toFloat(0));

            inWord = inWord.right(inWord.size() - cutPoint - 1);

            //qDebug() << "Remaining.." << inWord;
        }

        inData->append(inWord.toFloat(0));

        //qDebug() << "Extraction OVer";

        //Do a rdumentary CRC check

        double rxSum = 0;

        for (int i = 0; i < inData->size() - 1; i++)
        {
            rxSum+=inData->at(i);

            //qDebug() << "Extracting.." << inData->at(i);
        }

        if(rxSum - inData->last() > errorLimit)
        {
//            qDebug() << "RX SUM:" << rxSum;

//            qDebug() << "SUM SHOULD BE:" << inData->last();

            inData->clear();

            //qDebug() << "Packet Dropped. Data Was Garbled";
        }

        //delete the check sum from the parsed data

        if(!inData->isEmpty())
        {
            inData->removeLast();
        }

    }
}

void ArduinoTalker::ClearArduinoIncomingVector()
{
    inData->clear();
}

void ArduinoTalker::ConnectToArduino(QString portName, float eL)
{
    //open the port and set parameters

    arduino->setPortName(portName);

    errorLimit = eL;

    if(arduino->open(QIODevice::ReadWrite))
    {
        // Baud rate
        arduino->setBaudRate(QSerialPort::Baud38400);
        // 8bit mode
        arduino->setDataBits(QSerialPort::Data8);
        //No parity
        arduino->setParity(QSerialPort::NoParity);
        //One stop bit
        arduino->setStopBits(QSerialPort::OneStop);
        // Skipping hw/sw control
        arduino->setFlowControl(QSerialPort::NoFlowControl);

    }
}

void ArduinoTalker::DisconnectFromArduino()
{
    arduino->close();
}

void ArduinoTalker::SendDataToArduino()
{
    QByteArray outWord;

    double val =0;

    for (int i=0; i<=outData->size()-1; i++)
    {
        val = outData->at(i);

        outWord.append(QByteArray::number(val,'f',1));

        if(i==outData->size()-1)
        {
            //outWord.append("&");
        } else outWord.append(" ");

        qDebug() << "Appending..." << outWord.data();
    }

    arduino->write(outWord);

}

void ArduinoTalker::AddDataToOutBuffer(double num)
{
    outData->append(num);
}

void ArduinoTalker::ClearOutBuffer()
{
    outData->clear();
}
