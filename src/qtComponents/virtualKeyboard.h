#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QGridLayout>
#include <QLineEdit>
#include <QStyle>
#include <Qthread>
#pragma once

/**
 * Virtual Keyboard GUI with listener for button clicks
 */
class VirtualKeyboard : public QWidget {
    Q_OBJECT
public:
   VirtualKeyboard(QWidget *parent = nullptr);
};

/**
 * QThread subclass to run the keyboard GUI on a separate thread
 */
class KeyboardThread : public QThread {
    Q_OBJECT
    private:
        VirtualKeyboard* keyboard;  
        QApplication* app;
    public:

        KeyboardThread(QApplication* app){
            this->keyboard = new VirtualKeyboard();
            this->app = app;
        }

        void run();

        ~KeyboardThread();
};
