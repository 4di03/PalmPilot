#include "virtualKeyboard.h"
#include <iostream>

// TODO: add event handling for the keyboard buttons

VirtualKeyboard::VirtualKeyboard(QWidget *parent) : QWidget(parent) {
    setWindowTitle("Modern Virtual Keyboard");
    setFixedSize(500, 300);
    
    QVBoxLayout *layout = new QVBoxLayout(this);
    QLineEdit *display = new QLineEdit(this);
    display->setReadOnly(true);
    display->setStyleSheet("font-size: 18px; padding: 10px;");
    layout->addWidget(display);
    
    QGridLayout *grid = new QGridLayout();
    QString keys[4][10] = {
        {"1", "2", "3", "4", "5", "6", "7", "8", "9", "0"},
        {"Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P"},
        {"A", "S", "D", "F", "G", "H", "J", "K", "L", "Back"},
        {"Z", "X", "C", "V", "B", "N", "M", "Space", "Enter", "Clear"}
    };

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 10; j++) {
            QPushButton *btn = new QPushButton(keys[i][j], this);
            btn->setStyleSheet("font-size: 16px; padding: 10px; border-radius: 5px;");
            grid->addWidget(btn, i, j);
            connect(btn, &QPushButton::clicked, this, [=]() {
                if (keys[i][j] == "Back") {
                    display->setText(display->text().left(display->text().length() - 1));
                } else if (keys[i][j] == "Clear") {
                    display->clear();
                } else if (keys[i][j] == "Enter") {
                    display->setText(display->text() + "\n");
                } else if (keys[i][j] == "Space") {
                    display->setText(display->text() + " ");
                } else {
                    display->setText(display->text() + keys[i][j]);
                }
            });
        }
    }

    layout->addLayout(grid);
}

void KeyboardThread::run() {

    this->keyboard->show();
    this->app->exec();
}

KeyboardThread::~KeyboardThread(){
    if (this->keyboard != nullptr) {
        keyboard->deleteLater(); // safe deletion for QT objects
    }else{
        std::cout << "Keyboard is not raised" << std::endl;
    }

    delete this->app;
}