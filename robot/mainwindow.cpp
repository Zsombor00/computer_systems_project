#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->runningRobot->setText("Robot: Not Running (Dynamically)");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_closeButton_clicked()
{
    this->close();
}

void MainWindow::updateLabel(const QString &newText)
{
    ui->label->setText(newText);
}

