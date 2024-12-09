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


void MainWindow::on_regularButton_clicked()
{
    printf("Go regular mode plz");
}


void MainWindow::on_fastButton_clicked()
{
    printf("Increase the robot speed plz");
}


void MainWindow::on_emergencyStop_clicked()
{
    printf("Kill the ROS program, make the robot stop!!!");
}

