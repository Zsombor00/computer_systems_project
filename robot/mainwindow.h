#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_closeButton_clicked();

    void on_regularButton_clicked();

    void on_fastButton_clicked();

    void on_emergencyStop_clicked();

public slots:
    void updateLabel(const QString &newText); // Slot for updating QLabel text


private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
