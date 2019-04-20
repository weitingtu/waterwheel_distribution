#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

class QMenu;
class QAction;
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();
private slots:
    void _open();
private:
    void _create_actions();
    void _create_menus();

    QMenu*   _file_menu;
    QAction* _open_act;
};

#endif // MAINWINDOW_H
