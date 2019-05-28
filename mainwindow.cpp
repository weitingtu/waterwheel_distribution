#include "mainwindow.h"
#include <QAction>
#include <QFileDialog>
#include <QMenuBar>
#include <waterstationmanager.h>
#include <truckmanager.h>
#include <initialsolution.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      _file_menu(nullptr),
      _open_act(nullptr)
{
    _create_actions();
    _create_menus();
}

MainWindow::~MainWindow()
{

}

void MainWindow::_create_actions()
{
    _open_act = new QAction(tr("&Open..."), this);
    connect(_open_act, &QAction::triggered, this, &MainWindow::_open);
}

void MainWindow::_create_menus()
{
    _file_menu = menuBar()->addMenu(tr("&File"));
    _file_menu->addAction(_open_act);
}

void MainWindow::_open()
{
    QString file_name = QFileDialog::getOpenFileName(this, tr("Open file"), QString(), tr("csv (*.csv)"));
    WaterStationManager::get_inst().parse(file_name);
    WaterStationManager::get_inst().schedule();

    InitialSolution init(TruckManager::get_inst(), WaterStationManager::get_inst());
    init.init();
    init.compute_real_cost();
    std::vector<std::vector<std::vector<size_t> > > schedule_solutions = init.tabu();
    (void) schedule_solutions;
    init.aco(schedule_solutions);
    init.tsp(schedule_solutions);
    init.near_by(schedule_solutions);
    init.end();
}
