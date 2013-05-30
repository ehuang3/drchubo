/**
 * @file MainWindow.h
 * @brief Window with visualization of SL Data
 */

#ifndef _MAINWINDOW_H_
#define _MAINWINDOW_H_

#include <QMainWindow>
#include <QWidget>
#include "qnode.hpp"

// RViz stuff
namespace rviz {
  class RenderPanel;
  class VisualizationManager;
  class GridDisplay;
}

/**
 * @class MainWindow
 */
 class MainWindow : public QWidget
 {
     Q_OBJECT

 public:
   MainWindow( QNode *_node, QWidget *_parent = 0 );
   ~MainWindow();

   void closeEvent( QCloseEvent *_event );
   void showNoMasterMessage();

   /**< SLOTS */
   public Q_SLOTS:
   void on_button_connect_clicked( bool _check=true );

   /**< Manual connection */
   void updateView();

 private: 
   // ROS
   QNode* qnode;
   // rviz visualization
   rviz::VisualizationManager* mManager;
   rviz::RenderPanel* mRenderPanel;
   rviz::GridDisplay* mGrid;
 };

 #endif /** _MAINWINDOW_H_ */
