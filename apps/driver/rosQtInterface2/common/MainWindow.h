/**
 * @file MainWindow.h
 */

#ifndef _MAINWINDOW_H_
#define _MAINWINDOW_H_

#include <QMainWindow>
#include "qnode.hpp"

 class QAction;
 class QMenu;
 class QPlainTextEdit;

/**
 * @class MainWindow
 */
 class MainWindow : public QMainWindow
 {
     Q_OBJECT

 public:
   MainWindow( QNode *_node, QWidget *_parent = 0 );
   ~MainWindow() { };
 protected:
     void closeEvent(QCloseEvent *event);

 private Q_SLOTS:
     void newFile();
     void open();
     bool save();
     bool saveAs();
     void about();
     void documentWasModified();

 private:
     void createActions();
     void createMenus();
     void createToolBars();
     void createStatusBar();
     void readSettings();
     void writeSettings();
     bool maybeSave();
     void loadFile(const QString &fileName);
     bool saveFile(const QString &fileName);
     void setCurrentFile(const QString &fileName);
     QString strippedName(const QString &fullFileName);

     QPlainTextEdit *textEdit;
     QString curFile;

     QMenu *fileMenu;
     QMenu *editMenu;
     QMenu *helpMenu;
     QToolBar *fileToolBar;
     QToolBar *editToolBar;
     QAction *newAct;
     QAction *openAct;
     QAction *saveAct;
     QAction *saveAsAct;
     QAction *exitAct;
     QAction *cutAct;
     QAction *copyAct;
     QAction *pasteAct;
     QAction *aboutAct;
     QAction *aboutQtAct;

     QNode* qnode;
     
 };

 #endif
