/**
 * @file MainWindow.cpp
 */
// Qt
#include <QtGui>
#include <QLabel>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QMessageBox>

// Rviz
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display_wrapper.h"
#include "rviz/default_plugin/grid_display.h"
#include "rviz/ogre_helpers/grid.h"

#include "MainWindow.h"

/**
 * @function MainWindow
 * @brief Constructor
 */
MainWindow::MainWindow( QNode *_node, QWidget *_parent ) :
  QWidget( _parent ),
  qnode( _node ) {

  // Set node connections
  QObject::connect( qnode, SIGNAL( loggingUpdated() ), this, SLOT(updateView()) );
  QObject::connect( qnode, SIGNAL( rosShutdown()), this, SLOT(close()) );

  // Construct and layout connect control
  QLabel* connect_label = new QLabel("ROS Connect");

  QPushButton *button_connect = new QPushButton("Connect", this);
  button_connect->setGeometry( 50, 40, 75, 30 );

  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( connect_label, 0, 0 );
  controls_layout->addWidget( button_connect, 1, 1 );
  
  
  
  // Construct and lay out render panel
  mRenderPanel = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout( controls_layout );
  main_layout->addWidget( mRenderPanel );
  
  // Set the top-level layout for the rviz widget
  setLayout( main_layout );

  // Make signal / slots connections
  QObject::connect( button_connect, 
		    SIGNAL(clicked()), 
		    this, // IMPORTANT - NO qApp!!!!
		    SLOT(on_button_connect_clicked()) );

  // Initialize main RViz classes
  mManager = new rviz::VisualizationManager( mRenderPanel );
  mRenderPanel->initialize( mManager->getSceneManager(), mManager );
  mManager->initialize();
  mManager->startUpdate();

  // Create a Grid display
  rviz::DisplayWrapper* wrapper = mManager->createDisplay( "rviz/Grid",
							   "adjustable grid",
							   "true" );

  // Unwrap it.
  rviz::Display* display = wrapper->getDisplay();
  ROS_ASSERT( display != NULL );

  // Downcast it to the type we think we know it is.
  mGrid = dynamic_cast<rviz::GridDisplay*>( display );
  ROS_ASSERT( mGrid != NULL );

  // Configure the GridDisplay the way we like it.
  mGrid->setStyle( rviz::Grid::Billboards ); // Fat lines.
  mGrid->setColor( rviz::Color( 1.0f, 1.0f, 0.0f )); // I like yellow.

}

/**
 * @function ~MainWindow
 */
MainWindow::~MainWindow() {

  if( mManager != NULL ) {
    mManager->removeAllDisplays();
  }

  delete mRenderPanel;
  delete mManager;

}

/**
 * @function closeEvent
 */
void MainWindow::closeEvent( QCloseEvent *_event )
{
  qnode->shutdown();
  QWidget::closeEvent( _event );
}

/**
 * @function showNoMasterMessage
 */
void MainWindow::showNoMasterMessage() {

  QMessageBox msgBox;
  msgBox.setText( "Could not find the ros master \n" );
  msgBox.exec();
  close();

}


/**
 *
 */
void MainWindow::on_button_connect_clicked( bool _check ) {
  printf("Checking connection \n");
  if( !qnode->on_init(std::string("http://localhost:11311"), std::string("192.168.1.92") ) ) {
    showNoMasterMessage();
  }
  else {
    printf("It is connected! \n");
  }

}

/**
 * @function updateView
 */
void MainWindow::updateView() {
  printf("Updateing! \n");
}
