#include <ar_blocks/ARBlocksInterface.h>
#include <ros/ros.h>

namespace nxr {

ARBlocksInterface::ARBlocksInterface() :
  QMainWindow(),
  ar_blocks_client_("ar_blocks", true)
{
  side_layout_ = new QHBoxLayout;
  left_layout_ = new QVBoxLayout;
  right_layout_ = new QVBoxLayout;
  
  viz2d_panel_ = new QHBoxLayout;
  viz2d_tools_ = new QHBoxLayout;
  
  block_size_layout_ = new QHBoxLayout;
  block_size_label_ = new QLabel("Block Size");
  block_size_input_ = new QLineEdit;
  
  left_panel_title_ = new QLabel("Layer Creator");
  current_layer_ = new QLabel("1");
  current_layer_->setAlignment(Qt::AlignRight);
  block_scene_ = new Scene;
  QGraphicsView *qview = new QGraphicsView(block_scene_);
  qview->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  qview->setFrameStyle(0);
  qview->setMinimumSize(600, 600);
  qview->setMaximumSize(600,600);
  
  setGeometry(QStyle::alignedRect(Qt::LeftToRight, Qt::AlignCenter, size(), qApp->desktop()->availableGeometry()));

  prev_layer_btn_ = new QPushButton("Previous");
  next_layer_btn_ = new QPushButton("Next");
  remove_layer_btn_ = new QPushButton("Remove");
  add_layer_btn_ = new QPushButton("Add");
  
  layer_count_layout_ = new QHBoxLayout;
  block_count_layout_ = new QHBoxLayout;
  stability_layout_ = new QHBoxLayout;
  
  layer_count_label_ = new QLabel("Layer Count");
  block_count_label_ = new QLabel("Block Count");
  stability_label_ = new QLabel("Stability");
  
  layer_count_field_ = new QLabel;
  block_count_field_ = new QLabel;
  stability_field_ = new QLabel;

  build_progress_ = new QProgressBar;
  
  build_progress_->setRange(0,100);
  build_progress_->setOrientation(Qt::Horizontal);
  build_progress_->setValue(0);
  
  layer_count_layout_->addWidget(layer_count_label_);
  layer_count_layout_->addWidget(layer_count_field_);
  block_count_layout_->addWidget(block_count_label_);
  block_count_layout_->addWidget(block_count_field_);
  stability_layout_->addWidget(stability_label_);
  stability_layout_->addWidget(stability_field_);
  
  viz2d_tools_->addWidget(prev_layer_btn_);
  viz2d_tools_->addWidget(next_layer_btn_);
  viz2d_tools_->addWidget(remove_layer_btn_);
  viz2d_tools_->addWidget(add_layer_btn_);
  
  viz2d_panel_->addWidget(left_panel_title_);
  viz2d_panel_->addWidget(current_layer_);
  
  left_layout_->addLayout(viz2d_panel_);
  left_layout_->addWidget(qview);
  left_layout_->addLayout(viz2d_tools_);

  block_size_layout_->addWidget(block_size_label_);
  block_size_layout_->addWidget(block_size_input_);
  
  right_layout_->addLayout(block_size_layout_);
  right_layout_->addLayout(layer_count_layout_);
  right_layout_->addLayout(block_count_layout_);
  right_layout_->addLayout(stability_layout_);
  right_layout_->addWidget(build_progress_);
  right_layout_->setAlignment(Qt::AlignBottom);

  side_layout_->addLayout(left_layout_);
  side_layout_->addLayout(right_layout_);
  
  QWidget *main_widget = new QWidget;
  main_widget->setLayout(side_layout_);
  main_widget->setMinimumSize(900, 600);
  setCentralWidget(main_widget);
  
  // Setup the signals and slots
  connect(block_scene_, SIGNAL(message(QString)), this, SLOT(showMessage(QString)));

  // ar_blocks_client_.waitForServer();
  
}

void ARBlocksInterface::showMessage(QString msg)
{
  statusBar()->showMessage(msg);
}

} // namespace nxr
