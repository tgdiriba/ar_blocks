#include <ar_blocks/ARBlocksInterface.h>
#include <ros/ros.h>

namespace nxr {

ARBlocksInterface::ARBlocksInterface() :
  QMainWindow(),
  current_layer_number_(1),
  layer_count_(1),
  ar_blocks_client_("ar_blocks", true)
{
  center_layout_ = new QVBoxLayout;
  
  viz2d_panel_ = new QHBoxLayout;
  viz2d_tools_ = new QHBoxLayout;
  
  block_size_layout_ = new QHBoxLayout;
  block_size_label_ = new QLabel("Block Size: \t");
  block_size_input_ = new QLineEdit;
  
  left_panel_title_ = new QLabel("Layer Creator");
  current_layer_cb_ = new QComboBox;
  current_layer_cb_->addItem("1");
  current_layer_cb_->addItem("2");
  current_layer_cb_->addItem("3");
  //current_layer_cb_->setAlignment(Qt::AlignRight);
  
  // Outdated
  current_layer_ = new QLabel("1");
  current_layer_->setAlignment(Qt::AlignRight);
  
  block_scene_ = new Scene;
  //block_scene_->clear();
  QGraphicsView *qview = new QGraphicsView(block_scene_);
  qview->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  qview->setFrameStyle(0);
  qview->setMinimumSize(scene_width, scene_height);
  qview->setMaximumSize(scene_width, scene_height);
  
  setGeometry(QStyle::alignedRect(Qt::LeftToRight, Qt::AlignCenter, size(), qApp->desktop()->availableGeometry()));

  prev_layer_btn_ = new QPushButton("Previous");
  next_layer_btn_ = new QPushButton("Next");
  remove_layer_btn_ = new QPushButton("Remove");
  add_layer_btn_ = new QPushButton("Add");
  
  layer_count_layout_ = new QHBoxLayout;
  block_count_layout_ = new QHBoxLayout;
  stability_layout_ = new QHBoxLayout;
  build_layout_ = new QHBoxLayout;
  
  layer_count_label_ = new QLabel("Layer Count: \t");
  block_count_label_ = new QLabel("Block Count: \t");
  stability_label_ = new QLabel("Stability: \t");
  
  layer_count_field_ = new QLabel;
  block_count_field_ = new QLabel;
  stability_field_ = new QLabel;

  build_progress_ = new QProgressBar;
  build_btn_ = new QPushButton("Send Build");
  abort_btn_ = new QPushButton("Abort");
  
  build_progress_->setRange(0,100);
  build_progress_->setOrientation(Qt::Horizontal);
  build_progress_->setValue(0);
  
  layer_count_layout_->addWidget(layer_count_label_);
  layer_count_layout_->addWidget(layer_count_field_);
  block_count_layout_->addWidget(block_count_label_);
  block_count_layout_->addWidget(block_count_field_);
  stability_layout_->addWidget(stability_label_);
  stability_layout_->addWidget(stability_field_);
  build_layout_->addWidget(build_progress_);
  build_layout_->addWidget(abort_btn_);
  build_layout_->addWidget(build_btn_);
  
  viz2d_tools_->addWidget(prev_layer_btn_);
  viz2d_tools_->addWidget(next_layer_btn_);
  viz2d_tools_->addWidget(remove_layer_btn_);
  viz2d_tools_->addWidget(add_layer_btn_);
  
  viz2d_panel_->addWidget(left_panel_title_);
  viz2d_panel_->addWidget(current_layer_cb_);
  //viz2d_panel_->addWidget(current_layer_);
  
  // LEFT_LAYOUT DISPLACEMENT
  center_layout_->addLayout(viz2d_panel_);
  center_layout_->addWidget(qview);
  center_layout_->addLayout(viz2d_tools_);

  block_size_layout_->addWidget(block_size_label_);
  block_size_layout_->addWidget(block_size_input_);
 
  // RIGHT LAYOUT DISPLACEMENT 
  center_layout_->addLayout(block_size_layout_);
  center_layout_->addLayout(layer_count_layout_);
  center_layout_->addLayout(block_count_layout_);
  center_layout_->addLayout(stability_layout_);
  center_layout_->addLayout(build_layout_);
  center_layout_->setAlignment(Qt::AlignBottom);

  QWidget *main_widget = new QWidget;
  main_widget->setLayout(center_layout_);
  main_widget->setMinimumSize(scene_width, scene_height+250);
  main_widget->setMaximumSize(scene_width, scene_height+250);
  setCentralWidget(main_widget);
  
  // Setup the signals and slots
  // QPUSHBUTTON
  connect(prev_layer_btn_, SIGNAL(clicked()), this, SLOT(previousLayerBtnHandler()));
  connect(next_layer_btn_, SIGNAL(clicked()), this, SLOT(nextLayerBtnHandler()));
  connect(remove_layer_btn_, SIGNAL(clicked()), block_scene_, SLOT(clear()));
  connect(add_layer_btn_, SIGNAL(clicked()), block_scene_, SLOT(addLayerBtnHandler()));
  connect(abort_btn_, SIGNAL(clicked()), this, SIGNAL(abortBtnHandler()));
  connect(build_btn_, SIGNAL(clicked()), this, SIGNAL(buildBtnHandler()));
  
  // Button signals and slots
  
  // ar_blocks_client_.waitForServer();

  statusBar()->showMessage("");

  // Testing
  ar_blocks::Layer l;
  ar_blocks::Block b;
  b.length = 5.0;
  b.width = 5.0;
  b.height = 5.0;
  b.pose_stamped.pose.position.x = table_dim_x / 2.0;
  b.pose_stamped.pose.position.y = table_dim_y / 2.0;
  l.blocks.push_back(b);
  // drawLayer(l); 

  drawTable();
  
}

void ARBlocksInterface::drawTable()
{
  double x_ratio = (scene_width - 20.0)/ table_dim_x;
  double y_ratio = (scene_height - 20.0)/ table_dim_y;
  double ratio = (x_ratio < y_ratio) ? x_ratio : y_ratio; 
  
  block_scene_->addRect(0.0, 0.0, ratio*table_dim_x, ratio*table_dim_y);
}

void ARBlocksInterface::clearScene()
{
  block_scene_->clear();
}

void ARBlocksInterface::drawScene()
{
  
}

void ARBlocksInterface::drawLayer(ar_blocks::Layer layer, QPen pen, QBrush brush)
{
  for(int i = 0; i < layer.blocks.size(); i++) {
    // Use the block's 2D projection onto the table
    double num_pixels_y = ratio * layer.blocks[i].width;
    double num_pixels_x = ratio * layer.blocks[i].length;
    // ROS conventions flip the x and y dimensions due to the z axis being out of the plane
    
    block_scene_->addRect(-layer.blocks[i].pose_stamped.pose.position.y, layer.blocks[i].pose_stamped.pose.position.x, num_pixels_x, num_pixels_y, pen, brush);
  }
}

void ARBlocksInterface::drawStaticLayer(int layer_number)
{
  if(layer_number < layer_count_ && layer_number_ > 0) {
    std::vector<ar_blocks::Block> &blocks = goal_structure_.layers[layer_number_].blocks;
    for(int i = 0; i < goal_structure_.layers[layer_number_].blocks.size(); i++) {
      block_scene_->addRect(-blocks[i].pose_stamped.pose.position.y, blocks[i].pose_stamped.pose.position.x, ratio * blocks[i].length, ratio * blocks[i].width, QPen(Qt::gray));
    }
}

void ARBlocksInterface::redrawScene()
{
  drawTable();
  if(current_layer_number_ > 1) {
    
  }
}

void ARBlocksInterface::previousLayerBtnHandler()
{
  if(current_layer_number_ == 1) {
    statusBar()->showMessage("Reached the bottom layer.");
  }
  else {
    current_layer_number_--;
    redrawScene();
    statusBar()->showMessage("Drawing previous layer.");
  }
}

void ARBlocksInterface::nextLayerBtnHandler()
{
  if(current_layer_number_ == layer_count_) {
    statusBar()->showMessage("Reached the top layer.");
  }
  else {
    current_layer_number_++;
    redrawScene();
    statusBar()->showMessage("Drawing next layer.");
  }
}

void ARBlocksInterface::removeLayerBtnHandler()
{
  if(current_layer_number_ != layer_count_) {
    statusBar()->showMessage("Can only remove the top layer.");
  }
  else {
    block_store_.pop_back();
    layer_count_--;
    current_layer_number_--;
    redrawScene(); 
    statusBar()->showMessage("Removing the top layer.");
  }
}

void ARBlocksInterface::addLayerBtnHandler()
{
  if(current_layer_number_ != layer_count_) {
    statusBar()->showMessage("Can only add to the top layer.");
  }
  else {
    block_store_.push_back( std::vector<int>() );
    layer_count_++;
    current_layer_number_++;
    redrawScene();
    statusBar()->showMessage("Adding a top layer.");
  }
}

void ARBlocksInterface::abortBtnHandler()
{
  ar_blocks_client_.cancelGoal();
  statusBar()->showMessage("Build was cancelled.");
}

void ARBlocksInterface::buildBtnHandler()
{
  ar_blocks_client_.sendGoal( goal_structure_,
                              boost::bind(&ARBlocksInterface::arBlocksDoneCallback, this, _1, _2),
                              boost::bind(&ARBlocksInterface::arBlocksActiveCallback, this),
                              boost::bind(&ARBlocksInterface::arBlocksFeedbackCallback, this, _1) );
  statusBar()->showMessage("Build was sent.");
}

void ARBlocksInterface::arBlocksDoneCallback(const actionlib::SimpleClientGoalState &state,
                          const ar_blocks::BuildStructureResultConstPtr &result)
{
  statusBar()->showMessage("Build complete");
  build_progress_->setValue(100);
}

void ARBlocksInterface::arBlocksActiveCallback()
{
  statusBar()->showMessage("Build action reached. Now active.");
  build_progress_->setValue(0);
}

void ARBlocksInterface::arBlocksFeedbackCallback(const ar_blocks::BuildStructureFeedbackConstPtr &feedback)
{
  statusBar()->showMessage("Feedback message received...");
  
  // Count the number of blocks that were successfully placed and take a percentage of the total
  double valid_blocks = 0.0;
  double total_blocks = 0.0;
  for(int i = 0; i < feedback->updated_structure.layers.size(); i++) valid_blocks += feedback->updated_structure.layers[i].blocks.size();
  for(int i = 0; i < goal_structure_.goal_structure.layers.size(); i++) total_blocks += goal_structure_.goal_structure.layers[i].blocks.size();
  
  int percent = (int(total_blocks) != 0.0 && total_blocks >= 0.0) ? int(valid_blocks/total_blocks) : 0;
  
  build_progress_->setValue(percent);
}

} // namespace nxr