#include <ar_blocks/ARBlocksInterface.h>

namespace nxr {

ARBlocksInterface::ARBlocksInterface() :
  QMainWindow(),
  current_layer_number_(1),
  layer_count_(1),
  ar_blocks_client_("ar_blocks_action_server", true)
{
  center_layout_ = new QVBoxLayout;
  
  viz2d_panel_ = new QHBoxLayout;
  viz2d_tools_ = new QHBoxLayout;
  
  block_size_layout_ = new QHBoxLayout;
  block_size_label_ = new QLabel("Block Size: \t");
  block_size_value_ = new QLabel("6.35 cm");
  
  left_panel_title_ = new QLabel("Layer Creator");
  current_layer_cb_ = new QComboBox;
  current_layer_cb_->addItem("1");
  
  block_scene_ = new Scene(this);
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
  
  layer_count_field_ = new QLabel("1");
  block_count_field_ = new QLabel("0");
  layer_count_field_->setAlignment(Qt::AlignLeft);
  block_count_field_->setAlignment(Qt::AlignLeft);
  stability_field_ = new QLabel("0.0");

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
  
  // LEFT_LAYOUT DISPLACEMENT
  center_layout_->addLayout(viz2d_panel_);
  center_layout_->addWidget(qview);
  center_layout_->addLayout(viz2d_tools_);

  block_size_layout_->addWidget(block_size_label_);
  block_size_layout_->addWidget(block_size_value_);
 
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
  connect(remove_layer_btn_, SIGNAL(clicked()), this, SLOT(removeLayerBtnHandler()));
  connect(add_layer_btn_, SIGNAL(clicked()), this, SLOT(addLayerBtnHandler()));
  connect(abort_btn_, SIGNAL(clicked()), this, SLOT(abortBtnHandler()));
  connect(build_btn_, SIGNAL(clicked()), this, SLOT(buildBtnHandler()));
  connect(current_layer_cb_, SIGNAL(activated(int)), this, SLOT(indexChangeHandler()));
  
  // Uncomment on release 
  // ar_blocks_client_.waitForServer();

  statusBar()->showMessage("");

  setSizePolicy(QSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed));
  layout()->setSizeConstraint(QLayout::SetFixedSize);
  
  goal_structure_.goal_structure.layers.resize(1);
}

double ARBlocksInterface::calculateStabilityMeasure()
{
  double s = 0.0;
  for(int i = 1; i < goal_structure_.goal_structure.layers.size(); i++) {
    ar_blocks::Layer& layer = goal_structure_.goal_structure.layers[i];
    for(int j = 0; j < layer.blocks.size(); j++) {
      // Calcuate a simple stability measure relative to bottom layer
      // Check if block below
      s += lowerLayerStabilityMeasure(layer.blocks[j], i-1)*i;
    }
  }
  return s;
}

double ARBlocksInterface::lowerLayerStabilityMeasure(ar_blocks::Block b, int l)
{
  // Return 0 if the distance is less than some threshold
  // Return 0.25 if the distance is between some range
  // Return 1 if the distance is greater than some threshold
  double d = 0.0;
  for(int i = 0; i < goal_structure_.goal_structure.layers[l].blocks.size(); i++) {
    // Compute distance
    double c = sqrt(
      pow(goal_structure_.goal_structure.layers[l].blocks[i].pose_stamped.pose.position.x - b.pose_stamped.pose.position.x, 2) + 
      pow(goal_structure_.goal_structure.layers[l].blocks[i].pose_stamped.pose.position.y - b.pose_stamped.pose.position.y, 2)
    );
    
    if(c < goal_structure_.goal_structure.layers[l].blocks[i].length/2)
      return 0.0; 
    else if(c < goal_structure_.goal_structure.layers[l].blocks[i].length)
      d += 0.25;
    else
      d += 1;
  }
  return d;
}

void ARBlocksInterface::indexChangeHandler()
{
  goal_structure_.goal_structure.layers[current_layer_number_-1] = block_scene_->populateLayer();
  current_layer_number_ = current_layer_cb_->currentIndex()+1;
  redrawScene();
}

void ARBlocksInterface::drawStaticLayer(int layer_number)
{
  if(layer_number < layer_count_ && layer_number > 0) {
    std::vector<ar_blocks::Block> &blocks = goal_structure_.goal_structure.layers[layer_number-1].blocks;
    
    for(int i = 0; i < goal_structure_.goal_structure.layers[layer_number-1].blocks.size(); i++) {
      
      block_scene_->addRect(
        blocks[i].pose_stamped.pose.position.x - (blocks[i].length/2),
        blocks[i].pose_stamped.pose.position.y - (blocks[i].width/2), 
        blocks[i].length, 
        blocks[i].width, 
        QPen(Qt::gray)
      );
    }
    
  }
}

void ARBlocksInterface::redrawScene()
{
  block_scene_->clear();  
  if(current_layer_number_ > 1) {
    drawStaticLayer(current_layer_number_-1);
  }
  block_scene_->drawLayer(goal_structure_.goal_structure.layers[current_layer_number_-1]);
  
  // Update the block and layer counts
  std::stringstream ss_layer, ss_block, ss_stability;
  ss_layer << layer_count_;
  layer_count_field_->setText(ss_layer.str().c_str());
  
  int bcount = 0;
  for(int i = 0; i < goal_structure_.goal_structure.layers.size(); i++)
    bcount += goal_structure_.goal_structure.layers[i].blocks.size();
  ss_block << bcount;
  block_count_field_->setText(ss_block.str().c_str());
  
  double stability_measure = calculateStabilityMeasure();
  ss_stability << stability_measure;
  stability_field_->setText(ss_stability.str().c_str());
}

void ARBlocksInterface::previousLayerBtnHandler()
{
  if(current_layer_number_ == 1) {
    statusBar()->showMessage("Reached the bottom layer.");
  }
  else {
    goal_structure_.goal_structure.layers[current_layer_number_-1] = block_scene_->populateLayer();
    current_layer_number_--;
    current_layer_cb_->setCurrentIndex(current_layer_number_-1);
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
    goal_structure_.goal_structure.layers[current_layer_number_-1] = block_scene_->populateLayer();
    current_layer_number_++;
    current_layer_cb_->setCurrentIndex(current_layer_number_-1);
    redrawScene();
    statusBar()->showMessage("Drawing next layer.");
  }
}

void ARBlocksInterface::removeLayerBtnHandler()
{
  if(current_layer_number_ != layer_count_) {
    statusBar()->showMessage("Can only remove the top layer.");
  }
  else if(current_layer_number_ == 1) {
    statusBar()->showMessage("Cleared initial layer.");
    block_scene_->clear();
    redrawScene();
  }
  else {
    block_scene_->clear();
    layer_count_--;
    current_layer_number_--;
    goal_structure_.goal_structure.layers.resize(layer_count_);
    
    current_layer_cb_->setCurrentIndex(layer_count_-1);
    current_layer_cb_->removeItem(layer_count_);
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
    goal_structure_.goal_structure.layers[current_layer_number_-1] = block_scene_->populateLayer();
    layer_count_++;
    current_layer_number_++;
    goal_structure_.goal_structure.layers.resize(layer_count_);

    std::stringstream ss;
    ss << layer_count_;
    current_layer_cb_->addItem(ss.str().c_str());
    current_layer_cb_->setCurrentIndex(current_layer_number_-1);
    redrawScene();
    
    std::stringstream output;
    output << "Adding a top layer: " << current_layer_number_ << " " << layer_count_;
    
    statusBar()->showMessage(output.str().c_str());
  }
}

void ARBlocksInterface::abortBtnHandler()
{
  ar_blocks_client_.cancelGoal();
  statusBar()->showMessage("Build was cancelled.");
}

void ARBlocksInterface::buildBtnHandler()
{
  // Final update
  goal_structure_.goal_structure.layers[current_layer_number_-1] = block_scene_->populateLayer();
  
  // Prepare the final goal for build
  ar_blocks::BuildStructureGoal gpub = goal_structure_;
  gpub.goal_structure.header.frame_id = "/base";
  gpub.goal_structure.header.stamp = ros::Time::now();
  for(int i = 0; i < gpub.goal_structure.layers.size(); i++) {
    ar_blocks::Layer& layer = gpub.goal_structure.layers[i];
    for(int j = 0; j < gpub.goal_structure.layers[i].blocks.size(); j++) {
      layer.blocks[j].pose_stamped.header.stamp = ros::Time::now();
      layer.blocks[j].pose_stamped.pose.position.x /= ratio;
      layer.blocks[j].pose_stamped.pose.position.y /= -ratio;
      layer.blocks[j].pose_stamped.pose.position.y += table_dim_y;
    }
  }
  
  // Convert the block locations to be relative to the table
  
  ar_blocks_client_.sendGoal( gpub,
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
  
  int percent = ((int)(total_blocks) != 0 && total_blocks >= 0.0) ? (int)(valid_blocks/total_blocks) : 0;
  
  build_progress_->setValue(percent);
}

} // namespace nxr
