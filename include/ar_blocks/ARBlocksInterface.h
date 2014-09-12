#ifndef ARBLOCKSINTERFACE_H
#define ARBLOCKSINTERFACE_H

#include <QMainWindow>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <map>
#include <cmath>
#include <ar_blocks/Scene.h>
#include <ar_blocks/Rectangle.h>
#include <ar_blocks/BuildStructureAction.h>

namespace nxr {

class ARBlocksInterface : public QMainWindow
{
  Q_OBJECT
  
  typedef actionlib::SimpleActionClient<ar_blocks::BuildStructureAction> Client;
public:
  friend class Scene;
  ARBlocksInterface();
  
private:
  // Qt
  QVBoxLayout *center_layout_;
  
  QHBoxLayout *viz2d_panel_;
  QHBoxLayout *viz2d_tools_;
  
  QHBoxLayout *block_size_layout_;
  QHBoxLayout *layer_count_layout_;
  QHBoxLayout *block_count_layout_;
  QHBoxLayout *stability_layout_;
  QHBoxLayout *build_layout_;
  
  QLabel *left_panel_title_;
  QComboBox *current_layer_cb_;
  QLabel *current_layer_;
  
  QLabel *block_size_label_;
  QLineEdit *block_size_input_;
  QLabel *block_size_units_;
  
  QLabel *layer_count_label_;
  QLabel *block_count_label_;
  QLabel *stability_label_;
  
  QLabel *layer_count_field_;
  QLabel *block_count_field_;
  QLabel *stability_field_;
  
  QPushButton *prev_layer_btn_;
  QPushButton *next_layer_btn_;
  QPushButton *remove_layer_btn_;
  QPushButton *add_layer_btn_;
  
  QPushButton *abort_btn_;
  QPushButton *build_btn_;
  QProgressBar *build_progress_;
  
  Scene *block_scene_;

  // ROS
  ros::NodeHandle nh_;
  Client ar_blocks_client_;
  
public slots:
  void previousLayerBtnHandler();
  void nextLayerBtnHandler();
  void removeLayerBtnHandler();
  void addLayerBtnHandler();
  void abortBtnHandler();
  void buildBtnHandler();
  
  void indexChangeHandler();
private:
  void arBlocksDoneCallback(const actionlib::SimpleClientGoalState &state,
                            const ar_blocks::BuildStructureResultConstPtr &result);
  void arBlocksActiveCallback();
  void arBlocksFeedbackCallback(const ar_blocks::BuildStructureFeedbackConstPtr &feedback);

  void drawStaticLayer(int layer_number);
  void drawDynamicLayer(int layer_number);
  
  double calculateStabilityMeasure();
  double lowerLayerStabilityMeasure(ar_blocks::Block, int l);
 
  void drawTable(); 
  void redrawScene();
  
  int current_layer_number_;
  int layer_count_;
  ar_blocks::BuildStructureGoal goal_structure_;
  
};

} // namespace nxr

#endif // ARBLOCKSINTERFACE_H
