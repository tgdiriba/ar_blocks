#ifndef SCENE_H
#define SCENE_H

#include <QGraphicsScene>
#include <ar_blocks/Rectangle.h>
#include <ar_blocks/Structure.h>

class QGraphicsSceneMouseEvent;

namespace nxr {

class ARBlocksInterface;

static int scene_width = 630;
static int scene_height = 500;
static double table_dim_x = 121.602; 
static double table_dim_y = 60.8012;
static double x_ratio = scene_width/table_dim_x;
static double y_ratio = scene_height/table_dim_y;
static double ratio = (x_ratio < y_ratio) ? x_ratio : y_ratio;

class Scene : public QGraphicsScene
{
  Q_OBJECT
public:
  friend class ARBlocksInterface; 
  Scene(ARBlocksInterface *parent = (ARBlocksInterface*)0);
  ar_blocks::Structure blockStoreToStructure(); 
  double block_width;
  double block_length;
signals:
  void message(QString);
protected:
  void mousePressEvent(QGraphicsSceneMouseEvent*);
  void contextMenuEvent(QGraphicsSceneContextMenuEvent*);
private:
  std::vector< std::vector<Rectangle*> > block_store_;
  ARBlocksInterface* parent_;
  int id_count_;
};

} // namespace nxr

#endif // SCENE_H
