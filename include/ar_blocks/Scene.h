#ifndef SCENE_H
#define SCENE_H

#include <QGraphicsScene>
#include <ar_blocks/Rectangle.h>
#include <ar_blocks/Structure.h>
#include <ar_blocks/Layer.h>
#include <ar_blocks/Block.h>
#include <ar_blocks/Geometry.h>

class QGraphicsSceneMouseEvent;

namespace nxr {

class ARBlocksInterface;

static double table_dim_x = 121.602; 
static double table_dim_y = 60.8012;
static int scene_width = 630;
// static int scene_height = 500;
static double x_ratio = scene_width/table_dim_x;
static int scene_height = int(x_ratio * table_dim_y);
static double y_ratio = scene_height/table_dim_y;
static double ratio = (x_ratio < y_ratio) ? x_ratio : y_ratio;

class Scene : public QGraphicsScene
{
  Q_OBJECT
public:
  friend class ARBlocksInterface; 
  Scene(ARBlocksInterface *parent = (ARBlocksInterface*)0);
  bool existsCollision();
  ar_blocks::Layer populateLayer();
  void drawLayer(ar_blocks::Layer layer);
  double block_width;
  double block_length;
signals:
  void message(QString);
protected:
  void mousePressEvent(QGraphicsSceneMouseEvent*);
private:
  ARBlocksInterface* parent_;
  int id_count_;
};

} // namespace nxr

#endif // SCENE_H
