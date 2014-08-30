#ifndef SCENE_H
#define SCENE_H

#include <QGraphicsScene>
#include <ar_blocks/Rectangle.h>

class QGraphicsSceneMouseEvent;

namespace nxr {

class ARBlocksInterface;

class Scene : public QGraphicsScene
{
  Q_OBJECT
public:
  friend class ARBlocksInterface; 
  Scene(ARBlocksInterface *parent = (ARBlocksInterface*)0);
signals:
  void message(QString);
protected:
  void mousePressEvent(QGraphicsSceneMouseEvent*);
  void contextMenuEvent(QGraphicsSceneContextMenuEvent*);
private:
  std::map<int, Rectangle*> block_ids_;
  std::vector< std::vector<int> > block_store_;
  ARBlocksInterface* parent_;
  int id_count_;
};

} // namespace nxr

#endif // SCENE_H
