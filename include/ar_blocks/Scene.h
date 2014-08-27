#ifndef SCENE_H
#define SCENE_H

#include <QGraphicsScene>

class QGraphicsSceneMouseEvent;

class Scene : public QGraphicsScene
{
  Q_OBJECT
public:
  Scene();
signals:
  void message(QString);
protected:
  void mousePressEvent(QGraphicsSceneMouseEvent*);
  void contextMenuEvent(QGraphicsSceneContextMenuEvent*);
};

#endif // SCENE_H
