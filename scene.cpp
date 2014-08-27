#include <scene.h>
#include <block.h>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSceneContextMenuEvent>
#include <QMenu>
#include <QAction>

Scene::Scene() : QGraphicsScene()
{
  addLine(0,0,0,1,QPen(Qt::transparent,1));
}

void Scene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
  qreal x = event->scenePos().x();
  qreal y = event->scenePos().y();
  Block *block = dynamic_cast<Block*>(itemAt(x,y));
  if(event->button() == Qt::LeftButton) {
    if(block == 0 && event->button() == Qt::LeftButton) {
      addItem(new Block(x,y));
      emit message(QString("Block added at %1, %2").arg(x).arg(y));
    }
    
    QGraphicsScene::mousePressEvent(event);
  }
  else if(event->button() == Qt::RightButton) {
    if(block != 0) {
      removeItem(block);
      delete block;
    }
  }
}

void Scene::contextMenuEvent(QGraphicsSceneContextMenuEvent* event)
{
  qreal x = event->scenePos().x();
  qreal y = event->scenePos().y();
  
  Block *block = dynamic_cast<Block*>(itemAt(x,y));
  
  QMenu menu;
  QAction *deleteAction = menu.addAction("Delete Block");
  if(menu.exec(event->screenPos() ) == deleteAction) {
    removeItem(block);
    delete block;
    emit message(QString("Block deleted at %1, %2").arg(x).arg(y));
  }
}
