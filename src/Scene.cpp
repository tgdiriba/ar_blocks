#include <ar_blocks/Scene.h>
#include <ar_blocks/Square.h>
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
  Square *block = dynamic_cast<Square*>(itemAt(x,y));
  if(event->button() == Qt::LeftButton) {
    if(block == 0 && event->button() == Qt::LeftButton) {
      addItem(new Square(x,y));
      emit message(QString("Square added at %1, %2").arg(x).arg(y));
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
  
  Square *block = dynamic_cast<Square*>(itemAt(x,y));
  
  QMenu menu;
  QAction *deleteAction = menu.addAction("Delete Square");
  if(menu.exec(event->screenPos() ) == deleteAction) {
    removeItem(block);
    delete block;
    emit message(QString("Square deleted at %1, %2").arg(x).arg(y));
  }
}
