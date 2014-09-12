#include <ar_blocks/Scene.h>
#include <ar_blocks/Rectangle.h>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSceneContextMenuEvent>
#include <ar_blocks/ARBlocksInterface.h>
#include <QMenu>
#include <QAction>
#include <iostream>

namespace nxr {

Scene::Scene(ARBlocksInterface *parent) :
  QGraphicsScene(),
  parent_(parent),
  block_width(6.35),
  block_length(6.35),
  id_count_(1)
{
  addLine(0,0,0,1,QPen(Qt::transparent,1));
}

bool Scene::existsCollision()
{
  for(int i = 0; i < parent_->goal_structure_.goal_structure.layers.size(); i++) {
    ar_blocks::Layer &layer = parent_->goal_structure_.goal_structure.layers[i];
    for(int j = 0; j < layer.blocks.size(); j++) {
      return false;
    }
  }
}

void Scene::drawLayer(ar_blocks::Layer layer)
{
  for(int i = 0; i < layer.blocks.size(); i++) {
    addItem(
      new Rectangle(
        layer.blocks[i].pose_stamped.pose.position.x,
        layer.blocks[i].pose_stamped.pose.position.y, 
        layer.blocks[i].id, 
        layer.blocks[i].length, 
        layer.blocks[i].width
      )
    );
  }
}

ar_blocks::Layer Scene::populateLayer()
{
  ar_blocks::Layer l;
  QList<QGraphicsItem*> lit = items();

  for(int i = 0; i < lit.size(); i++) {
    if(lit[i]->type() == Rectangle::Type) {
      Rectangle *r = dynamic_cast<Rectangle*>(lit[i]);
      ar_blocks::Block b;
      b.length = r->length_;
      b.width = r->width_;
      b.height = r->length_;
      b.id = r->id_;
      
      b.pose_stamped.header.frame_id = "/base";
      b.pose_stamped.header.stamp = ros::Time::now();
      b.pose_stamped.pose.position.x = r->x();
      b.pose_stamped.pose.position.y = r->y();
      b.pose_stamped.pose.position.z = 0.0;
      b.pose_stamped.pose.orientation.x = 0.0;
      b.pose_stamped.pose.orientation.y = 0.0;
      b.pose_stamped.pose.orientation.z = 0.0;
      b.pose_stamped.pose.orientation.w = 1.0;
      
      l.blocks.push_back(b); 
    }
  }
  
  return l;
}

void Scene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
  qreal x = event->scenePos().x();
  qreal y = event->scenePos().y();
  Rectangle *block = dynamic_cast<Rectangle*>(itemAt(x,y));
  if(event->button() == Qt::LeftButton) {
    if(block == 0 && event->button() == Qt::LeftButton) {
      addItem(new Rectangle(x,y, id_count_, int(block_length*ratio), int(block_width*ratio)));
      id_count_++;

      parent_->statusBar()->showMessage(QString("Rectangle added at %1, %2").arg(x).arg(y));
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

} // namespace nxr
