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
  block_store_.push_back( std::vector<Rectangle*>() );
}

ar_blocks::Structure Scene::blockStoreToStructure()
{
  ar_blocks::Structure t;
  t.layers = std::vector<ar_blocks::Layer>(block_store_.size());
  for(int i = 0; i < block_store_.size(); i++) {
    for(int j = 0; j < block_store_[i].size(); j++) {
      ar_blocks::Block b;
      b.length = block_store_[i][j]->length_;
      b.width = block_store_[i][j]->width_;
      b.height = b.length;
      b.id = block_store_[i][j]->id_;
      
      b.pose_stamped.header.frame_id = "/base";
      b.pose_stamped.header.stamp = ros::Time::now();
      b.pose_stamped.pose.position.x = block_store_[i][j]->x() - int(table_dim_x*ratio/2); // CALCULATE position relative to the center of the table
      b.pose_stamped.pose.position.y = block_store_[i][j]->y() + int(table_dim_y*ratio/2);
      b.pose_stamped.pose.position.z = 0.0;
      b.pose_stamped.pose.orientation.x = 0.0;
      b.pose_stamped.pose.orientation.y = 0.0;
      b.pose_stamped.pose.orientation.z = 0.0;
      b.pose_stamped.pose.orientation.w = 1.0;
       
      t.layers[i].blocks.push_back(b);
    }
  }
  
  return t;
}

void Scene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
  qreal x = event->scenePos().x();
  qreal y = event->scenePos().y();
  Rectangle *block = dynamic_cast<Rectangle*>(itemAt(x,y));
  if(event->button() == Qt::LeftButton) {
    if(block == 0 && event->button() == Qt::LeftButton) {
      block_store_[parent_->current_layer_number_-1].push_back(new Rectangle(x,y, id_count_, int(block_length*ratio), int(block_width*ratio)));
      addItem(block_store_[parent_->current_layer_number_-1].back());
      id_count_++;
      parent_->statusBar()->showMessage(QString("Rectangle added at %1, %2").arg(x).arg(y));
    }
    
    QGraphicsScene::mousePressEvent(event);
  }
  else if(event->button() == Qt::RightButton) {
    if(block != 0) {
      // Perform a linear search for the Rectangle pointer to make sure it exists
      std::vector<Rectangle*>::iterator search_block = find(block_store_[parent_->current_layer_number_-1].begin(), block_store_[parent_->current_layer_number_-1].end(), block);
      std::vector<Rectangle*>::iterator end = block_store_[parent_->current_layer_number_-1].end();
      if(search_block != end) {
        block_store_[parent_->current_layer_number_-1].erase(search_block);
        parent_->statusBar()->showMessage(QString("Rectangle removed at %1, %2").arg(x).arg(y));
      }
      removeItem(block);
      delete block;
    }
  }

  QList<QGraphicsItem*> lit = items();
  for(int i = 0; i < lit.size(); i++) {
    std::cout << (lit[i]->type() == Rectangle::Type) << std::endl;
  }

}

void Scene::contextMenuEvent(QGraphicsSceneContextMenuEvent* event)
{
  /*qreal x = event->scenePos().x();
  qreal y = event->scenePos().y();
  
  Rectangle *block = dynamic_cast<Rectangle*>(itemAt(x,y));
  
  QMenu menu;
  QAction *deleteAction = menu.addAction("Delete Rectangle");
  if(menu.exec(event->screenPos() ) == deleteAction) {
    removeItem(block);
    delete block;
    parent_->statusBar()->showMessage(QString("Rectangle added at %1, %2").arg(x).arg(y));
  }*/
}

} // namespace nxr
