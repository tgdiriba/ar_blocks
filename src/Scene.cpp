#include <ar_blocks/Scene.h>
#include <ar_blocks/Rectangle.h>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSceneContextMenuEvent>
#include <ar_blocks/ARBlocksInterface.h>
#include <QMenu>
#include <QAction>

namespace nxr {

Scene::Scene(ARBlocksInterface *parent) :
  QGraphicsScene(),
  parent_(parent),
  id_count_(0)
{
  addLine(0,0,0,1,QPen(Qt::transparent,1));
}

void Scene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
  qreal x = event->scenePos().x();
  qreal y = event->scenePos().y();
  Rectangle *block = dynamic_cast<Rectangle*>(itemAt(x,y));
  if(event->button() == Qt::LeftButton) {
    if(block == 0 && event->button() == Qt::LeftButton) {
      block_ids_[id_count_] = new Rectangle(x,y);
      addItem(block_ids_[id_count_]);
      block_store_[parent_->current_layer_number_].push_back(id_count_);
      id_count_++;
      parent_->statusBar()->showMessage(QString("Rectangle added at %1, %2").arg(x).arg(y));
    }
    
    QGraphicsScene::mousePressEvent(event);
  }
  else if(event->button() == Qt::RightButton) {
    if(block != 0) {
      // Perform a linear search for the Rectangle pointer to make sure it exists
      std::map<int, Rectangle*>::iterator p_id = block_ids_.end();
      std::map<int, Rectangle*>::iterator it = block_ids_.begin();
      std::map<int, Rectangle*>::iterator end = block_ids_.end();
      for( ; it != end; it++) {
        if(it->second == block) {
          p_id = it;
          break;
        }
      }
     
      // Remove the  
      if(p_id != block_ids_.end()) {
        std::vector<int>::iterator removed_block = find(block_store_[parent_->current_layer_number_].begin(), block_store_[parent_->current_layer_number_].end(), p_id->first);
        if(removed_block != block_store_[parent_->current_layer_number_].end()) {
          block_store_[parent_->current_layer_number_].erase(removed_block);
        }
        block_ids_.erase(p_id);
        parent_->statusBar()->showMessage(QString("Rectangle added at %1, %2").arg(x).arg(y));
      }
      removeItem(block);
      delete block;
    }
  }
}

void Scene::contextMenuEvent(QGraphicsSceneContextMenuEvent* event)
{
  qreal x = event->scenePos().x();
  qreal y = event->scenePos().y();
  
  Rectangle *block = dynamic_cast<Rectangle*>(itemAt(x,y));
  
  QMenu menu;
  QAction *deleteAction = menu.addAction("Delete Rectangle");
  if(menu.exec(event->screenPos() ) == deleteAction) {
    removeItem(block);
    delete block;
    parent_->statusBar()->showMessage(QString("Rectangle added at %1, %2").arg(x).arg(y));
  }
}

} // namespace nxr
