#include <ar_blocks/Rectangle.h>
#include <QPainter>

namespace nxr {

Rectangle::Rectangle(qreal x, qreal y, int id, int length, int width) : 
  QGraphicsItem(),
  length_(length),
  width_(width),
  id_(id)
{
  setPos(x, y);
  setFlags( QGraphicsItem::ItemIsMovable |
            QGraphicsItem::ItemIsSelectable |
            QGraphicsItem::ItemIgnoresTransformations );
}

void Rectangle::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  painter->setRenderHint(QPainter::Antialiasing);
  painter->setPen(QPen(Qt::black, 1));
  painter->drawRect(-length_/2, -width_/2, length_, width_);
}

} // namespace nxr
