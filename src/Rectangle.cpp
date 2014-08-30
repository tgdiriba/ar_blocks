#include <ar_blocks/Rectangle.h>
#include <QPainter>

Rectangle::Rectangle(qreal x, qreal y, int width, int height) : 
  QGraphicsItem(),
  width_(width),
  height_(height)
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
  painter->drawRect(-width_/2, -height_/2, width_, height_);
}
