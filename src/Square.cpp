#include <ar_blocks/Square.h>
#include <QPainter>

Square::Square(qreal x, qreal y) : QGraphicsItem()
{
  setPos(x, y);
  setFlags( QGraphicsItem::ItemIsMovable |
            QGraphicsItem::ItemIsSelectable |
            QGraphicsItem::ItemIgnoresTransformations );
}

void Square::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  painter->setRenderHint(QPainter::Antialiasing);
  painter->setPen(QPen(Qt::black, 2));
  painter->drawRect(-5, -5, 10, 10);
}
