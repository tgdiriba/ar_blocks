#include <block.h>
#include <QPainter>

Block::Block(qreal x, qreal y) : QGraphicsItem()
{
  setPos(x, y);
  setFlags( QGraphicsItem::ItemIsMovable |
            QGraphicsItem::ItemIsSelectable |
            QGraphicsItem::ItemIgnoresTransformations );
}

void Block::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  painter->setRenderHint(QPainter::Antialiasing);
  painter->setPen(QPen(Qt::black, 2));
  painter->drawRect(-5, -5, 10, 10);
}
