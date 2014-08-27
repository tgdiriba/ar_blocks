#ifndef BLOCK_H
#define BLOCK_H

#include <QtGui/QtGui>
#include <QtGui/QGraphicsItem>
#include <QtGui/QPainter>

class Square : public QGraphicsItem
{
public:
  Square(qreal, qreal);
  void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget*);
  QRectF boundingRect() const { return QRectF(-6.0, -6.0, 12.0, 12.0); }
};

#endif // BLOCK_H
