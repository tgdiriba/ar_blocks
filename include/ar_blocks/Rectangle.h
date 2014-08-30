#ifndef BLOCK_H
#define BLOCK_H

#include <QtGui/QtGui>
#include <QtGui/QGraphicsItem>
#include <QtGui/QPainter>

class Rectangle : public QGraphicsItem
{
public:
  Rectangle(qreal, qreal, int width = 10, int height = 10);
  void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget*);
  QRectF boundingRect() const { return QRectF(-width_/2 - 1.0, -height_/2 - 1.0, width_ + 2.0, height_ + 2.0); }
private:
  int width_;
  int height_;
};

#endif // BLOCK_H
