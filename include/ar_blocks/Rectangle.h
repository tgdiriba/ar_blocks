#ifndef RECTANGLE_H
#define RECTANGLE_H

#include <QtGui/QtGui>
#include <QtGui/QGraphicsItem>
#include <QtGui/QPainter>

namespace nxr {

class Rectangle : public QGraphicsItem
{
public:

  enum { Type = UserType + 1 };

  Rectangle(qreal, qreal, int id=-1, int length = 10, int width = 10);
  void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget*);
  QRectF boundingRect() const { return QRectF(-length_/2 - 1.0, -width_/2 - 1.0, length_ + 2.0, width_ + 2.0); }
  int type() const
  {
    return Type;
  }
  int id_;
  int length_;
  int width_;
};

} // namespace nxr

#endif // RECTANGLE_H
