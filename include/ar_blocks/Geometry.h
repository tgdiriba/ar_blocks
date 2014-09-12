#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>

namespace nxr {

struct Prism {
  double length;
  double width;
  double height;
};

struct Area {
  double length;
  double width;
};

struct Point {
  double x;
  double y;
};

struct Rect {
  Point point;
  Area area;
};

bool pointInRect(Rect r, Point p); 
bool pointInRect(Rect r, std::vector<Point> pv);

} // namespace nxr

#endif // GEOMETRY_H
