#include <ar_blocks/Geometry.h>

namespace nxr {

using namespace std;

bool pointInRect(Rect r, Point p)
{
  return (p.x >= r.point.x) && (p.y > r.point.y) &&
         (p.x <= r.point.x + r.area.length) && (p.y <= r.point.y + r.area.width);
}

bool pointInRect(Rect r, vector<Point> pv) 
{
  for(int i = 0; i < pv.size(); i++) {
    if(pointInRect(r, pv[i])) return true;
  }
  return false;
}

} // namespace nxr
