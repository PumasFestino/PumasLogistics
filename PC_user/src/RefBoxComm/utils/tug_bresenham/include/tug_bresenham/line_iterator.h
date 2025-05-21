#ifndef _TUG_BRESENHAM__LINE_ITERATOR_H_
#define _TUG_BRESENHAM__LINE_ITERATOR_H_

namespace tug_bresenham
{

/**
 * Implementation of the compact Bresenham line algorithm in iterator form.
 *
 * Note that the iterator can be advanced beyond the end point to extrapolate
 * the line.
 */
class LineIterator
{
public:
  LineIterator(int start_x, int start_y, int end_x, int end_y);

  void operator++();
  void operator++(int);
  void operator+=(int delta);

  inline int getX() const
  {
    return x_;
  }

  inline int getY() const
  {
    return y_;
  }

  bool isAtEnd() const;
  bool isBeyondEnd() const;

protected:
  void advance();

  int x_;
  int y_;
  int end_x_;
  int end_y_;
  int dx_;
  int dy_;
  int sx_;
  int sy_;
  int error_;
  bool beyond_end_;
};

}

#endif
