#ifndef _TUG_BRESENHAM__CIRCLE_ITERATOR_H_
#define _TUG_BRESENHAM__CIRCLE_ITERATOR_H_

#include <cstdlib>
#include <tug_bresenham/basic_circle_iterator.h>



namespace tug_bresenham
{

/**
 * Implementation of the Bresenham circle algorithm in iterator form (more
 * simple-to-use version than BasicCircleIterator).
 *
 * Due to the nature of the algorithm, results are returned for each quadrant
 * in turns. Note that (x, y) will be invalid values once isFinished() returns
 * true.
 */
class CircleIterator
{
public:
  CircleIterator(int center_x, int center_y, int radius);

  void operator++();
  void operator++(int);

  int getX() const;
  int getY() const;

  bool isFinished() const;

protected:
  static const size_t NUM_QUADRANTS = 4;

  void advance();
  void computeCoordinates();

  int center_x_;
  int center_y_;
  int x_[NUM_QUADRANTS];
  int y_[NUM_QUADRANTS];
  size_t quadrant_;
  BasicCircleIterator basic_it_;
};

}

#endif
