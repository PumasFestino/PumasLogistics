#include <tug_bresenham/circle_iterator.h>



namespace tug_bresenham
{

CircleIterator::CircleIterator(int center_x, int center_y, int radius)
  : center_x_(center_x), center_y_(center_y), quadrant_(0), basic_it_(radius)
{
  computeCoordinates();
}

void CircleIterator::operator++()
{
  advance();
}

void CircleIterator::operator++(int)
{
  advance();
}

int CircleIterator::getX() const
{
  return x_[quadrant_];
}

int CircleIterator::getY() const
{
  return y_[quadrant_];
}

bool CircleIterator::isFinished() const
{
  return basic_it_.isFinished();
}

void CircleIterator::advance()
{
  if (!isFinished())
  {
    ++quadrant_;
    if (quadrant_ >= NUM_QUADRANTS)
    {
      quadrant_ = 0;
      ++basic_it_;
      computeCoordinates();
    }
  }
}

void CircleIterator::computeCoordinates()
{
  x_[0] = center_x_ + basic_it_.getDx();
  y_[0] = center_y_ + basic_it_.getDy();
  x_[1] = center_x_ - basic_it_.getDx();
  y_[1] = center_y_ + basic_it_.getDy();
  x_[2] = center_x_ - basic_it_.getDx();
  y_[2] = center_y_ - basic_it_.getDy();
  x_[3] = center_x_ + basic_it_.getDx();
  y_[3] = center_y_ - basic_it_.getDy();
}

}
