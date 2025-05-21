#include <tug_bresenham/line_iterator.h>
#include <cmath>



namespace tug_bresenham
{

LineIterator::LineIterator(int start_x, int start_y, int end_x, int end_y)
  : x_(start_x), y_(start_y), end_x_(end_x), end_y_(end_y), beyond_end_(false)
{
  dx_ = std::abs(end_x - start_x);
  dy_ = -std::abs(end_y - start_y);

  sx_ = (start_x < end_x) ? 1 : -1;
  sy_ = (start_y < end_y) ? 1 : -1;

  error_ = dx_ + dy_;
}

void LineIterator::operator++()
{
  advance();
}

void LineIterator::operator++(int)
{
  advance();
}

void LineIterator::operator+=(int delta)
{
  //TODO find method without case distinction

  int x_advance = 0;
  int y_advance = 0;
  if (dx_ == 0)
  {
    y_advance = delta;
  }
  else if (dy_ == 0)
  {
    x_advance = delta;
  }
  else
  {
    if (dx_ >= - dy_)
    {
      x_advance = delta;
      double dy_dx = static_cast<double>(-dy_) / static_cast<double>(dx_);
      y_advance = static_cast<int>(round(dy_dx * static_cast<double>(x_advance)));
    }
    else
    {
      y_advance = delta;
      double dx_dy = static_cast<double>(dx_) / static_cast<double>(-dy_);
      x_advance = static_cast<int>(round(dx_dy * static_cast<double>(y_advance)));
    }
  }

  error_ += x_advance * dy_ + y_advance * dx_;

  x_ = x_advance * sx_;
  y_ = y_advance * sy_;
}

bool LineIterator::isAtEnd() const
{
  return (x_ == end_x_) && (y_ == end_y_);
}

bool LineIterator::isBeyondEnd() const
{
  return beyond_end_;
}

void LineIterator::advance()
{
  if (isAtEnd())
  {
    beyond_end_ = true;
  }

  int error_2 = 2 * error_;

  if (error_2 > dy_)
  {
    error_ += dy_;
    x_ += sx_;
  }

  if (error_2 < dx_)
  {
    error_ += dx_;
    y_ += sy_;
  }
}

}
