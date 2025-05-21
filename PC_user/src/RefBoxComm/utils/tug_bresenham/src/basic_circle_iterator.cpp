#include <tug_bresenham/basic_circle_iterator.h>
#include <stdexcept>



namespace tug_bresenham
{

BasicCircleIterator::BasicCircleIterator(int radius)
  : dx_(radius), dy_(0), error_(static_cast<ErrorInt>(1 - radius) * 2)
{
  if (radius < 1)
  {
    throw std::invalid_argument(std::string(__func__) + ": radius must be >= 1");
  }
  if ((error_ / 2) != (1 - radius))
  {
    throw std::invalid_argument(std::string(__func__) + ": radius is too large for computations");
  }
}

void BasicCircleIterator::operator++()
{
  advance();
}

void BasicCircleIterator::operator++(int)
{
  advance();
}

bool BasicCircleIterator::isFinished() const
{
  return dx_ < 0;
}

void BasicCircleIterator::advance()
{
  if (!isFinished())
  {
    ErrorInt error_2 = 2 * error_ + 2;

    ErrorInt dx_error = 3 - static_cast<ErrorInt>(2) * dx_;
    if (error_2 > dx_error)
    {
      --dx_;
      error_ += dx_error;
    }

    ErrorInt dy_error = 3 + static_cast<ErrorInt>(2) * dy_;
    if (error_2 < dy_error)
    {
      ++dy_;
      error_ += dy_error;
    }
  }
}

}
