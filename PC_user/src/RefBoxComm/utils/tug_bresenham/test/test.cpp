#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <tug_bresenham/basic_circle_iterator.h>
#include <tug_bresenham/circle_iterator.h>
#include <tug_bresenham/line_iterator.h>
#include <vector>



typedef std::vector<std::pair<int, int> > CoordsVector;

class CoordsBuilder
{
public:
  CoordsBuilder(int x, int y)
  {
    coords_.push_back(std::make_pair(x, y));
  }

  CoordsBuilder& operator()(int x, int y)
  {
    coords_.push_back(std::make_pair(x, y));
    return *this;
  }

  operator CoordsVector&()
  {
    return coords_;
  }

protected:
  CoordsVector coords_;
};

static void expectCoords(tug_bresenham::BasicCircleIterator& bci, const CoordsVector& expected_coords)
{
  size_t i = 0;
  for (; i < expected_coords.size() && !bci.isFinished(); ++i, ++bci)
  {
    //std::cerr << "{ x: " << bci.getDx() << "; y: " << bci.getDy() << " }" << std::endl;
    EXPECT_EQ(expected_coords[i].first, bci.getDx());
    EXPECT_EQ(expected_coords[i].second, bci.getDy());
    EXPECT_FALSE(bci.isFinished());
  }
  EXPECT_EQ(i, expected_coords.size());
  EXPECT_TRUE(bci.isFinished());
}

static void doTestRadius(int radius, int max_iterations, bool& is_finished, int& actual_iterations)
{
  tug_bresenham::BasicCircleIterator bci(radius);
  int last_dx = radius;
  int last_dy = -1;
  int i = 0;
  for (; i < max_iterations && !bci.isFinished(); ++i, ++bci)
  {
    int x_distance = bci.getDx() - last_dx;
    int y_distance = bci.getDy() - last_dy;
    EXPECT_LE(-1, x_distance);
    EXPECT_LE(x_distance, 1);
    EXPECT_LE(-1, y_distance);
    EXPECT_LE(y_distance, 1);
    int pixel_distance = std::abs(x_distance) + std::abs(y_distance);
    EXPECT_LE(1, pixel_distance);
    EXPECT_LE(pixel_distance, 2);

    //std::cerr << "{ x: " << bci.getDx() << "; y: " << bci.getDy() << " }" << std::endl;
    double actual_radius = std::sqrt(std::pow(bci.getDx(), 2.0) +
                                     std::pow(bci.getDy(), 2.0));
    EXPECT_NEAR(radius, actual_radius, 0.5);

    last_dx = bci.getDx();
    last_dy = bci.getDy();
  }
  is_finished = bci.isFinished();
  actual_iterations = i;
}

static void testRadius(int radius)
{
  bool is_finished = false;
  int actual_iterations = 0;
  doTestRadius(radius, std::numeric_limits<int>::max(), is_finished, actual_iterations);
  EXPECT_GT(actual_iterations, radius); // Iterator must return more than <radius> points
  EXPECT_LE(actual_iterations, 2L * radius); // Iterator must return less than 2*<radius> points
  EXPECT_TRUE(is_finished);
}

static void testRadius(int radius, int iterations)
{
  bool is_finished = true;
  int actual_iterations = 0;
  doTestRadius(radius, iterations, is_finished, actual_iterations);
  EXPECT_EQ(actual_iterations, iterations);
  EXPECT_FALSE(is_finished);
}

static void testExpectedException(int radius)
{
  try
  {
    tug_bresenham::BasicCircleIterator bci(radius);
    ADD_FAILURE() << "Expected exception was not thrown";
  }
  catch (std::invalid_argument&)
  {
    // Expected exception.
  }
  catch (...)
  {
    ADD_FAILURE() << "Caught unexpected exception";
  }
}

TEST(BasicCircleIterator, testCircleNegativeRadius)
{
  testExpectedException(std::numeric_limits<int>::min());
  testExpectedException(-2);
  testExpectedException(-1);
}

TEST(BasicCircleIterator, testCircle0)
{
  testExpectedException(0);
}

TEST(BasicCircleIterator, testCircle1)
{
  tug_bresenham::BasicCircleIterator bci(1);
  expectCoords(bci, CoordsBuilder(1, 0)(0, 1));
}

TEST(BasicCircleIterator, testCircle2)
{
  tug_bresenham::BasicCircleIterator bci(2);
  expectCoords(bci, CoordsBuilder(2, 0)(2, 1)(1, 2)(0, 2));
}

TEST(BasicCircleIterator, testCircle3)
{
  tug_bresenham::BasicCircleIterator bci(3);
  expectCoords(bci, CoordsBuilder(3, 0)(3, 1)(2, 2)(1, 3)(0, 3));
}

TEST(BasicCircleIterator, testCircleRadius1To100)
{
  for (int radius = 1; radius <= 100; ++radius)
  {
    testRadius(radius);
  }
}

TEST(BasicCircleIterator, testCircleRadiusLarge)
{
  testRadius(std::numeric_limits<int>::max(), 10000);
}

TEST(LineIterator, test_advanceDelta_onlyX)
{
  tug_bresenham::LineIterator iterator(0, 0, 10, 0);
  iterator += 5;

  EXPECT_EQ(5, iterator.getX());
  EXPECT_EQ(0, iterator.getY());
}

TEST(LineIterator, test_advanceDelta_onlyNegativeX)
{
  tug_bresenham::LineIterator iterator(0, 0, -10, 0);
  iterator += 5;

  EXPECT_EQ(-5, iterator.getX());
  EXPECT_EQ(0, iterator.getY());
}

TEST(LineIterator, test_advanceDelta_onlyY)
{
  tug_bresenham::LineIterator iterator(0, 0, 0, 10);
  iterator += 5;

  EXPECT_EQ(0, iterator.getX());
  EXPECT_EQ(5, iterator.getY());
}

TEST(LineIterator, test_advanceDelta_onlyNegativeY)
{
  tug_bresenham::LineIterator iterator(0, 0, 0, -10);
  iterator += 5;

  EXPECT_EQ(0, iterator.getX());
  EXPECT_EQ(-5, iterator.getY());
}

TEST(LineIterator, test_advanceDelta_sameXYDisplacement)
{
  tug_bresenham::LineIterator iterator(0, 0, 10, 10);
  iterator += 5;

  EXPECT_EQ(5, iterator.getX());
  EXPECT_EQ(5, iterator.getY());
}

TEST(LineIterator, test_advanceDelta_sameNegativeXYDisplacement)
{
  tug_bresenham::LineIterator iterator(0, 0, -10, -10);
  iterator += 5;

  EXPECT_EQ(-5, iterator.getX());
  EXPECT_EQ(-5, iterator.getY());
}

TEST(LineIterator, test_advanceDelta_sameXYDisplacementXPositiveYNegative)
{
  tug_bresenham::LineIterator iterator(0, 0, 10, -10);
  iterator += 5;

  EXPECT_EQ(5, iterator.getX());
  EXPECT_EQ(-5, iterator.getY());
}

TEST(LineIterator, test_advanceDelta_sameXYDisplacementXNegativeYPositive)
{
  tug_bresenham::LineIterator iterator(0, 0, -10, 10);
  iterator += 5;

  EXPECT_EQ(-5, iterator.getX());
  EXPECT_EQ(5, iterator.getY());
}

TEST(LineIterator, test_advanceDelta_moreXThenYDisplacement)
{
  tug_bresenham::LineIterator iterator(0, 0, 10, 5);
  iterator += 5;

  EXPECT_EQ(5, iterator.getX());
  EXPECT_EQ(3, iterator.getY());
}

TEST(LineIterator, test_advanceDelta_moreYThenXDisplacement)
{
  tug_bresenham::LineIterator iterator(0, 0, 5, 10);
  iterator += 5;

  EXPECT_EQ(3, iterator.getX());
  EXPECT_EQ(5, iterator.getY());
}

TEST(LineIterator, test_advanceDelta_completeLineMoreXThenY)
{
  tug_bresenham::LineIterator iterator(0, 0, 10, 5);

  tug_bresenham::LineIterator standard_iterator(0, 0, 10, 5);

  CoordsVector line_coords;
  CoordsVector standard_line_coords;

  iterator += 5;
  line_coords.push_back(std::make_pair(iterator.getX(), iterator.getY()));

  for (size_t i = 0; i < 5; ++i)
  {
    standard_iterator++;
  }
  standard_line_coords.push_back(std::make_pair(iterator.getX(), iterator.getY()));

  for (size_t i = 0; i < 5; ++i)
  {
    iterator++;
    standard_iterator++;

    line_coords.push_back(std::make_pair(iterator.getX(), iterator.getY()));
    standard_line_coords.push_back(std::make_pair(iterator.getX(), iterator.getY()));
  }

  EXPECT_TRUE(iterator.isAtEnd());

  EXPECT_EQ(standard_line_coords, line_coords);
}

TEST(LineIterator, test_advanceDelta_completeLineMoreYThenX)
{
  tug_bresenham::LineIterator iterator(0, 0, 5, 10);

  tug_bresenham::LineIterator standard_iterator(0, 0, 5, 10);

  CoordsVector line_coords;
  CoordsVector standard_line_coords;

  iterator += 5;
  line_coords.push_back(std::make_pair(iterator.getX(), iterator.getY()));

  for (size_t i = 0; i < 5; ++i)
  {
    standard_iterator++;
  }
  standard_line_coords.push_back(std::make_pair(iterator.getX(), iterator.getY()));

  for (size_t i = 0; i < 5; ++i)
  {
    iterator++;
    standard_iterator++;

    line_coords.push_back(std::make_pair(iterator.getX(), iterator.getY()));
    standard_line_coords.push_back(std::make_pair(iterator.getX(), iterator.getY()));
  }

  EXPECT_TRUE(iterator.isAtEnd());

  EXPECT_EQ(standard_line_coords, line_coords);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
