#include "Clipper.hpp"

#include "JsonSerializers.hpp"
#include <filesystem>

#define ErrorIf(expression, message) \
do                                   \
{                                    \
  if(expression)                     \
    __debugbreak();                  \
} while(false)                       

size_t FindPointIn(const Vec2& point, const PointContour& pointList, float epsilon = 0.01f)
{
  size_t result = pointList.size();
  for(size_t i = 0; i < pointList.size(); ++i)
  {
    if(Vec2::DistanceSq(point, pointList[i]) <= epsilon)
    {
      result = i;
      break;
    }
  }
  return result;
}

bool TestContour(const PointContour& input, const PointContour& expected, float epsilon = 0.01f)
{
  // Find the first point in the input list that matches the first point in the expected list
  size_t startIndex = FindPointIn(expected[0], input);
  if(startIndex >= input.size())
    return false;

  // Now simultaneously iterate both lists, making sure all the points are the same
  for(size_t i = 0; i < expected.size(); ++i)
  {
    size_t inputIndex = (startIndex + i) % input.size();
    Vec2 inputPoint = input[inputIndex];
    Vec2 expectedPoint = expected[i];
    if(Vec2::DistanceSq(inputPoint, expectedPoint) > epsilon)
      return false;
  }
  return true;
}

bool TestContours(const PointContourList& input, const PointContourList& expected, float epsilon = 0.01f)
{
  if(input.size() != expected.size())
    return false;

  // There's no easy way to match contours. Do a brute force approach of checking all contours for equality (n^2). 
  // This could be improved by removing contours in the expected list once they're matched.
  for(size_t i = 0; i < input.size(); ++i)
  {
    bool foundMatch = false;
    for(size_t j = 0; j < expected.size(); ++j)
    {
      foundMatch = TestContour(input[i], expected[j], epsilon);
      if(foundMatch)
        break;
    }
    if(!foundMatch)
      return false;
  }
  return true;
}

void LoadPoint(JsonLoader& loader, Vec2& point)
{
  size_t count = 0;
  loader.BeginArray(count);
  if(count != 2)
    __debugbreak();

  if(loader.BeginArrayItem(0))
  {
    loader.SerializePrimitive(point.x);
    loader.EndArrayItem();
  }
  if(loader.BeginArrayItem(1))
  {
    loader.SerializePrimitive(point.y);
    loader.EndArrayItem();
  }
}

void LoadContour(JsonLoader& loader, PointContour& points)
{
  size_t count = 0;
  loader.BeginArray(count);
  points.resize(count);
  for(size_t i = 0; i < count; ++i)
  {
    if(loader.BeginArrayItem(i))
    {
      LoadPoint(loader, points[i]);
      loader.EndArrayItem();
    }
  }
}

void LoadContourList(JsonLoader& loader, PointContourList& contours)
{
  size_t count = 0;
  loader.BeginArray(count);
  contours.resize(count);
  for(size_t i = 0; i < count; ++i)
  {
    if(loader.BeginArrayItem(i))
    {
      LoadContour(loader, contours[i]);
      loader.EndArrayItem();
    }
  }
}

void TestUnion(JsonLoader& loader, PointContour& polyList, PointContour& clipRegion)
{
  PointContour expected;
  if(loader.BeginMember("Union"))
  {
    LoadContour(loader, expected);
    loader.EndMember();
  }

  PointContour results;
  Clipper clipper;
  clipper.Union(polyList, clipRegion, results);
  bool passed = TestContour(results, expected);
  ErrorIf(!passed, "Failed");
}

void TestSubtraction(JsonLoader& loader, PointContour& polyList, PointContour& clipRegion)
{
  PointContourList expected;
  if(loader.BeginMember("Subtraction"))
  {
    LoadContourList(loader, expected);
    loader.EndMember();
  }

  PointContourList results;
  Clipper clipper;
  clipper.Subtract(polyList, clipRegion, results);
  bool passed = TestContours(results, expected);
  ErrorIf(!passed, "Failed");
}

void TestIntersection(JsonLoader& loader, PointContour& polyList, PointContour& clipRegion)
{
  PointContourList expected;
  if(loader.BeginMember("Intersection"))
  {
    LoadContourList(loader, expected);
    loader.EndMember();
  }

  PointContourList results;
  Clipper clipper;
  clipper.Intersect(polyList, clipRegion, results);
  bool passed = TestContours(results, expected);
  ErrorIf(!passed, "Failed");
}

void RunTestFile(const std::filesystem::path& filePath)
{
  JsonLoader loader;
  loader.LoadFromFile(filePath.string());

  PointContour polygon;
  PointContour clipRegion;
  if(loader.BeginMember("Polygon"))
  {
    LoadContour(loader, polygon);
    loader.EndMember();
  }
  if(loader.BeginMember("ClipRegion"))
  {
    LoadContour(loader, clipRegion);
    loader.EndMember();
  }
  TestUnion(loader, polygon, clipRegion);
  TestSubtraction(loader, polygon, clipRegion);
  TestIntersection(loader, polygon, clipRegion);
}

void RunTests(const std::filesystem::path& path)
{
  for(auto it : std::filesystem::directory_iterator(path))
  {
    if(it.is_directory())
    {
      RunTests(it.path());
      continue;
    }
    RunTestFile(it.path());
  }
}

int main()
{
  std::filesystem::path dataPath = "Data";
  RunTests(dataPath);

  return 0;
}
