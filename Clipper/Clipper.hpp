#pragma once

#include "Vector2.hpp"
#include <vector>

template <typename T, typename...Extra>
using Array = std::vector<T, Extra...>;

enum class ClipVertexClassification
{
  None,
  OutToIn,
  InToOut,
  Inside,
  Outside
};

enum class ClipVertexSearchDirection
{
  Backward,
  Forwards
};

float Cross2d(const Vec2& lhs, const Vec2& rhs);
float SignedArea(const Vec2& a, const Vec2& b, const Vec2& c);
// Finds the interesction of the given two lines. The resultant t-value for the first line (line0).
// If there was an intersection, then the the flags for each line are filled out to indicate if it went
// from inside to out, or the opposite (where the inside is determined using the right-hand rule).
float ComputeIntersectionPoint(const Vec2& start0, const Vec2& end0, const Vec2& start1, const Vec2& end1, ClipVertexClassification& line0Flags, ClipVertexClassification& line1Flags);
ClipVertexSearchDirection FlipSearchDirection(ClipVertexSearchDirection direction);

//-------------------------------------------------------------------ClipVertex
struct ClipVertex
{
  Vec2 mPoint;
  ClipVertexClassification mClassification = ClipVertexClassification::None;
  bool mVisited = false;
  ClipVertex* mTwin = nullptr;
  ClipVertex* mPrev = nullptr;
  ClipVertex* mNext = nullptr;

  // Traverses the vertex list loop, processing a callback on each vertex. Callback returns false if the iterations should stop.
  // The given callback is expected to modify the next vertex if it's not the next pointer of the original vertex.
  template <typename Callback>
  static void Traverse(ClipVertex* v, Callback callback)
  {
    ClipVertex* begin = v;
    do
    {
      // Cache the next vertex in-case any new vertices are created while processing this vertex.
      ClipVertex* nextVert = v->mNext;
      if(!callback(v, nextVert))
        return;
      v = nextVert;
    } while(v != begin);
  }
  // Given a vertex walk it's twin vertex's list until it meets back up with the original list.
  template <typename Predicate, typename Callback>
  static ClipVertex* WalkTwinList(ClipVertex* vertex, Predicate predicate, Callback callback)
  {
    ClipVertex* twin = vertex->mTwin;
    if(twin == nullptr)
      return vertex;

    do
    {
      twin = predicate(twin);
      callback(twin);

    } while(twin->mTwin == nullptr);
    return twin->mTwin;
  }
  template <typename Callback>
  static ClipVertex* WalkTwinListForwards(ClipVertex* vertex, Callback callback)
  {
    auto predicate = [](ClipVertex* v) { return v->mNext; };
    return ClipVertex::WalkTwinList(vertex, predicate, callback);
  }

  template <typename Callback>
  static ClipVertex* WalkTwinListBackwards(ClipVertex* vertex, Callback callback)
  {
    auto predicate = [](ClipVertex* v) { return v->mPrev; };
    return ClipVertex::WalkTwinList(vertex, predicate, callback);
  }
  static ClipVertex* FindFirstOf(ClipVertex* vertexList, ClipVertexClassification classification);
  static ClipVertex* FindFirstIntersection(ClipVertex* vertexList);
  ClipVertex* GetNext(ClipVertexSearchDirection direction);
};

//-------------------------------------------------------------------ClipVertexList
struct ClipVertexList
{
  ClipVertexList() {}
  ClipVertexList(ClipVertex* vertex) : mHead(vertex) {}
  ~ClipVertexList();

  ClipVertex* mHead = nullptr;
};

//-------------------------------------------------------------------PointContour
struct PointContour : public Array<Vec2>
{
  typedef Array<Vec2> BaseType;
  using BaseType::BaseType;
};

//-------------------------------------------------------------------PointContourList
struct PointContourList : public Array<PointContour>
{
  typedef Array<PointContour> BaseType;
  using BaseType::BaseType;
};

//-------------------------------------------------------------------Clipper
struct Clipper
{
  // Converts the given points into a vertex list
  void BuildVertexList(const PointContour& points, ClipVertexList& result);
  // Classifies each vertex in the given list as being inside or outside.
  // This assumes that the list has already been clipped so that intersection points are tagged.
  void ClassifyVertices(ClipVertexList& vertices);
  // Clips the given edge against the provided clip polygon. Intersection point vertices are inserted into each polygon list.
  void ClipEdges(ClipVertex* start, ClipVertex* end, ClipVertexList& clipRegion);
  // Clips the provided polygon against the clip region polygon, creating all intersection points.
  // Both polygons are assumed to not contain self-intersections.
  void ClipPolygon(ClipVertexList& polygonToClip, ClipVertexList& clipRegion);
  // Convertex the given point lists into two clipped lists, full of all the intersection points and classifications.
  void BuildClipList(const PointContour& polygonPoints, const PointContour& clipRegionPoints, ClipVertexList& polyList, ClipVertexList& clipRegionList);
  void Union(ClipVertexList& polygon, PointContour& results);
  void Subtract(ClipVertexList& polygon, PointContourList& contours);
  void Intersect(ClipVertexList& polygon, PointContourList& contours);

  void Union(const PointContour& polygonPoints, const PointContour& clipRegion, PointContour& results);
  void Subtract(const PointContour& polygonPoints, const PointContour& clipRegion, PointContourList& contours);
  void Intersect(const PointContour& polygonPoints, const PointContour& clipRegion, PointContourList& contours);
};
