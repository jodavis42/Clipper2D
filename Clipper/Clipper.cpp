#include "Clipper.hpp"

#include <algorithm>

float Cross2d(const Vec2& lhs, const Vec2& rhs)
{
  return lhs.x * rhs.y - lhs.y * rhs.x;
}

float SignedArea(const Vec2& a, const Vec2& b, const Vec2& c)
{
  Vec2 ac = a - c;
  Vec2 bc = b - c;
  return Cross2d(ac, bc);
}

float ComputeIntersectionPoint(const Vec2& start0, const Vec2& end0, const Vec2& start1, const Vec2& end1, ClipVertexClassification& line0Flags, ClipVertexClassification& line1Flags)
{
  line1Flags = line0Flags = ClipVertexClassification::None;
  float a1 = SignedArea(start0, end0, end1);
  float a2 = SignedArea(start0, end0, start1);
  float a3 = SignedArea(start1, end1, start0);
  float a4 = a3 + a2 - a1;
  if(a1 * a2 > 0)
    return -1;
  if(a3 * a4 < 0)
  {
    line0Flags = a3 < 0 ? ClipVertexClassification::InToOut : ClipVertexClassification::OutToIn;
    line1Flags = a2 < 0 ? ClipVertexClassification::InToOut : ClipVertexClassification::OutToIn;
    return a3 / (a3 - a4);
  }
  return -1;
}

ClipVertexSearchDirection FlipSearchDirection(ClipVertexSearchDirection direction)
{
  if(direction == ClipVertexSearchDirection::Forwards)
    return ClipVertexSearchDirection::Backward;
  return ClipVertexSearchDirection::Forwards;
}

//-------------------------------------------------------------------ClipVertex
ClipVertex* ClipVertex::FindFirstOf(ClipVertex* vertexList, ClipVertexClassification classification)
{
  ClipVertex* result = nullptr;
  ClipVertex::Traverse(vertexList, [&result, classification](ClipVertex* vertex, ClipVertex*& nextVertex)
  {
    if(vertex->mClassification == classification)
    {
      result = vertex;
      return false;
    }
    return true;
  });
  return result;
}

ClipVertex* ClipVertex::FindFirstIntersection(ClipVertex* vertexList)
{
  ClipVertex* result = nullptr;
  ClipVertex::Traverse(vertexList, [&result](ClipVertex* vertex, ClipVertex*& nextVertex)
  {
    if(vertex->mClassification == ClipVertexClassification::InToOut ||
       vertex->mClassification == ClipVertexClassification::OutToIn)
    {
      result = vertex;
      return false;
    }
    return true;
  });
  return result;
}

ClipVertex* ClipVertex::GetNext(ClipVertexSearchDirection direction)
{
  if(direction == ClipVertexSearchDirection::Forwards)
    return mNext;
  return mPrev;
}

//-------------------------------------------------------------------ClipVertexList
ClipVertexList::~ClipVertexList()
{
  ClipVertex* node = mHead;
  do
  {
    ClipVertex* next = node->mNext;
    delete node;
    node = next;
  } while(node != mHead);
  mHead = nullptr;
}

//-------------------------------------------------------------------Clipper
void Clipper::BuildVertexList(const PointContour& points, ClipVertexList& result)
{
  result.mHead = nullptr;
  if(points.empty())
    return;

  size_t count = points.size();
  Array<ClipVertex*> vertices;
  vertices.resize(count);
  for(size_t i = 0; i < count; ++i)
  {
    vertices[i] = new ClipVertex();
    vertices[i]->mPoint = points[i];
  }
  for(size_t i = 0; i < count; ++i)
  {
    vertices[i]->mNext = vertices[(i + 1) % count];
    vertices[i]->mPrev = vertices[(i + count - 1) % count];
  }
  result.mHead = vertices[0];
}

void Clipper::ClassifyVertices(ClipVertexList& vertices)
{
  // Find the first intersection point
  ClipVertex* firstIntersection = ClipVertex::FindFirstOf(vertices.mHead, ClipVertexClassification::None);

  // If there is an intersection point, now tag all vertices as being inside or outside
  if(firstIntersection != nullptr)
  {
    ClipVertexClassification flags = firstIntersection->mClassification == ClipVertexClassification::InToOut ? ClipVertexClassification::Outside : ClipVertexClassification::Inside;
    ClipVertex::Traverse(firstIntersection, [&flags](ClipVertex* v, ClipVertex*& nextVertex)
    {
      if(v->mClassification == ClipVertexClassification::None)
        v->mClassification = flags;
      else if(v->mClassification == ClipVertexClassification::InToOut)
        flags = ClipVertexClassification::Outside;
      else if(v->mClassification == ClipVertexClassification::OutToIn)
        flags = ClipVertexClassification::Inside;
      return true;
    });
  }
  else
  {
    ClipVertex::Traverse(firstIntersection, [](ClipVertex* v, ClipVertex*& nextVertex)
    {
      if(v->mClassification == ClipVertexClassification::None)
        v->mClassification = ClipVertexClassification::Inside;
      return true;
    });
  }
}

void Clipper::ClipEdges(ClipVertex* start, ClipVertex* end, ClipVertexList& clipRegion)
{
  // This algorithm effectively works by checking each edge in the clip region against this edge,
  // inserting new vertices on the edge when there's an intersection point. These edges need to
  // be in the correct order (t-value wise). Each edge on the region is guaranteed to only have
  // one new intersection point, but the given edge could have multiple intersection points and 
  // we'll find them in traversal order (not t-order). To fix this, store them and then sort
  // by t-value so we can be guaranteed to have a valid line.
  struct IntersectionVerts
  {
    ClipVertex* mVertex;
    float mTime;
  };
  Array<IntersectionVerts> newVerts;

  ClipVertex* clipStart = clipRegion.mHead;
  do
  {
    ClipVertex* clipNext = clipStart->mNext;

    ClipVertexClassification line0Flags, line1Flags;
    float time = ComputeIntersectionPoint(start->mPoint, end->mPoint, clipStart->mPoint, clipNext->mPoint, line0Flags, line1Flags);
    if(0 <= time && time <= 1)
    {
      // Create the vertex we're inserting into the clip region list
      ClipVertex* clipVert = new ClipVertex();
      clipVert->mPoint = start->mPoint + (end->mPoint - start->mPoint) * time;
      clipVert->mClassification = line1Flags;
      // Link it into the clip list
      clipStart->mNext = clipVert;
      clipVert->mPrev = clipStart;
      clipVert->mNext = clipNext;
      clipNext->mPrev = clipVert;

      // Also create the vertex for the given edge, but defer adding it until the end.
      ClipVertex* edgeVert = new ClipVertex();
      edgeVert->mPoint = clipVert->mPoint;
      edgeVert->mClassification = line0Flags;
      // Make sure to link the two edges together
      clipVert->mTwin = edgeVert;
      edgeVert->mTwin = clipVert;

      IntersectionVerts iVert;
      iVert.mVertex = edgeVert;
      iVert.mTime = time;
      newVerts.push_back(iVert);
    }

    clipStart = clipNext;
  } while(clipStart != clipRegion.mHead);

  // Sort the vertices in t-first order so we can create a valid line
  std::sort(newVerts.begin(), newVerts.end(), [](const IntersectionVerts& lhs, const IntersectionVerts& rhs)
  {
    return lhs.mTime < rhs.mTime;
  });

  // Insert all of the vertices into the given edge list
  ClipVertex* prevVertex = start;
  size_t newVertCount = newVerts.size();
  for(size_t i = 0; i < newVertCount; ++i)
  {
    ClipVertex* newVertex = newVerts[i].mVertex;
    newVertex->mPrev = prevVertex;
    prevVertex->mNext = newVertex;
    prevVertex = newVertex;
  }
  if(newVertCount != 0)
  {
    ClipVertex* lastNewVertex = newVerts[newVertCount - 1].mVertex;
    lastNewVertex->mNext = end;
    end->mPrev = lastNewVertex;
  }
}

void Clipper::ClipPolygon(ClipVertexList& polygonToClip , ClipVertexList& clipRegion)
{
  // Simply clip each edge in the polygon against the clip region. If new vertices are
  // created in the polygon, we don't need to test sub-edges as we've already processed 
  // the whole edge and new vertices are colinear.
  ClipVertex::Traverse(polygonToClip.mHead, [this, &clipRegion](ClipVertex* vertex, ClipVertex*& nextVertex)
  {
    ClipEdges(vertex, vertex->mNext, clipRegion);
    return true;
  });
}

void Clipper::BuildClipList(const PointContour& polygonPoints, const PointContour& clipRegionPoints, ClipVertexList& polyList, ClipVertexList& clipRegionList)
{
  // Build the individual vertex lists for each polygon
  BuildVertexList(clipRegionPoints, clipRegionList);
  BuildVertexList(polygonPoints, polyList);

  // Clip them against each other and then classify all vertices. Classification is needed to know how to start certain algorithms
  ClipPolygon(polyList, clipRegionList);
  ClassifyVertices(polyList);
  ClassifyVertices(clipRegionList);
}

void Clipper::Union(ClipVertexList& polygon, PointContour& results)
{
  // To start the algorithm, we need a point on the original polygon that will not be clipped away.
  // The only guarantee for this is a intersection point, in particular we need one that is entering the clip region.
  // If there is no intersection point then there's no union to do.
  ClipVertex* firstIntersection = ClipVertex::FindFirstOf(polygon.mHead, ClipVertexClassification::OutToIn);
  if(firstIntersection == nullptr)
    return;

  // Traverse the vertex list, adding each point to the result. If a vertex has a twin, walk that list until they meet back up again.
  ClipVertex::Traverse(firstIntersection, [&results](ClipVertex* vertex, ClipVertex*& nextVertex)
  {
    results.push_back(vertex->mPoint);
    if(vertex->mTwin != nullptr)
    {
      ClipVertex* next = ClipVertex::WalkTwinListForwards(vertex, [&results](ClipVertex* v)
      {
        results.push_back(v->mPoint);
      });
      nextVertex = next->mNext;
    }
    return true;
  });
}

void Clipper::Subtract(ClipVertexList& polygon, PointContourList& contours)
{
  // To do a subtraction, we need to trace all contours on the original polygon
  // that start from any vertex that leaves the clip region.
  ClipVertex* head = ClipVertex::FindFirstOf(polygon.mHead, ClipVertexClassification::InToOut);
  // No intersection points, there's nothing to do
  if(head == nullptr)
    return;

  // Keep track of all potential contour starting points (any point that leaves the clip region).
  // The algorithm will find new points during traversal and will finish once this is empty.
  // Every new point in here that hasn't already been visited is a new contour.
  Array<ClipVertex*> verticesToVisit{head};
  while(!verticesToVisit.empty())
  {
    ClipVertex* contourStart = verticesToVisit.back();
    verticesToVisit.pop_back();

    if(contourStart->mVisited)
      continue;

    ClipVertex* vertex = contourStart;
    ClipVertexSearchDirection direction = ClipVertexSearchDirection::Forwards;
    contours.push_back(PointContour());
    PointContour& currentContour = contours.back();
    
    // Trace this contour by hoping between the polygon and clip region every time we hit an intersection point.
    do
    {
      currentContour.push_back(vertex->mPoint);
      vertex->mVisited = true;
      vertex = vertex->GetNext(direction);

      if(vertex->mTwin != nullptr)
      {
        // The resultant contours could be separated by a clip region. To find possible new contour starts,
        // any time we find a vertex on the original polygon that is entering the clip region, iterate past
        // all vertices that are inside the clip region until we find an intersection point leaving.
        // This exiting point is potentially a new contour for us to start later.
        if(!vertex->mVisited && direction == ClipVertexSearchDirection::Forwards && vertex->mClassification == ClipVertexClassification::OutToIn)
        {
          ClipVertex* nextVertToLeaveClipRegion = ClipVertex::FindFirstOf(vertex, ClipVertexClassification::InToOut);
          verticesToVisit.push_back(nextVertToLeaveClipRegion);
        }
        // Every time we switch between the polygon and clip region we need to change our winding order as we have to traverse the clip region backwards
        direction = FlipSearchDirection(direction);
        vertex = vertex->mTwin;
      }
      // If we reach the starting vertex (or it's twin) then we've finished the loop of this contour.
    } while(vertex != contourStart && vertex->mTwin != contourStart);
  }
}

void Clipper::Intersect(ClipVertexList& polygon, PointContourList& contours)
{
  // To compute an intersection, we need to trace along the intersection of the two regions.
  // To do this, start with a vertex that is going into the clip region.
  ClipVertex* head = ClipVertex::FindFirstOf(polygon.mHead, ClipVertexClassification::OutToIn);
  // No intersection points, there's nothing to do
  if(head == nullptr)
    return;

  Array<ClipVertex*> verticesToVisit{head};
  while(!verticesToVisit.empty())
  {
    ClipVertex* contourStart = verticesToVisit.back();
    verticesToVisit.pop_back();
    
    if(contourStart->mVisited)
      continue;

    ClipVertex* vertex = contourStart;
    contours.push_back(PointContour());
    PointContour& currentContour = contours.back();

    // Trace this contour by hoping between polygons any time we try to leave the interior of one of them.
    do
    {
      currentContour.push_back(vertex->mPoint);
      vertex->mVisited = true;
      vertex = vertex->mNext;

      if(vertex->mClassification == ClipVertexClassification::InToOut)
      {
        // If we were leaving this clip region, there could be another intersection further away.
        // Iterate from this exit point until we next enter an intersection. This point is a new possible contour candidate.
        if(!vertex->mVisited)
        {
          ClipVertex* nextVertToLeaveClipRegion = ClipVertex::FindFirstOf(vertex, ClipVertexClassification::OutToIn);
          verticesToVisit.push_back(nextVertToLeaveClipRegion);
        }
        
        vertex = vertex->mTwin;
      }
      // If we reach the starting vertex (or it's twin) then we've finished the loop of this contour.
    } while(vertex != contourStart && vertex->mTwin != contourStart);
  }
}

void Clipper::Union(const PointContour& polygonPoints, const PointContour& clipRegion, PointContour& results)
{
  results.clear();

  ClipVertexList clipList;
  ClipVertexList polyList;
  BuildClipList(polygonPoints, clipRegion, polyList, clipList);

  Union(polyList, results);
}

void Clipper::Subtract(const PointContour& polygonPoints, const PointContour& clipRegion, PointContourList& contours)
{
  contours.clear();

  ClipVertexList clipList;
  ClipVertexList polyList;
  BuildClipList(polygonPoints, clipRegion, polyList, clipList);

  Subtract(polyList, contours);
}

void Clipper::Intersect(const PointContour& polygonPoints, const PointContour& clipRegion, PointContourList& contours)
{
  contours.clear();

  ClipVertexList clipList;
  ClipVertexList polyList;
  BuildClipList(polygonPoints, clipRegion, polyList, clipList);

  Intersect(polyList, contours);
}
