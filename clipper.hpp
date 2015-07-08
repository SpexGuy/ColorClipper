/*******************************************************************************
*                                                                              *
* Author    :  Angus Johnson, Martin Wickham                                   *
* Version   :  7.1.0                                                           *
* Date      :  7 July 2015                                                     *
* Website   :  http://www.angusj.com                                           *
* Copyright :  Angus Johnson 2010-2014, Martin Wickham 2015                    *
*                                                                              *
* License:                                                                     *
* Use, modification & distribution is subject to Boost Software License Ver 1. *
* http://www.boost.org/LICENSE_1_0.txt                                         *
*                                                                              *
* Attributions:                                                                *
* The code in this library is an extension of Bala Vatti's clipping algorithm: *
* "A generic solution to polygon clipping"                                     *
* Communications of the ACM, Vol 35, Issue 7 (July 1992) pp 56-63.             *
* http://portal.acm.org/citation.cfm?id=129906                                 *
*                                                                              *
* Computer graphics and geometric modeling: implementation and algorithms      *
* By Max K. Agoston                                                            *
* Springer; 1 edition (January 4, 2005)                                        *
* http://books.google.com/books?q=vatti+clipping+agoston                       *
*                                                                              *
* See also:                                                                    *
* "Polygon Offsetting by Computing Winding Numbers"                            *
* Paper no. DETC2005-85513 pp. 565-575                                         *
* ASME 2005 International Design Engineering Technical Conferences             *
* and Computers and Information in Engineering Conference (IDETC/CIE2005)      *
* September 24-28, 2005 , Long Beach, California, USA                          *
* http://www.me.berkeley.edu/~mcmains/pubs/DAC05OffsetPolygon.pdf              *
*                                                                              *
*******************************************************************************/

#ifndef clipper_hpp
#define clipper_hpp

#define CLIPPER_VERSION "7.1.0"

//use_int32: When enabled 32bit ints are used instead of 64bit ints. This
//improve performance but coordinate values are limited to the range +/- 46340
//#define use_int32

//use_xyz: adds a Z member to IntPoint. Adds a cost to performance.
#define use_xyz

//use_lines: Enables line clipping. Adds a very minor cost to performance.
//#define use_lines
  
//use_deprecated: Enables temporary support for the obsolete functions
//#define use_deprecated  

#include <vector>
#include <set>
#include <stdexcept>
#include <cstring>
#include <cstdlib>
#include <ostream>
#include <functional>
#include <queue>

namespace ClipperLib {

enum ClipType { ctIntersection, ctUnion, ctDifference, ctXor };
enum PolyType { ptSubject, ptClip };
//By far the most widely used winding rules for polygon filling are
//EvenOdd & NonZero (GDI, GDI+, XLib, OpenGL, Cairo, AGG, Quartz, SVG, Gr32)
//Others rules include Positive, Negative and ABS_GTR_EQ_TWO (only in OpenGL)
//see http://glprogramming.com/red/chapter11.html
enum PolyFillType { pftEvenOdd, pftNonZero, pftPositive, pftNegative };

#ifdef use_int32
  typedef int cInt;
  static cInt const loRange = 0x7FFF;
  static cInt const hiRange = 0x7FFF;
#else
  typedef signed long long cInt;
  static cInt const loRange = 0x3FFFFFFF;
  static cInt const hiRange = 0x3FFFFFFFFFFFFFFFLL;
  typedef signed long long long64;     //used by Int128 class
  typedef unsigned long long ulong64;

#endif

#ifdef use_xyz
struct IntPoint2Z;
#endif

struct IntPoint {
  cInt X;
  cInt Y;
#ifdef use_xyz
  cInt Z;
  IntPoint(cInt x = 0, cInt y = 0, cInt z = 0): X(x), Y(y), Z(z) {};
  IntPoint(const IntPoint2Z& pt);
#else
  IntPoint(cInt x = 0, cInt y = 0): X(x), Y(y) {};
#endif
};

#ifdef use_xyz
struct IntPoint2Z {
  cInt X;
  cInt Y;
  cInt correctZ;
  cInt reverseZ;
  IntPoint2Z(cInt x = 0, cInt y = 0, cInt cz = 0, cInt rz = 0) : X(x), Y(y), correctZ(cz), reverseZ(rz) {};
  IntPoint2Z(const IntPoint& pt) : X(pt.X), Y(pt.Y), correctZ(pt.Z), reverseZ(0) {};
  inline void reverse() { std::swap(correctZ, reverseZ); }
  inline cInt getZ() const { return correctZ; }
};
typedef IntPoint2Z OutCoord;
#else
typedef IntPoint OutCoord;
#endif

inline bool operator== (const IntPoint& a, const IntPoint& b)
{
  return a.X == b.X && a.Y == b.Y;
}
inline bool operator!= (const IntPoint& a, const IntPoint& b)
{
  return a.X != b.X  || a.Y != b.Y;
}
#ifdef use_xyz
inline bool operator== (const IntPoint& a, const IntPoint2Z& b)
{
  return a.X == b.X && a.Y == b.Y;
}
inline bool operator!= (const IntPoint& a, const IntPoint2Z& b)
{
  return a.X != b.X  || a.Y != b.Y;
}
inline bool operator== (const IntPoint2Z& a, const IntPoint& b)
{
  return a.X == b.X && a.Y == b.Y;
}
inline bool operator!= (const IntPoint2Z& a, const IntPoint& b)
{
  return a.X != b.X  || a.Y != b.Y;
}
inline bool operator== (const IntPoint2Z& a, const IntPoint2Z& b)
{
  return a.X == b.X && a.Y == b.Y;
}
inline bool operator!= (const IntPoint2Z& a, const IntPoint2Z& b)
{
  return a.X != b.X  || a.Y != b.Y;
}
#endif

//------------------------------------------------------------------------------

typedef std::vector< IntPoint > Path;
typedef std::vector< Path > Paths;

inline Path& operator <<(Path& poly, const IntPoint& p) {poly.push_back(p); return poly;}
inline Paths& operator <<(Paths& polys, const Path& p) {polys.push_back(p); return polys;}

std::ostream& operator <<(std::ostream &s, const IntPoint &p);
std::ostream& operator <<(std::ostream &s, const Path &p);
std::ostream& operator <<(std::ostream &s, const Paths &p);

struct DoublePoint
{
  double X;
  double Y;
  DoublePoint(double x = 0, double y = 0) : X(x), Y(y) {}
  DoublePoint(IntPoint ip) : X((double)ip.X), Y((double)ip.Y) {}
};
//------------------------------------------------------------------------------

#ifdef use_xyz
class ZFill {
public:
  virtual void InitializeReverse(IntPoint2Z &curr, IntPoint2Z &next);
  virtual void OnIntersection(const IntPoint2Z& e1bot, IntPoint2Z &e1pt, const IntPoint2Z& e1top,
                              const IntPoint2Z& e2bot, IntPoint2Z &e2pt, const IntPoint2Z& e2top);
  // Points here are passed in the same order they were in the original polygon
  virtual void OnSplitEdge(const IntPoint2Z &prev, IntPoint2Z &pt, const IntPoint2Z &next);
  virtual void OnAppendOverlapping(IntPoint2Z &prev, IntPoint2Z &to);
  virtual void OnJoin(IntPoint2Z &e1from, IntPoint2Z &e1to, IntPoint2Z &e2from, IntPoint2Z &e2to);
  virtual void OnRemoveSpike(IntPoint2Z &prev, IntPoint2Z &curr, IntPoint2Z &next);
  virtual void OnOffset(int step, int steps, const IntPoint &prev, const IntPoint &curr, const IntPoint &next, IntPoint& pt);
  virtual void OnReversePath(Path &poly); // passed before the reversal occurs
  virtual void OnFinishOffset(Path &poly);
  virtual ~ZFill() {}
};

class FollowingZFill : public ZFill {
public:
  virtual void InitializeReverse(IntPoint2Z &curr, IntPoint2Z &next) override;
  virtual void OnIntersection(const IntPoint2Z& e1bot, IntPoint2Z &e1pt, const IntPoint2Z& e1top,
                              const IntPoint2Z& e2bot, IntPoint2Z &e2pt, const IntPoint2Z& e2top) override;
  virtual void OnSplitEdge(const IntPoint2Z &prev, IntPoint2Z &pt, const IntPoint2Z &next) override;
  virtual void OnAppendOverlapping(IntPoint2Z &prev, IntPoint2Z &to) override;
  virtual void OnJoin(IntPoint2Z &e1from, IntPoint2Z &e1to, IntPoint2Z &e2from, IntPoint2Z &e2to) override;
  virtual void OnRemoveSpike(IntPoint2Z &prev, IntPoint2Z &curr, IntPoint2Z &next) override;
  virtual void OnOffset(int step, int steps, const IntPoint &prev, const IntPoint &curr, const IntPoint &next, IntPoint& pt) override;
  virtual void OnReversePath(Path &poly) override;
  virtual void OnFinishOffset(Path &poly) override;
  virtual ~FollowingZFill() {}
protected:
  // Override these functions for more complex edge attributes (like sub-extents)

  // Reverse the edge attribute pointed to by z
  virtual void ReverseZ(cInt z);
  // Clone the edge attribute pointed to by z and return the pointer to this clone.
  virtual cInt Clone(cInt z);
  // Strip the range [from, pt) out of the edge attribute pointed to by z and return
  // a new attribute containing only this range.
  // pt is guaranteed to be on the line between from and to (within integer truncation)
  // and not coincident with either.
  virtual cInt StripBegin(cInt z, const IntPoint& from, const IntPoint& to, const IntPoint& pt);
};
#endif

enum InitOptions {ioReverseSolution = 1, ioStrictlySimple = 2, ioPreserveCollinear = 4};
enum JoinType {jtSquare, jtRound, jtMiter};
enum EndType {etClosedPolygon, etClosedLine, etOpenButt, etOpenSquare, etOpenRound};

class PolyNode;
typedef std::vector< PolyNode* > PolyNodes;

class PolyNode 
{ 
public:
    PolyNode();
    virtual ~PolyNode(){};
    Path Contour;
    PolyNodes Childs;
    PolyNode* Parent;
    PolyNode* GetNext() const;
    bool IsHole() const;
    bool IsOpen() const;
    int ChildCount() const;
private:
    unsigned Index; //node index in Parent.Childs
    bool m_IsOpen;
    JoinType m_jointype;
    EndType m_endtype;
    PolyNode* GetNextSiblingUp() const;
    void AddChild(PolyNode& child);
    friend class Clipper; //to access Index
    friend class ClipperOffset; 
};

class PolyTree: public PolyNode
{ 
public:
    ~PolyTree(){Clear();};
    PolyNode* GetFirst() const;
    void Clear();
    int Total() const;
private:
    PolyNodes AllNodes;
    friend class Clipper; //to access AllNodes
};

bool Orientation(const Path &poly);
double Area(const Path &poly);
int PointInPolygon(const IntPoint &pt, const Path &path);

void SimplifyPolygon(const Path &in_poly, Paths &out_polys, PolyFillType fillType = pftEvenOdd);
void SimplifyPolygons(const Paths &in_polys, Paths &out_polys, PolyFillType fillType = pftEvenOdd);
void SimplifyPolygons(Paths &polys, PolyFillType fillType = pftEvenOdd);

void CleanPolygon(const Path& in_poly, Path& out_poly, double distance = 1.415);
void CleanPolygon(Path& poly, double distance = 1.415);
void CleanPolygons(const Paths& in_polys, Paths& out_polys, double distance = 1.415);
void CleanPolygons(Paths& polys, double distance = 1.415);

void MinkowskiSum(const Path& pattern, const Path& path, Paths& solution, bool pathIsClosed);
void MinkowskiSum(const Path& pattern, const Paths& paths, Paths& solution, bool pathIsClosed);
void MinkowskiDiff(const Path& poly1, const Path& poly2, Paths& solution);

void PolyTreeToPaths(const PolyTree& polytree, Paths& paths);
void ClosedPathsFromPolyTree(const PolyTree& polytree, Paths& paths);
void OpenPathsFromPolyTree(PolyTree& polytree, Paths& paths);

void ReversePath(Path& p);
void ReversePaths(Paths& p);

struct IntRect { cInt left; cInt top; cInt right; cInt bottom; };

//enums that are used internally ...
enum EdgeSide { esLeft = 1, esRight = 2};

//forward declarations (for stuff used internally) ...
struct TEdge;
struct IntersectNode;
struct LocalMinimum;
struct Scanbeam;
struct OutPt;
struct OutRec;
struct Join;

typedef std::vector < OutRec* > PolyOutList;
typedef std::vector < TEdge* > EdgeList;
typedef std::vector < Join* > JoinList;
typedef std::vector < IntersectNode* > IntersectList;

//------------------------------------------------------------------------------

//ClipperBase is the ancestor to the Clipper class. It should not be
//instantiated directly. This class simply abstracts the conversion of sets of
//polygon coordinates into edge objects that are stored in a LocalMinima list.
class ClipperBase
{
public:
  ClipperBase();
  virtual ~ClipperBase();
  bool AddPath(const Path &pg, PolyType PolyTyp, bool Closed);
  bool AddPaths(const Paths &ppg, PolyType PolyTyp, bool Closed);
  virtual void Clear();
  IntRect GetBounds();
  bool PreserveCollinear() {return m_PreserveCollinear;};
  void PreserveCollinear(bool value) {m_PreserveCollinear = value;};
#ifdef use_xyz
  void Callback(ZFill *zFill);
#endif
protected:
  void DisposeLocalMinimaList();
  TEdge* AddBoundsToLML(TEdge *e, bool IsClosed);
  void PopLocalMinima();
  virtual void Reset();
  void InitEdge2(TEdge& e, PolyType Pt);
  TEdge* ProcessBound(TEdge* E, bool IsClockwise);
  void DoMinimaLML(TEdge* E1, TEdge* E2, bool IsClosed);
  TEdge* DescendToMin(TEdge *&E);
  void AscendToMax(TEdge *&E, bool Appending, bool IsClosed);

  typedef std::vector<LocalMinimum> MinimaList;
  MinimaList::iterator m_CurrentLM;
  MinimaList           m_MinimaList;

  bool              m_UseFullRange;
  EdgeList          m_edges;
  bool             m_PreserveCollinear;
  bool             m_HasOpenPaths;
#ifdef use_xyz
  ZFill           *m_ZFill; //custom callback
#endif
};
//------------------------------------------------------------------------------

class Clipper : public virtual ClipperBase
{
public:
  Clipper(int initOptions = 0);
  ~Clipper();
  bool Execute(ClipType clipType,
    Paths &solution,
    PolyFillType subjFillType = pftEvenOdd,
    PolyFillType clipFillType = pftEvenOdd);
  bool Execute(ClipType clipType,
    PolyTree &polytree,
    PolyFillType subjFillType = pftEvenOdd,
    PolyFillType clipFillType = pftEvenOdd);
  bool ReverseSolution() {return m_ReverseOutput;};
  void ReverseSolution(bool value) {m_ReverseOutput = value;};
  bool StrictlySimple() {return m_StrictSimple;};
  void StrictlySimple(bool value) {m_StrictSimple = value;};
  //set the callback function for z value filling on intersections (otherwise Z is 0)
protected:
  void Reset();
  virtual bool ExecuteInternal();
private:
  PolyOutList       m_PolyOuts;
  JoinList          m_Joins;
  JoinList          m_GhostJoins;
  IntersectList     m_IntersectList;
  ClipType          m_ClipType;
  typedef std::priority_queue<cInt> ScanbeamList;
  ScanbeamList      m_Scanbeam;
  TEdge           *m_ActiveEdges;
  TEdge           *m_SortedEdges;
  bool             m_ExecuteLocked;
  PolyFillType     m_ClipFillType;
  PolyFillType     m_SubjFillType;
  bool             m_ReverseOutput;
  bool             m_UsingPolyTree; 
  bool             m_StrictSimple;
  void SetWindingCount(TEdge& edge);
  bool IsEvenOddFillType(const TEdge& edge) const;
  bool IsEvenOddAltFillType(const TEdge& edge) const;
  void InsertScanbeam(const cInt Y);
  cInt PopScanbeam();
  void InsertLocalMinimaIntoAEL(const cInt botY);
  void InsertEdgeIntoAEL(TEdge *edge, TEdge* startEdge);
  void AddEdgeToSEL(TEdge *edge);
  void CopyAELToSEL();
  void DeleteFromSEL(TEdge *e);
  void DeleteFromAEL(TEdge *e);
  void UpdateEdgeIntoAEL(TEdge *&e);
  void SwapPositionsInSEL(TEdge *edge1, TEdge *edge2);
  bool IsContributing(const TEdge& edge) const;
  bool IsTopHorz(const cInt XPos);
  void SwapPositionsInAEL(TEdge *edge1, TEdge *edge2);
  void DoMaxima(TEdge *e);
  void ProcessHorizontals(bool IsTopOfScanbeam);
  void ProcessHorizontal(TEdge *horzEdge, bool isTopOfScanbeam);
  void AddLocalMaxPoly(TEdge *e1, TEdge *e2, const OutCoord &pt);
  OutPt* AddLocalMinPoly(TEdge *e1, TEdge *e2, const OutCoord &pt);
  OutPt* AddIntersectionMinPoly(TEdge *e1, TEdge *e2, const IntPoint &pt);
  OutRec* GetOutRec(int idx);
  void LinkPolygon(OutPt *tail1, OutPt *head1, OutPt *tail2, OutPt *head2);
  void AppendPolygon(TEdge *e1, TEdge *e2);
  void IntersectEdges(TEdge *e1, TEdge *e2, const IntPoint &pt);
  OutRec* CreateOutRec();
  OutPt* AddOutPt(TEdge *e, const OutCoord &pt);
  void DisposeAllOutRecs();
  void DisposeOutRec(PolyOutList::size_type index);
  bool ProcessIntersections(const cInt topY);
  void BuildIntersectList(const cInt topY);
  void ProcessIntersectList();
  void ProcessEdgesAtTopOfScanbeam(const cInt topY);
  void PromoteIntermediate(TEdge *&e);
  void BuildResult(Paths& polys);
  void BuildResult2(PolyTree& polytree);
  void SetHoleState(TEdge *e, OutRec *outrec);
  void DisposeIntersectNodes();
  bool FixupIntersectionOrder();
  void FixupOutPolygon(OutRec &outrec);
  bool IsHole(TEdge *e);
  bool FindOwnerFromSplitRecs(OutRec &outRec, OutRec *&currOrfl);
  void FixHoleLinkage(OutRec &outrec);
  void AddJoin(OutPt *op1, OutPt *op2, const IntPoint offPt);
  void ClearJoins();
  void ClearGhostJoins();
  void AddGhostJoin(OutPt *op, const IntPoint offPt);
  bool JoinHorz(OutPt* op1, OutPt* op1b, OutPt* op2, OutPt* op2b, OutCoord Pt, bool DiscardLeft);
  bool JoinPoints(Join *j, OutRec* outRec1, OutRec* outRec2);
  void JoinCommonEdges();
  void DoSimplePolygons();
  void FixupFirstLefts1(OutRec* OldOutRec, OutRec* NewOutRec);
  void FixupFirstLefts2(OutRec* OldOutRec, OutRec* NewOutRec);
#ifdef use_xyz
  void SetIntermediateZ(TEdge *e, IntPoint2Z& pt);
  void SetLocalMaxZ(TEdge* e1, TEdge* e2, IntPoint2Z& pt);
  void SetLocalMinZ(TEdge* e1, TEdge* e2, IntPoint2Z& pt);
  void SetIntersectionZ(TEdge *e1, TEdge *e2, IntPoint2Z &p1, IntPoint2Z &p2);
  void SetIntersectionIntermediateZ(TEdge *e1, TEdge *e2, IntPoint2Z &left, IntPoint2Z &right);
  void SetIntersectionMinMaxZ(TEdge *e1, TEdge *e2, IntPoint2Z &min, IntPoint2Z &max);
  void SetEdgeSplitZ(OutPt *splitPt);
  void SetEdgeSplitZ(TEdge *edge, IntPoint2Z &pt);
#endif
};
//------------------------------------------------------------------------------

class ClipperOffset 
{
public:
  ClipperOffset(double miterLimit = 2.0, double roundPrecision = 0.25);
  ~ClipperOffset();
  void AddPath(const Path& path, JoinType joinType, EndType endType);
  void AddPaths(const Paths& paths, JoinType joinType, EndType endType);
  void Execute(Paths& solution, double delta);
  void Execute(PolyTree& solution, double delta);
  void Clear();
#ifdef use_xyz
  void Callback(ZFill *zFill);
#endif
  double MiterLimit;
  double ArcTolerance;
private:
  Paths m_destPolys;
  EndType m_endType;
  Path m_srcPoly;
  Path m_destPoly;
  std::vector<DoublePoint> m_normals;
  double m_delta, m_sinA, m_sin, m_cos;
  double m_miterLim, m_StepsPerRad;
  IntPoint m_lowest;
  PolyNode m_polyNodes;
#ifdef use_xyz
  ZFill *m_ZFill;
#endif

  void FixOrientations();
  void DoOffset(double delta);
  void OffsetPoint(int j, int& k, JoinType jointype);
  void DoSquare(int j, int k);
  void DoMiter(int j, int k, double r);
  void DoRound(int j, int k);
#ifdef use_xyz
  void SetOffsetZ(int step, int steps, int index, IntPoint& dest);
  void SetPointOffsetZ(int step, int steps, IntPoint& dest);
  void SetFirstOffsetZ(int step, int steps, IntPoint& dest);
  void SetLastOffsetZ(int step, int steps, IntPoint& dest);
#endif
};
//------------------------------------------------------------------------------

class clipperException : public std::exception
{
  public:
    clipperException(const char* description): m_descr(description) {}
    virtual ~clipperException() throw() {}
    virtual const char* what() const throw() {return m_descr.c_str();}
  private:
    std::string m_descr;
};
//------------------------------------------------------------------------------

} //ClipperLib namespace

#endif //clipper_hpp


