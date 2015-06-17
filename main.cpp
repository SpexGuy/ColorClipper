#include <iostream>
#include <assert.h>
#include "clipper.hpp"

using namespace std;
using namespace ClipperLib;

#ifdef use_xyz
class FollowingZFill : public ZFill {
public:
    virtual void OnIntersection(IntPoint& e1bot, IntPoint& e1top, bool e1Forward,
                                IntPoint& e2bot, IntPoint& e2top, bool e2Forward,
                                const IntPoint &pt, cInt& z1f, cInt& z1r, cInt& z2f, cInt& z2r) override {
        // I was really hoping that there was some great gestalt of understanding for this.
        // But I can't see it. This routine is the result of enumerating all 4 cases.

        // The problem decomposes nicely into one problem repeated independently on each edge.
        // a     d
        //  \   /
        // z1f z2f
        // z1r z2r
        //  /   \
        // c     f
        SplitEdge(e1bot, e1top, e1Forward, pt, z1f, z2r);
        SplitEdge(e2bot, e2top, e2Forward, pt, z2f, z1r);
    }
    virtual void OnPoint(IntPoint& prev, IntPoint& curr, IntPoint& next, bool forward, IntPoint2Z& pt) override {
        if (forward) {
            pt.forwardZ = curr.Z;
            pt.reverseZ = ReverseZ(prev.Z);
        } else {
            pt.forwardZ = ReverseZ(next.Z);
            pt.reverseZ = curr.Z;
        }
    }

    virtual void OnOffset(int step, int steps, IntPoint& source, IntPoint& dest) override {
        dest.Z = Clone(source.Z);
    }

protected:
    virtual cInt ReverseZ(cInt z) {return z;}
    virtual cInt Clone(cInt z) {return z;}
    virtual cInt StripBegin(cInt z, IntPoint& from, IntPoint& to, const IntPoint& pt) {return z;}
    virtual cInt StripEnd(cInt z, IntPoint& from, IntPoint& to, const IntPoint& pt) {return z;}

private:
    void SplitEdge(IntPoint& bot, IntPoint& top, bool forward, const IntPoint& pt, cInt& zf, cInt& zr) {
        if (forward) {
            // set z1f=b(f), z2r=r(e(f)), e1top.Z=e(f)
            zf = StripBegin(top.Z, bot, top, pt);
            zr = ReverseZ(StripEnd(top.Z, bot, top, pt));
            top.Z = StripEnd(top.Z, bot, top, pt);
        } else {
            // set z1f=r(e(a)), z2r=b(a), e1bot.Z=b(a)
            zf = ReverseZ(StripEnd(bot.Z, top, bot, pt));
            zr = StripBegin(bot.Z, top, bot, pt);
            bot.Z = StripBegin(bot.Z, top, bot, pt);
        }
    }
};

class TestFollower : public FollowingZFill {
protected:
    virtual cInt ReverseZ(cInt z) override {return -z;}
    virtual cInt StripBegin(cInt z, IntPoint& from, IntPoint& to, const IntPoint& pt) {return z-10;}
    virtual cInt StripEnd(cInt z, IntPoint& from, IntPoint& to, const IntPoint& pt) {return z+10;}
};

#endif

int main() {
    #ifdef use_xyz
    cout << "Hello, World!" << endl;
    TestFollower zFill;
    Clipper clpr;
    clpr.Callback(&zFill);
    Path figure8;
    figure8.push_back(IntPoint(2010,1010,150));
    figure8.push_back(IntPoint(4020,3020,250));
    figure8.push_back(IntPoint(5030,2030,350));
    figure8.push_back(IntPoint(4040,1040,450));
    figure8.push_back(IntPoint(2050,3050,550));
    figure8.push_back(IntPoint(1060,2060,650));
    Paths intersectionTest;
    Path intersectionTestLargePart;
    intersectionTestLargePart << IntPoint( 300,  300,  150)
                              << IntPoint( 600,  600,  250)
                              << IntPoint( 700,  500,  350)
                              << IntPoint( 500,  300,  450)
                              << IntPoint( 600,  200,  550)
                              << IntPoint( 700,  300,  650)
                              << IntPoint( 300,  700,  750)
                              << IntPoint( 100,  500,  850);
    Path intersectionTestSmallPart;
    intersectionTestSmallPart << IntPoint( 300,  100, 1050)
                              << IntPoint( 500,  300, 1150)
                              << IntPoint( 300,  500, 1250)
                              << IntPoint( 100,  300, 1350);
    intersectionTest << intersectionTestLargePart << intersectionTestSmallPart;

    clpr.AddPaths(intersectionTest, ptSubject, true);
    Paths solution;
    clpr.Execute(ctUnion, solution);
    for (Path p : solution) {
        cout << "Path (" << p.size() << ")" << endl;
        for (IntPoint pt : p) {
            cout << "    (" << pt.X << ", " << pt.Y << ", " << pt.Z << ")" << endl;
        }
    }
    #endif
    return 0;
}
