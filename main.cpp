#include <iostream>
#include <assert.h>
#include "clipper.hpp"

using namespace std;
using namespace ClipperLib;

int main() {
    cout << "Hello, World!" << endl;
    Clipper clpr;
    Path figure8;
    figure8.push_back(IntPoint(2010,1010,1));
    figure8.push_back(IntPoint(4020,3020,2));
    figure8.push_back(IntPoint(5030,2030,3));
    figure8.push_back(IntPoint(4040,1040,4));
    figure8.push_back(IntPoint(2050,3050,5));
    figure8.push_back(IntPoint(1060,2060,6));
    Path boundingBox;
    boundingBox.push_back(IntPoint( 500,  500, -1));
    boundingBox.push_back(IntPoint( 500, 5500, -2));
    boundingBox.push_back(IntPoint(4500, 5500, -3));
    boundingBox.push_back(IntPoint(4500,  500, -4));

    clpr.AddPath(figure8, ptSubject, true);
    Paths solution;
    clpr.Execute(ctUnion, solution);
    for (Path p : solution) {
        cout << "Path (" << p.size() << ")" << endl;
        for (IntPoint pt : p) {
            cout << "    (" << pt.X << ", " << pt.Y << ", " << pt.Z << ")" << endl;
        }
    }
    return 0;
}

class FollowingZFill : public ZFill {
public:
    virtual void OnIntersection(IntPoint& e1bot, IntPoint& e1top, bool e1Forward,
                                IntPoint& e2bot, IntPoint& e2top, bool e2Forward,
                                const IntPoint &pt, cInt& z1f, cInt& z1r, cInt& z2f, cInt& z2r) {
        // I was really hoping that there was some great gestalt of understanding for this.
        // But I can't see it. This routine is the result of enumerating all 4 cases.

        // The problem decomposes nicely into independent problems on each edge.
        // a     d
        //  \   /
        // z1f z2f
        // z1r z2r
        //  /   \
        // c     f

        if (e1Forward) {
            // set z1f=b(f), z2r=r(e(f)), e1top.Z=e(f)
            z1f = StripBegin(e1top.Z, e1bot, e1top, pt);
            z2r = ReverseZ(StripEnd(e1top.Z, e1bot, e1top, pt));
            e1top.Z = StripEnd(e1top.Z, e1bot, e1top, pt);
        } else {
            // set z1f=r(e(a)), z2r=b(a), e1bot.Z=b(a)
            z1f = ReverseZ(StripEnd(e1bot.Z, e1top, e1bot, pt));
            z2r = StripBegin(e1bot.Z, e1top, e1bot, pt);
            e1bot.Z = StripBegin(e1bot.Z, e1top, e1bot, pt);
        }

        if (e2Forward) {
            // set z2f=b(c), z1r=r(e(c)), e2top.Z=e(c)
            z2f = StripBegin(e2top.Z, e2bot, e2top, pt);
            z1r = ReverseZ(StripEnd(e2top.Z, e2bot, e2top, pt));
            e2top.Z = StripEnd(e2top.Z, e2bot, e2top, pt);
        } else {
            // set z2f=r(e(d)), z1r=b(d), e2bot.Z=e(d)
            z2f = ReverseZ(StripEnd(e2bot.Z, e2top, e2bot, pt));
            z1r = StripBegin(e2bot.Z, e2top, e2bot, pt);
            e2bot.Z = StripBegin(e2bot.Z, e2top, e2bot, pt);
        }
    }
    virtual void OnPoint(IntPoint& prev, IntPoint& curr, IntPoint& next, bool forward, IntPoint2Z& pt) {
        if (forward) {
            pt.forwardZ = curr.Z;
            pt.reverseZ = ReverseZ(next.Z);
        } else {
            pt.forwardZ = ReverseZ(prev.Z);
            pt.reverseZ = curr.Z;
        }
    }

    virtual void BeginLoopReversal(IntPoint& last, IntPoint& first, cInt filler) {
        assert(firstWas == 0);
        firstWas = first.Z;
        if (filler) {
            first.Z = ReverseZ(filler);
        } else {
            first.Z = ReverseZ(last.Z);
        }
    }
    virtual void ReverseEdge(IntPoint& first, IntPoint& second) {
        cInt firstWill = second.Z;
        second.Z = ReverseZ(firstWas);
        firstWas = firstWill;
    }
    virtual void FinishLoopReversal(IntPoint& last, IntPoint& first) {
        firstWas = 0;
    }

    virtual void OnOffset(int step, int steps, IntPoint& source, IntPoint& dest) {
        dest.Z = Clone(source.Z);
    }

protected:
    virtual cInt ReverseZ(cInt z) {return z;}
    virtual cInt Clone(cInt z) {return z;}
    virtual cInt StripBegin(cInt& z, IntPoint& from, IntPoint& to, const IntPoint& pt) {return z;}
    virtual cInt StripEnd(cInt& z, IntPoint& from, IntPoint& to, const IntPoint& pt) {return z;}
    cInt firstWas = 0;
};
