#include <iostream>
#include <algorithm>
#include <assert.h>
#include <sstream>
#include "clipper.hpp"

using namespace std;
using namespace ClipperLib;

#ifdef use_xyz
cInt CreateZ(const string &name) {
    // leaky, but program doesn't run long so it's fine
    string *value = new string(name);
    return reinterpret_cast<cInt>(value);
}
string &toString(cInt value) {
    if (!value) {
        value = CreateZ("NULL"); // This is a failure.
        cout << "Null!!!!" << endl;
    }
    assert(value >= 10000); // this probably means we forgot to update the test
    return *reinterpret_cast<string *>(value);
}
float dist(const IntPoint &p1, const IntPoint &p2) {
    cInt dx = p1.X - p2.X;
    cInt dy = p1.Y - p2.Y;
    return sqrt(float(dx*dx + dy*dy));
}

class FollowingZFill : public ZFill {
public:
    virtual void InitializeReverse(IntPoint2Z &curr, IntPoint2Z &next) override {
        curr.reverseZ = Clone(next.correctZ);
        ReverseZ(curr.reverseZ);
    }
    // Pre-join tasks
    virtual void OnIntersection(IntPoint2Z& e1bot, IntPoint2Z& e1top, bool e1Forward,
                                IntPoint2Z& e2bot, IntPoint2Z& e2top, bool e2Forward,
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
        SplitIntersection(e1bot, e1top, e1Forward, pt, z1f, z2r);
        SplitIntersection(e2bot, e2top, e2Forward, pt, z2f, z1r);
    }
    virtual void OnAppendOverlapping(IntPoint2Z &from, IntPoint2Z &to) override {
        to.correctZ = from.correctZ;
        from.reverseZ = to.reverseZ;
    }
    // Post-join tasks
    virtual void OnJoin(IntPoint2Z &e1from, IntPoint2Z &e1to, IntPoint2Z &e2from, IntPoint2Z &e2to) override {
        e1to.correctZ = e2from.correctZ;
        e1to.reverseZ = e2from.reverseZ;
        e2to.correctZ = e1from.correctZ;
        e2to.reverseZ = e1from.reverseZ;
    }
    virtual void OnSplitEdge(IntPoint2Z &prev, IntPoint2Z &pt, IntPoint2Z &next) override {
        pt.correctZ = StripBegin(next.correctZ, prev, next, pt);
        pt.reverseZ = StripBegin(prev.reverseZ, next, prev, pt);
    }
    virtual void OnRemoveSpike(IntPoint2Z &prev, IntPoint2Z &curr, IntPoint2Z &next) override {
        RemoveSpike(prev, curr, next, prev.correctZ, curr.correctZ, next.correctZ);
        RemoveSpike(next, curr, prev, next.reverseZ, curr.reverseZ, prev.reverseZ);
    }

    virtual void OnOffset(int step, int steps, IntPoint& source, IntPoint& dest) override {
        dest.Z = Clone(source.Z);
    }

protected:
    virtual void ReverseZ(cInt z) { }
    virtual cInt Clone(cInt z) {return z;}
    virtual cInt StripBegin(cInt z, const IntPoint& from, const IntPoint& to, const IntPoint& pt) {return z;}

private:
    void SplitIntersection(IntPoint2Z &bot, IntPoint2Z &top, bool forward, const IntPoint &pt, cInt &zf, cInt &zr) {
        if (forward) {
            zf = StripBegin(top.correctZ, bot, top, pt);
            zr = StripBegin(bot.reverseZ, top, bot, pt);
        } else {
            zf = StripBegin(top.reverseZ, bot, top, pt);
            zr = StripBegin(bot.correctZ, top, bot, pt);
        }
    }
    void RemoveSpike(const IntPoint &from, const IntPoint &spike, const IntPoint &to, cInt &fromZ, const cInt spikeZ, cInt &toZ) {
        if (from == to) {
            toZ = fromZ;
        } else if (abs(from.X - spike.X) > abs(from.Y - spike.Y)) { // is x more precise than y?
            if (abs(from.X - spike.X) > abs(to.X - spike.X)) { // is from further than to?
                toZ = StripBegin(spikeZ, from, spike, to);
            } else {
                StripBegin(toZ, spike, to, from);
            }
        } else {
            if (abs(from.Y - spike.Y) > abs(to.Y - spike.Y)) { // is from further than to?
                toZ = StripBegin(spikeZ, from, spike, to);
            } else {
                StripBegin(toZ, spike, to, from);
            }
        }
    }
};

class TestFollower : public FollowingZFill {
protected:
    virtual void ReverseZ(cInt z) override {toString(z) = "r("+toString(z)+")";}
    virtual cInt Clone(cInt z) override {return CreateZ(toString(z));}
    virtual cInt StripBegin(cInt z, const IntPoint& from, const IntPoint& to, const IntPoint& pt) override {
        stringstream begin, end;
        begin << "b(" << toString(z) << ", " << pt.X - from.X << "," << pt.Y - from.Y << ")";
        end   << "e(" << toString(z) << ", " << pt.X - to.X   << "," << pt.Y - to.Y   << ")";
        toString(z) = end.str();
        return CreateZ(begin.str());
    }
};

void figure8Test(Paths &test) {
    Path figure8;
    figure8.push_back(IntPoint(2000,1000,CreateZ("1")));
    figure8.push_back(IntPoint(4000,3000,CreateZ("2")));
    figure8.push_back(IntPoint(5000,2000,CreateZ("3")));
    figure8.push_back(IntPoint(4000,1000,CreateZ("4")));
    figure8.push_back(IntPoint(2000,3000,CreateZ("5")));
    figure8.push_back(IntPoint(1000,2000,CreateZ("6")));
    test << figure8;
}

void intersectionTest(Paths &test) {
    Path intersectionTestLargePart;
    intersectionTestLargePart << IntPoint( 300,  300, CreateZ("L1"))
                              << IntPoint( 600,  600, CreateZ("L2"))
                              << IntPoint( 700,  500, CreateZ("L3"))
                              << IntPoint( 500,  300, CreateZ("L4"))
                              << IntPoint( 600,  200, CreateZ("L5"))
                              << IntPoint( 700,  300, CreateZ("L6"))
                              << IntPoint( 300,  700, CreateZ("L7"))
                              << IntPoint( 100,  500, CreateZ("L8"));
    Path intersectionTestSmallPart;
    intersectionTestSmallPart << IntPoint( 300,  100, CreateZ("S1"))
                              << IntPoint( 500,  300, CreateZ("S2"))
                              << IntPoint( 300,  500, CreateZ("S3"))
                              << IntPoint( 100,  300, CreateZ("S4"));
    ReversePath(intersectionTestSmallPart);
    test << intersectionTestLargePart << intersectionTestSmallPart;
}

void horizontalEdgeTest(Paths &test) {
    Path horizontalEdgeTest;
    horizontalEdgeTest << IntPoint( 600, 800,  150)
                       << IntPoint( 700, 800,  250)
                       << IntPoint( 800, 800,  350)

                       << IntPoint( 800, 600,  450)
                       << IntPoint( 900, 600,  550)
                       << IntPoint(1000, 600,  650)

                       << IntPoint(1000, 400,  750)
                       << IntPoint( 900, 400,  850)
                       << IntPoint( 800, 400,  950)

                       << IntPoint( 800, 200, 1050)
                       << IntPoint( 700, 200, 1150)
                       << IntPoint( 600, 200, 1250)

                       << IntPoint( 600, 100, 1350)
                       << IntPoint( 500, 100, 1450)
                       << IntPoint( 400, 100, 1550)

                       << IntPoint( 400, 300, 1650)
                       << IntPoint( 300, 300, 1750)
                       << IntPoint( 200, 300, 1850)

                       << IntPoint( 200, 500, 1950)
                       << IntPoint( 300, 500, 2050)
                       << IntPoint( 400, 500, 2150)

                       << IntPoint( 400, 700, 2250)
                       << IntPoint( 500, 700, 2350)
                       << IntPoint( 600, 700, 2450);
    ReversePath(horizontalEdgeTest);
    test << horizontalEdgeTest;
}

void horizontalIntersectionTest(Paths &test) {
    Path mounds;
    mounds << IntPoint(100, 200, 150)
           << IntPoint(700, 200, 250)
           << IntPoint(600, 300, 350)
           << IntPoint(400, 100, 450)
           << IntPoint(200, 200, 550);
    test << mounds;
}

void horizontalJoinTest(Paths &test) {
    // 3|    12       22
    //  |   /  \     /  \
    // 2| 13-31-11|23-33-21
    //  |      \     /
    // 1|        32
    // 0+--+--+---+---+--+
    //  0  1  2   3   4  5
    Path horzJoinTopA;
    Path horzJoinTopB;
    Path horzJoinBot;
    horzJoinTopA << IntPoint(350, 200, 1150)
                 << IntPoint(200, 300, 1250)
                 << IntPoint(100, 200, 1350);
    horzJoinTopB << IntPoint(500, 200, 2150)
                 << IntPoint(400, 300, 2250)
                 << IntPoint(250, 200, 2350);
    horzJoinBot  << IntPoint(200, 200, 3150)
                 << IntPoint(300, 100, 3250)
                 << IntPoint(400, 200, 3350);
    test << horzJoinTopA << horzJoinTopB << horzJoinBot;
}

void joinTest(Paths &test) {
    Path base;
    Path tlTrig;
    Path trTrig;
    Path blTrig;
    Path brTrig;
    base << IntPoint(400, 700, 150)
         << IntPoint(100, 400, 250)
         << IntPoint(300, 200, 350)
         << IntPoint(500, 200, 450)
         << IntPoint(700, 400, 550);
    tlTrig << IntPoint(200, 600, 1150)
           << IntPoint(200, 500, 1250)
           << IntPoint(300, 600, 1350);
    trTrig << IntPoint(600, 600, 2150)
           << IntPoint(500, 600, 2250)
           << IntPoint(600, 500, 2350);
    blTrig << IntPoint(200, 100, 3150)
           << IntPoint(300, 200, 3250)
           << IntPoint(200, 300, 3350);
    brTrig << IntPoint(600, 100, 4150)
           << IntPoint(600, 300, 4250)
           << IntPoint(500, 200, 4350);
    test << base << tlTrig << trTrig << blTrig << brTrig;
}

void noncontributingIntersectionTest(Paths &test) {
    Path partA;
    Path partB;
    partA << IntPoint(100, 400, 1150)
          << IntPoint(400, 100, 1250)
          << IntPoint(500, 200, 1350)
          << IntPoint(200, 500, 1450);
    partB << IntPoint(400, 500, 2150)
          << IntPoint(100, 200, 2250)
          << IntPoint(200, 100, 2350)
          << IntPoint(500, 400, 2450);
    test << partA << partB;
}

void coincidentEdgeTest(Paths &test) {
    Path partA;
    Path partB;
    partA << IntPoint(350, 100, 1150)
          << IntPoint(200, 200, 1250)
          << IntPoint(100, 100, 1350);
    partB << IntPoint(500, 100, 2150)
          << IntPoint(400, 200, 2250)
          << IntPoint(250, 100, 2350);
    test << partA << partB;
}
void nightmareJoinTest(Paths &test) {
    Path center, tl, tr, bl, br;
    center << IntPoint(300, 100, 150)
           << IntPoint(500, 300, 250)
           << IntPoint(300, 500, 350)
           << IntPoint(100, 300, 450);
    tl    << IntPoint(100, 500, 1150)
          << IntPoint(100, 300, 1250)
          << IntPoint(300, 300, 1350)
          << IntPoint(300, 500, 1450);
    tr    << IntPoint(500, 500, 2150)
          << IntPoint(300, 500, 2250)
          << IntPoint(300, 300, 2350)
          << IntPoint(500, 300, 2450);
    bl    << IntPoint(100, 100, 3150)
          << IntPoint(300, 100, 3250)
          << IntPoint(300, 300, 3350)
          << IntPoint(100, 300, 3450);
    br    << IntPoint(500, 100, 4150)
          << IntPoint(500, 300, 4250)
          << IntPoint(300, 300, 4350)
          << IntPoint(300, 100, 4450);
    test << center << tl << tr << bl << br;
}
#endif

int main() {
    #ifdef use_xyz
    TestFollower zFill;
    Clipper clpr;
    clpr.PreserveCollinear(true);
    clpr.Callback(&zFill);
    Paths test;
    figure8Test(test);
    clpr.AddPaths(test, ptSubject, true);
    Paths solution;
    clpr.Execute(ctUnion, solution, pftNonZero);
    cout << "Clipper returned " << solution.size() << " paths." << endl;
    for (Path p : solution) {
        cout << "Path (" << p.size() << ")" << endl;
        for (IntPoint pt : p) {
            cout << "    (" << pt.X << ", " << pt.Y << ", " << toString(pt.Z) << ")" << endl;
        }
    }
    #endif
    return 0;
}
