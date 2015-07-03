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
//    if (!value) {
//        value = CreateZ("NULL"); // This is a failure.
//        cout << "Null!!!!" << endl;
//    }
    assert(value >= 10000); // this probably means we forgot to update the test
    return *reinterpret_cast<string *>(value);
}
float dist(const IntPoint &p1, const IntPoint &p2) {
    cInt dx = p1.X - p2.X;
    cInt dy = p1.Y - p2.Y;
    return sqrt(float(dx*dx + dy*dy));
}


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
    base << IntPoint(400, 700, CreateZ("C1"))
         << IntPoint(100, 400, CreateZ("C2"))
         << IntPoint(300, 200, CreateZ("C3"))
         << IntPoint(500, 200, CreateZ("C4"))
         << IntPoint(700, 400, CreateZ("C5"));
    tlTrig << IntPoint(200, 600, CreateZ("Tl1"))
           << IntPoint(200, 500, CreateZ("Tl2"))
           << IntPoint(300, 600, CreateZ("Tl3"));
    trTrig << IntPoint(600, 600, CreateZ("Tr1"))
           << IntPoint(500, 600, CreateZ("Tr2"))
           << IntPoint(600, 500, CreateZ("Tr3"));
    blTrig << IntPoint(200, 100, CreateZ("Bl1"))
           << IntPoint(300, 200, CreateZ("Bl2"))
           << IntPoint(200, 300, CreateZ("Bl3"));
    brTrig << IntPoint(600, 100, CreateZ("Br1"))
           << IntPoint(600, 300, CreateZ("Br2"))
           << IntPoint(500, 200, CreateZ("Br3"));
    test << base << tlTrig << trTrig << blTrig << brTrig;
}

void noncontributingIntersectionTest(Paths &test) {
    Path partA;
    Path partB;
    partA << IntPoint(100, 400, CreateZ("A1"))
          << IntPoint(400, 100, CreateZ("A2"))
          << IntPoint(500, 200, CreateZ("A3"))
          << IntPoint(200, 500, CreateZ("A4"));
    partB << IntPoint(400, 500, CreateZ("B1"))
          << IntPoint(100, 200, CreateZ("B2"))
          << IntPoint(200, 100, CreateZ("B3"))
          << IntPoint(500, 400, CreateZ("B4"));
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
    center << IntPoint(300, 100, CreateZ("X1"))
           << IntPoint(500, 300, CreateZ("X2"))
           << IntPoint(300, 500, CreateZ("X3"))
           << IntPoint(100, 300, CreateZ("X4"));
    tl    << IntPoint(100, 500, CreateZ("Tl1"))
          << IntPoint(100, 300, CreateZ("Tl2"))
          << IntPoint(300, 300, CreateZ("Tl3"))
          << IntPoint(300, 500, CreateZ("Tl4"));
    tr    << IntPoint(500, 500, CreateZ("Tr1"))
          << IntPoint(300, 500, CreateZ("Tr2"))
          << IntPoint(300, 300, CreateZ("Tr3"))
          << IntPoint(500, 300, CreateZ("Tr4"));
    bl    << IntPoint(100, 100, CreateZ("Bl1"))
          << IntPoint(300, 100, CreateZ("Bl2"))
          << IntPoint(300, 300, CreateZ("Bl3"))
          << IntPoint(100, 300, CreateZ("Bl4"));
    br    << IntPoint(500, 100, CreateZ("Br1"))
          << IntPoint(500, 300, CreateZ("Br2"))
          << IntPoint(300, 300, CreateZ("Br3"))
          << IntPoint(300, 100, CreateZ("Br4"));
    test << center << tl << tr << bl << br;
}

void offsetTest(Paths &test) {
    Path trig;
    trig << IntPoint(200, 200, CreateZ("BL"))
         << IntPoint(600, 200, CreateZ("BR"))
         << IntPoint(400, 400, CreateZ("TC"));
    test << trig;
}
#endif

int main() {
    #ifdef use_xyz
    TestFollower zFill;
    ClipperOffset clpr;
    clpr.Callback(&zFill);
    Paths test;
    offsetTest(test);
    clpr.AddPaths(test, jtMiter, etClosedPolygon);
    Paths solution;
    clpr.Execute(solution, 100);
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
