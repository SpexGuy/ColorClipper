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
    virtual void OnIntersection(IntPoint& e1bot, IntPoint& e1top, EdgeSide e1Side, bool e1Forward,
                                IntPoint& e2bot, IntPoint& e2top, EdgeSide e2Side, bool e2Forward,
                                const IntPoint &pt, cInt& z1, cInt& z2) {
        // I was really hoping that there was some great gestalt of understanding for this.
        // Instead I just found some strangely ordered chaos.  So this routine is the result
        // of the enumeration of all 16 possible cases.
        // I have attempted to catalogue my thoughts about possible reasons in the comments.

        // First, identify the begin and end points according to the original polygon direction
        IntPoint& e1End   = e1Forward ? e1top : e1bot;
        IntPoint& e1Begin = e1Forward ? e1bot : e1top;
        IntPoint& e2End   = e2Forward ? e2top : e2bot;
        IntPoint& e2Begin = e2Forward ? e2bot : e2top;
        // Then, associate them with the side of the split they will contribute to.
        IntPoint& e1SplitEnd   = (e1Side == esLeft) ? e1End   : e2End;
        IntPoint& e1SplitBegin = (e1Side == esLeft) ? e1Begin : e2Begin;
        IntPoint& e2SplitEnd   = (e2Side == esLeft) ? e2End   : e1End;
        IntPoint& e2SplitBegin = (e2Side == esLeft) ? e2Begin : e1Begin;
        // Then we calculate other useful variables
        // I have no idea why this is how it is, but it is:
        bool e1Reverse = (e1Side == esLeft) ? !e1Forward : e2Forward;
        bool e2Reverse = (e2Side == esLeft) ? !e2Forward : e1Forward;
        bool e1StripBeginning = !e1Reverse;
        bool e2StripBeginning = !e2Reverse;
        bool replace = (e1Side != e2Side); //equivalent to e1SplitEnd == e2SplitEnd
        if (replace) {
            // Replace the side which is esRight
            // This replacement sets up the next set of points (above the intersection) to output correctly
            bool replaceSideIsE1 = (e1Side == esRight);
            // Associate points with their purpose
            IntPoint& replaceEnd       = replaceSideIsE1 ? e1End   : e2End;
            IntPoint& replaceBegin     = replaceSideIsE1 ? e1Begin : e2Begin;
            // Always strip the lower (closer to top) end. It is not reversed here (On*Intermediate will do that).
            bool replaceStripBeginning = replaceSideIsE1 ? !e1Forward : !e2Forward;
            IntPoint& otherEnd         = replaceSideIsE1 ? e2End   : e1End;
            IntPoint& otherBegin       = replaceSideIsE1 ? e2Begin : e1Begin;
            bool otherStripBeginning   = replaceSideIsE1 ? e2StripBeginning : e1StripBeginning;
            replaceEnd.Z = Strip(replaceStripBeginning, replaceEnd.Z, replaceBegin, replaceEnd, pt);
            // Our splitPoints are equal, so just split one of them
            z1 = Strip(e1StripBeginning, otherEnd.Z, otherBegin, otherEnd, pt);
            z2 = otherEnd.Z; // the other gets the unstripped half
        } else {
            z1 = Strip(e1StripBeginning, e1SplitEnd.Z, e1SplitBegin, e1SplitEnd, pt);
            z2 = Strip(e2StripBeginning, e2SplitEnd.Z, e2SplitBegin, e2SplitEnd, pt);
        }
        if (e1Reverse) z1 = ReverseZ(z1);
        if (e2Reverse) z2 = ReverseZ(z2);
    }
    virtual void OnLeftIntermediate(IntPoint& bot, IntPoint& top, IntPoint& next, bool forward, IntPoint& pt) {
        pt.Z = (forward ? top.Z : ReverseZ(bot.Z));
    }
    virtual void OnRightIntermediate(IntPoint& bot, IntPoint& top, IntPoint& next, bool forward, IntPoint& pt) {
        pt.Z = (forward ? ReverseZ(next.Z) : top.Z);
    }
    virtual void OnLocalMin(IntPoint& right, IntPoint& bot, IntPoint& left, bool leftIsForward, IntPoint& pt) {
        pt.Z = (leftIsForward ? bot.Z : ReverseZ(right.Z));
    }
    virtual void OnLocalMax(IntPoint& left, IntPoint& top, IntPoint& right, bool leftIsForward, IntPoint& pt) {
        pt.Z = (leftIsForward ? ReverseZ(left.Z) : top.Z);
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
private:
    inline cInt Strip(bool stripBeginning, cInt& z, IntPoint& from, IntPoint& to, const IntPoint& pt) {
        if (stripBeginning) {
            return StripBegin(z, from, to, pt);
        } else {
            return StripEnd(z, from, to, pt);
        }
    }

};
