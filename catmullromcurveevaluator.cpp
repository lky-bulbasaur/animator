#include "CatmullromCurveEvaluator.h"
#include <assert.h>
#include "mat.h"

void CatmullromCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const
{
	// Change this to smaller value for smoother curve
	float delta_t = 0.05;

	ptvEvaluatedCurvePts.clear();
	std::vector<Point> ctrlPts;
	ctrlPts.assign(ptvCtrlPts.begin(), ptvCtrlPts.end());
	int iCtrlPtCount = ctrlPts.size();

	if (bWrap) {
		// To achieve wrapping in Catmull-rom curve, similar to B-spline, we reuse the existing control points
		// And place them out of bound
		// Catmull-rom curve will then interpolate the curve should look like when out-of-bound, until the left and right bounds
		int ilCtrlPt1 = ctrlPts.size() - 1;
		int ilCtrlPt2 = ctrlPts.size() - 2;
		int irCtrlPt1 = 0;
		int irCtrlPt2 = 1;

		Point lCtrlPt1 = Point(-(fAniLength - ctrlPts[ilCtrlPt1].x), ctrlPts[ilCtrlPt1].y);
		Point lCtrlPt2 = Point(-(fAniLength - ctrlPts[ilCtrlPt2].x), ctrlPts[ilCtrlPt2].y);
		Point rCtrlPt1 = Point(fAniLength + ctrlPts[irCtrlPt1].x, ctrlPts[irCtrlPt1].y);
		Point rCtrlPt2 = Point(fAniLength + ctrlPts[irCtrlPt2].x, ctrlPts[irCtrlPt2].y);

		ctrlPts.insert(ctrlPts.begin(), lCtrlPt1);
		ctrlPts.insert(ctrlPts.begin(), lCtrlPt2);
		ctrlPts.push_back(rCtrlPt1);
		ctrlPts.push_back(rCtrlPt2);
		iCtrlPtCount = ctrlPts.size();
	}
	else {
		// Special case for base case (where only 2 control points are here)
		// Draw a straight line (because a valid curve should have at least 1 control point between the 2 end points
		if (iCtrlPtCount <= 2) {
			ctrlPts[0].x = 0;
			ctrlPts[1].x = fAniLength;
		}
		// By repeating the end point once, we can force the first interpolated Catmull-Rom point to be exactly the first control point
		// i.e. The end points are defined by 2 points with the same coordinates
		// The reason only 2 points are needed instead of 3 is that Catmull-rom interpolates the next ONE point,
		// while B-spline interpolates the next TWO points
		ctrlPts.insert(ctrlPts.begin(), Point(0, ctrlPts[0].y));
		ctrlPts.push_back(Point(fAniLength, ctrlPts[ctrlPts.size() - 1].y));

		ptvEvaluatedCurvePts.push_back(Point(0, ctrlPts[0].y));
		iCtrlPtCount = ctrlPts.size();
	}

	Mat4<float> M = Mat4<float>(
		-1,	3,	-3,	1,
		2,	-5,	4,	-1,
		-1,	0,	1,	0,
		0,	2,	0,	0
		) * tension;

	// See "Displaying B-splines" in "Curve Details"
	// Catmull-rom curve is like a special case of B-spline so many code from B-spline implementation will be reused
	for (int i = 0; i < iCtrlPtCount - 3; ++i) {
		for (float t = 0; t < 1; t += delta_t) {
			Mat4<float> T = Mat4<float>(
				pow(t, 3), pow(t, 2), t, 1,
				0, 0, 0, 0,
				0, 0, 0, 0,
				0, 0, 0, 0
				);
			Mat4<float> G = Mat4<float>(
				ctrlPts[i].x,		ctrlPts[i].y,		0, 0,
				ctrlPts[i + 1].x,	ctrlPts[i + 1].y,	0, 0,
				ctrlPts[i + 2].x,	ctrlPts[i + 2].y,	0, 0,
				ctrlPts[i + 3].x,	ctrlPts[i + 3].y,	0, 0
				);
			Mat4<float> evaluatedPt = T * M * G;
			float x = evaluatedPt[0][0];
			float y = evaluatedPt[0][1];

			// See project website:
			// Note, however, that for an interpolating curve (Catmull-Rom), the fact that the control points are given to you sorted by x
			// does not ensure that the curve itself will also monotonically increase in x. You should recognize and handle this case appropriately.
			// One solution is to RETURN ONLY THE EVALUTED POINTS THAT ARE INCREASING MONOTONICALLY IN X.

			// Yes I spent 30mins staring at the vertical lines until I realize I can hard code it according to the hints
			if ((ptvEvaluatedCurvePts.size() == 0) || (x > ptvEvaluatedCurvePts.back().x)) {
				ptvEvaluatedCurvePts.push_back(Point(x, y));
			}
		}
	}

	if (!bWrap) {
		ptvEvaluatedCurvePts.push_back(Point(fAniLength, ctrlPts[iCtrlPtCount - 1].y));
	}

}

void CatmullromCurveEvaluator::setTension(float t)
{
	tension = t;
}
