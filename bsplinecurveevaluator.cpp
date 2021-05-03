#include "BsplineCurveEvaluator.h"
#include <assert.h>
#include "mat.h"

void BsplineCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const
{
	// Change this to smaller value for smoother curve
	float delta_t = 0.01;

	ptvEvaluatedCurvePts.clear();
	std::vector<Point> ctrlPts;
	ctrlPts.assign(ptvCtrlPts.begin(), ptvCtrlPts.end());
	int iCtrlPtCount = ctrlPts.size();
	
	if (bWrap) {
		// To achieve wrapping in B-spline, we reuse the existing control points
		// And place them out of bound
		// B-spline will then interpolate the curve should look like when out-of-bound, until the left and right bounds
		int ilCtrlPt1 = ctrlPts.size() - 1;
		int ilCtrlPt2 = ctrlPts.size() - 2;
		int ilCtrlPt3 = ctrlPts.size() - 3; if (ilCtrlPt3 < 0) ilCtrlPt3 = 1;
		int irCtrlPt1 = 0;
		int irCtrlPt2 = 1;
		int irCtrlPt3 = 2; if (irCtrlPt3 >= ctrlPts.size()) irCtrlPt3 = 0;

		Point lCtrlPt1 = Point(-(fAniLength - ctrlPts[ilCtrlPt1].x), ctrlPts[ilCtrlPt1].y);
		Point lCtrlPt2 = Point(-(fAniLength - ctrlPts[ilCtrlPt2].x), ctrlPts[ilCtrlPt2].y);
		Point lCtrlPt3 = Point(-(2 * fAniLength - ctrlPts[ilCtrlPt3].x), ctrlPts[ilCtrlPt3].y);
		Point rCtrlPt1 = Point(fAniLength + ctrlPts[irCtrlPt1].x, ctrlPts[irCtrlPt1].y);
		Point rCtrlPt2 = Point(fAniLength + ctrlPts[irCtrlPt2].x, ctrlPts[irCtrlPt2].y);
		Point rCtrlPt3 = Point(2 * fAniLength + ctrlPts[irCtrlPt3].x, ctrlPts[irCtrlPt3].y);

		ctrlPts.insert(ctrlPts.begin(), lCtrlPt1);
		ctrlPts.insert(ctrlPts.begin(), lCtrlPt2);
		ctrlPts.insert(ctrlPts.begin(), lCtrlPt3);
		ctrlPts.push_back(rCtrlPt1);
		ctrlPts.push_back(rCtrlPt2);
		ctrlPts.push_back(rCtrlPt3);
		iCtrlPtCount = ctrlPts.size();
	}
	else {
		// By repeating the end point twice, we can force the first interpolated De Boor point to be exactly the first control point
		// i.e. The end points are defined by 3 points with the same coordinates
		ctrlPts.insert(ctrlPts.begin(), ctrlPts[0]);
		ctrlPts.insert(ctrlPts.begin(), ctrlPts[0]);

		// Only also bind the right-end point if a valid spline is made
		// i.e. there is at least a control point between the two end points
		if (ctrlPts.size() > 4) {
			ctrlPts.push_back(ctrlPts[ctrlPts.size() - 1]);
			ctrlPts.push_back(ctrlPts[ctrlPts.size() - 1]);
		}

		ptvEvaluatedCurvePts.push_back(Point(0, ctrlPts[0].y));
		iCtrlPtCount = ctrlPts.size();
	}

	Mat4<float> M = Mat4<float>(
		-1, 3, -3, 1,
		3, -6, 3, 0,
		-3, 0, 3, 0,
		1, 4, 1, 0
		) / 6.0;

	// See "Displaying B-splines" in "Curve Details"
	for (int i = 0; i < iCtrlPtCount - 3; ++i) {
		for (float t = 0; t < 1; t += delta_t) {
			Mat4<float> T = Mat4<float>(
				pow(t, 3), pow(t, 2), t, 1,
				0, 0, 0, 0,
				0, 0, 0, 0,
				0, 0, 0, 0
				);
			Mat4<float> G = Mat4<float>(
				ctrlPts[i].x, ctrlPts[i].y, 0, 0,
				ctrlPts[i + 1].x, ctrlPts[i + 1].y, 0, 0,
				ctrlPts[i + 2].x, ctrlPts[i + 2].y, 0, 0,
				ctrlPts[i + 3].x, ctrlPts[i + 3].y, 0, 0
				);
			Mat4<float> evaluatedPt = T * M * G;
			float x = evaluatedPt[0][0];
			float y = evaluatedPt[0][1];
			ptvEvaluatedCurvePts.push_back(Point(x, y));
		}
	}

	if (!bWrap) {
		ptvEvaluatedCurvePts.push_back(Point(fAniLength, ctrlPts[iCtrlPtCount - 1].y));
	}

}
