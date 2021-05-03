#include "BezierCurveEvaluator.h"
#include <assert.h>
#include "mat.h"

void BezierCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
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
		// If there are not 3k control points, use linear interpolation instead of Bezier curve evaluation for the end points
		// Because it is impossible to form a Bezier curve using the end points under this condition
		if (iCtrlPtCount % 3 != 0) {
			ptvEvaluatedCurvePts.push_back(Point(0, (ctrlPts[0].y + ctrlPts[iCtrlPtCount - 1].y) / 2));
		}

		// Add the imaginary control point to the end of the control point list
		ctrlPts.push_back(Point(fAniLength + ctrlPts[0].x, ctrlPts[0].y));
		iCtrlPtCount = ctrlPts.size();
	} else {
		ptvEvaluatedCurvePts.push_back(Point(0, ctrlPts[0].y));
	}

	Mat4<float> M = Mat4<float>(
		-1,	3,	-3,	1,
		3,	-6,	3,	0,
		-3,	3,	0,	0,
		1,	0,	0,	0
	);

	int i = 0;
	// Bezier curve 
	while (iCtrlPtCount - i >= 4) {
		for (float t = 0; t < 1; t += delta_t) {
			Mat4<float> T = Mat4<float>(
				pow(t, 3),	pow(t, 2),	t,	1,
				0,			0,			0,	0,
				0,			0,			0,	0,
				0,			0,			0,	0
			);
			Mat4<float> G = Mat4<float>(
				ctrlPts[i].x,		ctrlPts[i].y,	0,	0,
				ctrlPts[i + 1].x,	ctrlPts[i + 1].y,	0,	0,
				ctrlPts[i + 2].x,	ctrlPts[i + 2].y,	0,	0,
				ctrlPts[i + 3].x,	ctrlPts[i + 3].y,	0,	0
			);
			Mat4<float> evaluatedPt = T * M * G;
			float x = evaluatedPt[0][0];
			float y = evaluatedPt[0][1];
			if (x > fAniLength) x -= fAniLength;
			ptvEvaluatedCurvePts.push_back(Point(x, y));
		}
		i += 3;
	}

	for (int j = i; j < iCtrlPtCount; ++j) {
		ptvEvaluatedCurvePts.push_back(ctrlPts[j]);
	}

	if (!bWrap) {
		ptvEvaluatedCurvePts.push_back(Point(fAniLength, ctrlPts[iCtrlPtCount - 1].y));
	}
	
}
