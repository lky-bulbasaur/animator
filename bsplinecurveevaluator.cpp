#include "BsplineCurveEvaluator.h"
#include <assert.h>
#include "mat.h"

void BsplineCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const
{
	/*
	int iCtrlPtCount = ptvCtrlPts.size();
	std::vector<Point> bezierPts;
	bezierPts.assign(ptvCtrlPts.begin(), ptvCtrlPts.end());

	Mat4<float> basis = Mat4<float>(
		-1,	3,	-3,	1,
		3,	-6,	3,	0,
		-3,	0,	3,	0,
		1,	4,	1,	0
	) / 6.0;

	for (int i = 0; i < iCtrlPtCount - 3; ++i) {
		Mat4<float> deboorPts = Mat4<float>(
			ptvCtrlPts[i].x,		ptvCtrlPts[i].y,		0,	0,
			ptvCtrlPts[i + 1].x,	ptvCtrlPts[i + 1].y,	0,	0,
			ptvCtrlPts[i + 2].x,	ptvCtrlPts[i + 2].y,	0,	0,
			ptvCtrlPts[i + 3].x,	ptvCtrlPts[i + 3].y,	0,	0
		);
		Mat4<float> temp = basis * deboorPts;
		ptvEvaluatedCurvePts[i] = Point(bezierPts[0][0], bezierPts[0][1]);
		ptvEvaluatedCurvePts[i + 1] = Point(bezierPts[1][0], bezierPts[1][1]);
		ptvEvaluatedCurvePts[i + 2] = Point(bezierPts[2][0], bezierPts[2][0]);
		ptvEvaluatedCurvePts[i + 3] = Point(bezierPts[3][0], bezierPts[3][1]);
		printf("%f, %f\n", bezierPts[0][0], bezierPts[0][1]);
	}
	*/
}
