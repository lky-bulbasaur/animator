#include "C2InterpolatingCurveEvaluator.h"
#include <assert.h>
#include "mat.h"
#include "Matrix.h"

void C2InterpolatingCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
	std::vector<Point>& ptvEvaluatedCurvePts,
	const float& fAniLength,
	const bool& bWrap) const
{
	// Change this to smaller value for smoother curve
	// C2 interpolating splines need really small delta t to work
	float delta_t = 0.001;
	bool hacked = false;

	ptvEvaluatedCurvePts.clear();
	std::vector<Point> ctrlPts;
	ctrlPts.assign(ptvCtrlPts.begin(), ptvCtrlPts.end());
	int iCtrlPtCount = ctrlPts.size();

	if (bWrap) {
		// To achieve wrapping in B-spline, we reuse the existing control points
		// And place them out of bound
		// B-spline will then interpolate the curve should look like when out-of-bound, until the left and right bounds
		int ilCtrlPt1 = ctrlPts.size() - 1;
		int irCtrlPt1 = 0;

		Point lCtrlPt1 = Point(-(fAniLength - ctrlPts[ilCtrlPt1].x), ctrlPts[ilCtrlPt1].y);
		Point rCtrlPt1 = Point(fAniLength + ctrlPts[irCtrlPt1].x, ctrlPts[irCtrlPt1].y);

		ctrlPts.insert(ctrlPts.begin(), lCtrlPt1);
		ctrlPts.push_back(rCtrlPt1);
		if (iCtrlPtCount % 2 == 0) {
			ctrlPts.push_back(ctrlPts[ctrlPts.size() - 1]);
			hacked = true;
		}

		iCtrlPtCount = ctrlPts.size();
	}
	else {
		// The code somehow refuses to work only when there are even number control points?????
		if (iCtrlPtCount % 2 == 0) {
			ctrlPts.push_back(ctrlPts[ctrlPts.size() - 1]);
			hacked = true;
		}
		ptvEvaluatedCurvePts.push_back(Point(0, ctrlPts[0].y));
		iCtrlPtCount = ctrlPts.size();
	}

	Mat4<float> M = Mat4<float>(
		-1, 3, -3, 1,
		3, -6, 3, 0,
		-3, 3, 0, 0,
		1, 0, 0, 0
	);

	// We convert all control points to Bezier points
	// Starting with solving for matrix D (the derivative matrix)
	Matrix<float> B(iCtrlPtCount, iCtrlPtCount, 0);
	Matrix<float> D(iCtrlPtCount, 2, 0);
	Matrix<float> C(iCtrlPtCount, 2, 0);
	for (int i = 0; i < iCtrlPtCount; ++i) {
		// First we populate the matrix B (which is the very big matrix on the left, see "Curve Details")
		// Then we populate the matrix C (which is the right-most matrix with all those 3(Cm - Cm-2) things, see "Curve Details"
		if (i == 0) {
			B[0][0] = 2;
			B[0][1] = 1;

			C[0][0] = 3 * (ctrlPts[1].x - ctrlPts[0].x);
			C[0][1] = 3 * (ctrlPts[1].y - ctrlPts[0].y);
		} else if (i == iCtrlPtCount - 1) {
			B[iCtrlPtCount - 1][iCtrlPtCount - 2] = 1;
			B[iCtrlPtCount - 1][iCtrlPtCount - 1] = 2;

			C[iCtrlPtCount - 1][0] = 3 * (ctrlPts[iCtrlPtCount - 1].x - ctrlPts[iCtrlPtCount - 2].x);
			C[iCtrlPtCount - 1][1] = 3 * (ctrlPts[iCtrlPtCount - 1].y - ctrlPts[iCtrlPtCount - 2].y);
		} else {
			B[i][i - 1] = 1;
			B[i][i] = 4;
			B[i][i + 1] = 1;

			C[i][0] = 3 * (ctrlPts[i + 1].x - ctrlPts[i - 1].x);
			C[i][1] = 3 * (ctrlPts[i + 1].y - ctrlPts[i - 1].y);
		}
	}

	// Now matrix D contains the solved values
	D = B.Inv() * C;

	// See "Displaying B-splines" in "Curve Details"
	for (int i = 0; i < iCtrlPtCount - 1; ++i) {
		for (float t = 0; t < 1; t += delta_t) {
			Mat4<float> T = Mat4<float>(
				pow(t, 3), pow(t, 2), t, 1,
				0, 0, 0, 0,
				0, 0, 0, 0,
				0, 0, 0, 0
			);
			Mat4<float> G = Mat4<float>(
				ctrlPts[i].x, ctrlPts[i].y, 0, 0,
				ctrlPts[i].x + D[i][0] / 3.0, ctrlPts[i].y + D[i][1] / 3.0, 0, 0,
				ctrlPts[i + 1].x - D[i + 1][0] / 3.0, ctrlPts[i + 1].y - D[i + 1][1] / 3.0, 0, 0,
				ctrlPts[i + 1].x, ctrlPts[i + 1].y, 0, 0
			);
			// Seriously I don't know why it is the case but it works?????
			// If i don't change the derivative for something extremely big for the second-to-last control point
			// The draw curve exceeds that control point for some reason
			if ((hacked) && (i == iCtrlPtCount - 2)) {
				G[2][0] += 10000 * D[i + 1][0] / 3.0;
				G[2][1] += 10000 * D[i + 1][1] / 3.0;
			}

			Mat4<float> evaluatedPt = T * M * G;
			float x = evaluatedPt[0][0];
			float y = evaluatedPt[0][1];

			// Copied from Catmull-rom Spline
			if ((ptvEvaluatedCurvePts.size() == 0) || (x > ptvEvaluatedCurvePts.back().x)) {
				ptvEvaluatedCurvePts.push_back(Point(x, y));
			}
		}
	}

	if (!bWrap) {
		ptvEvaluatedCurvePts.push_back(Point(fAniLength, ctrlPts[iCtrlPtCount - 1].y));
	}

}
