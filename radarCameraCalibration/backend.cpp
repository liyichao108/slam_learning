#include "backend.hpp"

void MatchPointFile::load() {
	ifstram ifs(mFilePath);
	mPs.reserve(1000);
	while (!ifs.eof()) {
		std::vector<double> p(6, 0);
		for (int i = 0; i < 6; ++i) {
			ifs >> p[i];
		}
		mPs.push_back(p);
	}
	ifs.close();
}

void SolveBA(const MatchPointFile &mpf, const double imageSize[2], 
	std::vector &cameraPara) {
	
	double camera[7] = {0,0,0,0,-7,0,1500};
	ceres::Problem problem;
	auto &matchPoints = mpf.mPs;
	for (int i = 0; i < matchPoints.size(); ++i) {
		double u = matchPoints[i][0] - imageSize[0];
		double v = matchPoints[i][1] - 0.5 * matchPoints[i][3] - imageSize[1];
		double x = matchPoints[i][4];
		double y = matchPoints[i][5];
		
		ceres::CostFunction *cf = ReprojectionError.create(u, v, x, y);
		ceres::LossFunction *lf = new ceres::HuberLoss(5.0);
		
		problem.AddResidualBlock(cf, lf, camera);
	}
	
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
	ceres::Solver::Summary summary;
	ceres::Solver(options, &problem, &summary);
	std::cout << summary.FullReport() << "\n";
}
