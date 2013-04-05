#pragma once
#include <utils/EigenHelper.h>

namespace Eigen { typedef Matrix<double, 6, 1> Vector6d; }
namespace kinematics { class Skeleton; }

namespace atlas
{

class AtlasKinematics {
public:
	AtlasKinematics();
	virtual ~AtlasKinematics();

	void init(kinematics::Skeleton *_atlas);

	Eigen::Matrix4d legT(int _frame, double _u);
	Eigen::Matrix4d legFK(const Eigen::Vector6d& _u);
	Eigen::Matrix4d legFK(double _u1, double _u2, double _u3, double _u4, double _u5, double _u6);
	Eigen::MatrixXd legIK(const Eigen::Matrix4d& _Tf);
	Eigen::Vector6d legIK(const Eigen::Matrix4d& _Tf, const Eigen::Vector6d& _p);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	double l1;
	double l2;
	double l3;
	double l4;
	double u_off[6];  // offsets to joint zeros in context of DART/urdf
	double lr[7], la[7], lt[7], ld[7];  // leg dh parameters

	Eigen::Matrix4d _legT(int _frame, double _u);
	Eigen::Matrix4d _legFK(double _u1, double _u2, double _u3, double _u4, double _u5, double _u6);
	Eigen::MatrixXd _legIK(const Eigen::Matrix4d& _Tf);
};

}
