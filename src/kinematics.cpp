#include "kinematics.h"
#include <iostream>
#include <fstream>

//一些数学工具
//定义点乘
double dots(const Vec& x, const Vec& y) {
	assert(x.size() == y.size());
	double sum = 0;
	for (size_t i = 0; i < x.size(); ++i)
		sum += x[i] * y[i];
	return sum;
}

//定义三维叉乘
Vec cross(const Vec& x, const Vec& y) {
	assert(x.size() == y.size() && x.size() == 3);
	return Vec{ x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0] };
}

//定义模长
double norm(const Vec& x) {
	double val = 0;
	for (auto elem : x)
		val += elem * elem;
	return sqrt(val);
}

//已知两个向量求夹角
double iangle(const Vec& v1, const Vec& v2) {
	double angle_vector = acos(abs(dots(v1, v2)) / (norm(v1) * norm(v2)));
	return angle_vector;
}

//余弦定理
//已知三边长度求角度
double acosine(double a, double b, double c) {
	double angle_triangle = acos((a * a + b * b - c * c) / (2 * a * b));
	return angle_triangle;
}

//已知夹角和邻边求对边长度
double lcosine(double a, double b, double angle_C) {
	double length_triangle = sqrt(a * a + b * b - 2 * a * b * cos(angle_C));
	return length_triangle;
}

void BipedIK(const Vec& end_xyz_position, const Vec& end_abc_pointing, double end_on_foot, double input[5]) {
	
	double l = end_on_foot;

	const double AO = 154.5;
	const double AD = 150;
	const double CD = 422.02;
	const double AB = 487.46;
	const double BC = 88.82;
	const double BE = 509.51;
	const double CE = 426.61;
	const double EG = 326.82;
	const double HI = 280;
	const double GH = 120;
	const double EK = 40;
	const double IK = 85;
	const double KM = l - 85;
	const double EI = sqrt(IK * IK + EK * EK);

	Vec m = cross(end_xyz_position, end_abc_pointing);
	Vec p1 = cross(m, Vec{ 1, 0, 0 });
	Vec p2 = cross(p1, Vec{ 1, 0, 0 });
	double theta_1 = iangle(p1, Vec{ 0, 1, 0 });
	double theta_2 = iangle(p2, m);

	//求q1
	if (p1[2] <= 0)
		input[0] = theta_1;
	else
		input[0] = -theta_1;

	//求q2
	if (m[0] >= 0)
		input[1] = theta_2;
	else
		input[1] = -theta_2;

	//将基座标系转移到腿平面坐标系下
	Vec xl, yl, zl;
	for (int i = 0; i <= 2; ++i) {
		yl.push_back(p1[i] / norm(p1));
		zl.push_back(m[i] / norm(m));
	}
	xl = cross(yl, zl);

	Vec M0 = { end_xyz_position[0] * xl[0] + end_xyz_position[1] * xl[1] + end_xyz_position[2] * xl[2], end_xyz_position[0] * yl[0] + end_xyz_position[1] * yl[1] + end_xyz_position[2] * yl[2] };
	Vec n0 = { end_abc_pointing[0] * xl[0] + end_abc_pointing[1] * xl[1] + end_abc_pointing[2] * xl[2], end_abc_pointing[0] * yl[0] + end_abc_pointing[1] * yl[1] + end_abc_pointing[2] * yl[2] };

	//求出点E坐标
	double a_EIM = atan(EK/IK);
	double theta_3 = atan(n0[1]/n0[0]);
	double theta_4;
	Vec E_leg;
	if (n0[0] == 0) {
		if (n0[1] >= 0) {
			E_leg = { M0[0] - EK, M0[1] - KM };
			theta_4 = PI / 2 + a_EIM;
		}
		else {
			E_leg = { M0[0] - EK, M0[1] - KM };
			theta_4 = -PI / 2 + a_EIM;
		}
	}
	else {
		E_leg = { M0[0] - KM * cos(theta_3) - EK * sin(theta_3), M0[1] - KM * sin(theta_3) + EK * cos(theta_3) };
		theta_4 = theta_3 + a_EIM;
	}

	//求出腿平面内其他角度
	Vec v_AE = { E_leg[0] - 0, E_leg[1] + AO };
	double AE = norm(v_AE);
	double theta_5 = iangle(v_AE, Vec{ 0, 1 });
	double a_ABE = acosine(AB, BE, AE);
	double a_CBE = acosine(BC, BE, CE);
	double a_ABC = a_ABE - a_CBE;
	double AC = lcosine(AB, BC, a_ABC);
	double a_ADC = acosine(AD, CD, AC);//角ADC与q4相关
	double a_BAC = acosine(AB, AC, BC);
	double a_CAD = acosine(AC, AD, CD);
	double a_BAD = a_BAC + a_CAD;
	double a_BAE = acosine(AB, AE, BE);
	double a_AEB = acosine(AE, BE, AB);
	double a_AEC = acosine(AE, CE, AC);
	double a_BEC = acosine(BE, CE, BC);
	
	double a_OAD;//角OAD与q3相关
	if (M0[0] >= 0)
		a_OAD = PI - (a_BAD - a_BAE + theta_5);
	else
		a_OAD = PI - (a_BAD - a_BAE - theta_5);

	double theta_6;
	if (a_AEB >= a_BEC) {
		if (M0[0] >= 0) 
			theta_6 = PI / 2 - theta_5 - a_AEC;
		else
			theta_6 = PI / 2 + theta_5 - a_AEC;
	}
	else {
		if (M0[0] >= 0) 
			theta_6 = PI / 2 - theta_5 + a_AEC;
		else
			theta_6 = PI / 2 + theta_5 + a_AEC;
	}
	double a_GEI = theta_4 + theta_6;
	double GI = lcosine(EG, EI, a_GEI);
	double a_EGI = acosine(GI, EG, EI);
	double a_HGI = acosine(GI, GH, HI);
	double a_EGH = a_HGI + a_EGI;//角EGH与q5相关

	//求出初始状态时腿平面各角度
	//设定q4初始角度（角ADC）为3/4π，点A与点E处在同一竖直线上
	double q40 = 3.0 / 4.0 * PI;

	double AC_0 = lcosine(AD, CD, q40);
	double a_BAD_0 = acosine(AC_0, AB, BC) + acosine(AC_0, AD, CD);
	double a_ABE_0 = acosine(AB, BC, AC_0) + a_CBE;
	double AE_0 = lcosine(AB, BE, a_ABE_0);
	double a_BAE_0 = acosine(AB, AE_0, BE);
	double q30 = PI - (a_BAD_0 - a_BAE_0);

	double a_AEC_0 = acosine(AE_0, CE, AC_0);
	double a_GEI_0 = PI/2 - a_AEC_0 + a_EIM;
	double GI_0 = lcosine(EG, EI, a_GEI_0);
	double q50 = acosine(GI_0, EG, EI) + acosine(GI_0, GH, HI);

	//求出q3,q4,q5
	input[2] = q30 - a_OAD;
	input[3] = a_ADC - q40;
	input[4] = q50 - a_EGH;

}

//int main() {
//	Vec end_position = { 93.5964, -717.1136, 3.231 };
//	Vec end_pointing = { 32.9283, 140.4168, 41.217 };
//	double end_foot = 150;
//	double input_angle[5] = { 0 };
//	BipedIK(end_position, end_pointing, end_foot, input_angle);
//	for (auto& x : input_angle)
//		cout << x << endl;
//	return 0;
//}