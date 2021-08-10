#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <cmath>
#include <vector>
#include <cassert>

using namespace std;
typedef vector<double> Vec;

const double PI = 3.14159265358979323846;
double dots(const Vec& x, const Vec& y);
Vec cross(const Vec& x, const Vec& y);
double norm(const Vec& x);
double iangle(const Vec& v1, const Vec& v2);
double acosine(double a, double b, double c);
double lcosine(double a, double b, double angle_C);
void BipedIK(const Vec& end_xyz_position, const Vec& end_abc_pointing, double end_on_foot, double input[5]);
#endif

