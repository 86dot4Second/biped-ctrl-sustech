#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

using namespace std;

namespace robot
{
	constexpr double PI = 3.141592653589793;

	class MoveJoint : public aris::core::CloneObject<MoveJoint, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		explicit MoveJoint(const string& name = "MoveJoint");
	private:
		double dir_;
	};

	class ReadPosition : public aris::core::CloneObject<ReadPosition, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		explicit ReadPosition(const string& name = "ReadPosition");

	};

	class TestMotor : public aris::core::CloneObject<TestMotor, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		explicit TestMotor(const string& name = "TestMotor");
	private:
		double dir_;
		int motornumber_;
	};

	class MoveEnd : public aris::core::CloneObject<MoveEnd, aris::plan::Plan> 
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		explicit MoveEnd(const string& name = "MoveEnd");
	private:
		double xl;
		double yl;
		double zl;
		double xr;
		double yr;
		double zr;
		double ll;
		double lr;
	};


	auto createControllerBiped()->unique_ptr<aris::control::Controller>;
	auto createPlanBiped()->unique_ptr<aris::plan::PlanRoot>;
}

#endif
