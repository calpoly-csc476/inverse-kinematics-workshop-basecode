
#include "InverseKinematics.h"

#include <iostream>
#include <glm/gtx/euler_angles.hpp>

using namespace std;
using namespace glm;


float clamp(const float a, const float min, const float max)
{
	return (a > max ? max : (a < min ? min : a));
}

float InverseKinematicsSolver::GetCurrentError(glm::vec3 const & GoalPosition) const
{
	vec3 const HandLoc = Joints.back()->GetOutboardLocation();
	return distance(GoalPosition, HandLoc);
}

void InverseKinematicsSolver::RunIK(glm::vec3 const & GoalPosition)
{
	if (FullReset)
	{
		for (auto Joint : Joints)
		{
			Joint->Rotation = glm::vec3(0);
		}
	}

	const int MaxSteps = 50;
	const float ErrorThreshold = 0.001f;

	for (int i = 0; i < MaxSteps; ++ i)
	{
		if (GetCurrentError(GoalPosition) < ErrorThreshold)
		{
			cout << "Found IK solution in " << i << " iterations." << endl;
			return;
		}
		else
		{
			StepFABRIK(GoalPosition);
		}
	}

	cout << "Exited IK attempt after " << MaxSteps << " iterations." << endl;
}

void InverseKinematicsSolver::StepFABRIK(glm::vec3 const & GoalPosition)
{
	vec3 RootPosition = Joints[0]->GetInboardLocation();

	// First pass - front to back
	FABRIKStepOne(GoalPosition);

	// Second pass - back to front
	FABRIKStepTwo(RootPosition);

	// Figure out Euler rotations for this configuration (e.g. for drawing/rigging)
	ConvertPositionsToEulerAngles();
}

void InverseKinematicsSolver::FABRIKStepOne(glm::vec3 const & GoalPosition)
{
	vec3 CurrentGoal = GoalPosition;

	for (int t = (int) Joints.size() - 1; t >= 0; -- t)
	{
		Joints[t]->InboardLocation = Joints[t]->GetInboardLocation();
		Joints[t]->OutboardLocation = Joints[t]->GetOutboardLocation();

		Joints[t]->OutboardLocation = CurrentGoal;
		const vec3 CurrentLine = normalize(Joints[t]->OutboardLocation - Joints[t]->InboardLocation);

		Joints[t]->InboardLocation = Joints[t]->OutboardLocation - CurrentLine * Joints[t]->Length;
		CurrentGoal = Joints[t]->InboardLocation;
	}

}

void InverseKinematicsSolver::FABRIKStepTwo(glm::vec3 const & GoalPosition)
{
	vec3 CurrentGoal = GoalPosition;

	for (int t = 0; t < Joints.size(); ++ t)
	{
		Joints[t]->InboardLocation = CurrentGoal;
		const vec3 CurrentLine = normalize(Joints[t]->InboardLocation - Joints[t]->OutboardLocation);

		Joints[t]->OutboardLocation = Joints[t]->InboardLocation - CurrentLine * Joints[t]->Length;
		CurrentGoal = Joints[t]->OutboardLocation;
	}
}

void InverseKinematicsSolver::ConvertPositionsToEulerAngles()
{
	// Convert inboard/outboard to direction vectors
	mat4 CurrentTransform = mat4(1.f);

	for (int t = 0; t < Joints.size(); ++ t)
	{
		vec3 Direction = normalize(Joints[t]->OutboardLocation - Joints[t]->InboardLocation);

		// Transform into local (joint) space
		Direction = vec3(inverse(CurrentTransform) * vec4(Direction, 0.f));

		const vec3 Axis = cross(vec3(1, 0, 0), Direction);
		const float Angle = acos(clamp(dot(Direction, vec3(1, 0, 0)), -1.f, 1.f)); // Clamp for numerical imprecision reasons

		if (length(Axis) < 0.0001f)
		{
			Joints[t]->Rotation = vec3(0, Angle, 0);
		}
		else
		{
			mat4 rotation = rotate(mat4(1.f), Angle, Axis);
			vec3 euler;
			extractEulerAngleXYZ(rotation, euler.x, euler.y, euler.z);

			Joints[t]->Rotation = euler;
		}

		CurrentTransform = CurrentTransform * Joints[t]->GetLocalTransform();
	}
}
