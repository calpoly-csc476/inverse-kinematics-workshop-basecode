
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

	for (int t = 0; t < Joints.size(); ++ t)
	{
		Joints[t]->InboardLocation = Joints[t]->GetInboardLocation();
		Joints[t]->OutboardLocation = Joints[t]->GetOutboardLocation();
	}

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

	Joints[2]->OutboardLocation = CurrentGoal;

	const vec3 CurrentLine = normalize(Joints[2]->OutboardLocation - Joints[2]->InboardLocation);
	Joints[2]->InboardLocation = Joints[2]->OutboardLocation - CurrentLine * Joints[2]->Length;
}

void InverseKinematicsSolver::FABRIKStepTwo(glm::vec3 const & GoalPosition)
{
	Joints[2]->InboardLocation = Joints[1]->OutboardLocation;

	const vec3 CurrentLine = normalize(Joints[2]->InboardLocation - Joints[2]->OutboardLocation);
	Joints[2]->OutboardLocation = Joints[2]->InboardLocation - CurrentLine * Joints[2]->Length;
}

void InverseKinematicsSolver::ConvertPositionsToEulerAngles()
{
	// FABRIK calculates positions for each joint inboard/outboard, but we may want to have joints rotations
	// at the end for rendering. This code will compute Euler angles from the rotations, it may likely be
	// better for whatever application to use quaternions or matrices.
	//
	// This code does not need to be touched.

	mat4 CurrentTransform = mat4(1.f);

	for (int t = 0; t < Joints.size(); ++ t)
	{
		vec3 Direction = normalize(Joints[t]->OutboardLocation - Joints[t]->InboardLocation);

		// Transform into local (joint) space
		Direction = vec3(inverse(CurrentTransform) * vec4(Direction, 0.f));

		// Compute angle and axis
		const vec3 Axis = cross(vec3(1, 0, 0), Direction);
		const float Angle = acos(clamp(dot(Direction, vec3(1, 0, 0)), -1.f, 1.f)); // Clamp for numerical imprecision reasons

		if (length(Axis) < 0.0001f)
		{
			// Axis is small - means either 0 rotation or 180 rotation.
			// Axis of rotation does not matter, so let's just rotate around the up-vector
			Joints[t]->Rotation = vec3(0, Angle, 0);
		}
		else
		{
			// Make a rotation matrix, then use glm's built-in euler angle extraction!
			mat4 rotation = rotate(mat4(1.f), Angle, Axis);
			vec3 euler;
			extractEulerAngleXYZ(rotation, euler.x, euler.y, euler.z);

			Joints[t]->Rotation = euler;
		}

		CurrentTransform = CurrentTransform * Joints[t]->GetLocalRotation();
	}
}
