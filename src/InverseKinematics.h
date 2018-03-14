
#pragma once

#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>


class InverseKinematicsSolver
{

public:

	struct SJoint
	{
		SJoint * Parent = nullptr;
		glm::vec3 Rotation = glm::vec3(0, 0, 0);
		float Length = 0.75f;

		glm::vec3 InboardLocation, OutboardLocation;

		glm::mat4 GetLocalRotation() const
		{
			glm::mat4 Rot = glm::mat4(1.f);
			Rot = glm::rotate(Rot, Rotation.x, glm::vec3(1, 0, 0));
			Rot = glm::rotate(Rot, Rotation.y, glm::vec3(0, 1, 0));
			Rot = glm::rotate(Rot, Rotation.z, glm::vec3(0, 0, 1));

			return Rot;
		}

		glm::mat4 GetInboardTransformation() const
		{
			//  _____    ___      ____     ___  
			// |_   _|  / _ \    |  _ \   / _ \ 
			//   | |   | | | |   | | | | | | | |
			//   | |   | |_| |   | |_| | | |_| |
			//   |_|    \___/    |____/   \___/ 
			//

			return glm::mat4(1.f);
		}

		glm::mat4 GetOutboardTransformation() const
		{
			//  _____    ___      ____     ___  
			// |_   _|  / _ \    |  _ \   / _ \ 
			//   | |   | | | |   | | | | | | | |
			//   | |   | |_| |   | |_| | | |_| |
			//   |_|    \___/    |____/   \___/ 
			//

			return glm::mat4(1.f);
		}

		glm::vec3 GetOutboardLocation() const
		{
			glm::vec4 v(0, 0, 0, 1);
			v = GetOutboardTransformation() * v;

			return glm::vec3(v.x, v.y, v.z);
		}

		glm::vec3 GetInboardLocation() const
		{
			glm::vec4 v(0, 0, 0, 1);
			v = GetInboardTransformation() * v;

			return glm::vec3(v.x, v.y, v.z);
		}
	};

	std::vector<SJoint *> Joints;
	bool FullReset = false;

	float GetCurrentError(glm::vec3 const & GoalPosition) const;

	void RunIK(glm::vec3 const & GoalPosition);

	void StepFABRIK(glm::vec3 const & GoalPosition);
	void FABRIKStepOne(glm::vec3 const & GoalPosition);
	void FABRIKStepTwo(glm::vec3 const & GoalPosition);

	void ConvertPositionsToEulerAngles();

};
