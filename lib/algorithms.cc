#include "algorithms.h"
#include <algorithm>

void setHeading(RotationalState currentStates, RotationalState* goalStates, int newHeading)
{
    if (currentStates.heading > newHeading)
    {
		goalStates->w = MINW;
    }
    else if (currentStates.heading < newHeading)
    {
		goalStates->w = MAXW;
    }
    else
    {
		goalStates->w = 0;
    }
	goalStates->heading = newHeading;
}

void collisionAvoidanceBrake(VehicleState2f* goalScalar)
{
    goalScalar->w = 0;
    goalScalar->heading = goalScalar->heading;
    goalScalar->acc = MINACC;
    goalScalar->speed = MINSPEED;
}

void collisionAvoidanceAccelerate(VehicleState2f* goalScalar)
{
	goalScalar->w = 0;
	goalScalar->heading = goalScalar->heading;
	goalScalar->acc = MAXACC;
	goalScalar->speed = MAXSPEED;
}

void collisionAvoidanceNaive(int obsNum, vector<Point> obsPosition, vector<VehicleState2f> obsStates,
	Point avPosition, VehicleState2f avStates, Point goalPosition, VehicleState2f* goalStates)
{
	double distance = 0.0, min = 32767.0, disBeforeStop = 0.0;
	Point2f projection, current = { avPosition.x, avPosition.y }, goal = { goalPosition.x, goalPosition.y };
	int closest = -1;

	for (int i = 0; i < obsNum; i++)
	{
		if (isAhead(avPosition, goalPosition, obsPosition[i]))
		{
			Point2f obs = { obsPosition[i].x, obsPosition[i].y };
			distance = getDisPointToLine(current, goal, obs, &projection);
			if (distance < min)
			{
				closest = i;
			}
		}
	}
	if (closest != -1)
	{
		Point2f obs = { obsPosition[closest].x, obsPosition[closest].y };
		getDisPointToLine(current, goal, obs, &projection);
		distance = getDisPointToPoint(current, projection);
		disBeforeStop = pow(avStates.speed, 2) / abs(MINACC);

		if (disBeforeStop < distance)
		{
			//can stop timely
			collisionAvoidanceBrake(goalStates);
		}
		else
		{
			//cannot stop timely
			collisionAvoidanceAccelerate(goalStates);
		}
	}

}

Point2f Theta2FlowForce(Point2f current, vector<Point> path_) {
	double s;
	int num_of_segment = path_.size() - 1;
	double* x = new double[num_of_segment];
	double* y = new double[num_of_segment];
	double* nx = new double[num_of_segment];
	double* ny = new double[num_of_segment];
	double* segment_len = new double[num_of_segment];
	//std::reverse(path_.begin(), path_.end());
	Point2f end = { (double)path_[num_of_segment].x, (double)path_[num_of_segment].y };


	for (int i = 0; i < num_of_segment; ++i) {
		x[i] = (double)path_[i].x;
		y[i] = (double)path_[i].y;
		nx[i] = (double)path_[i + 1].x - (double)path_[i].x;
		ny[i] = (double)path_[i + 1].y - (double)path_[i].y;
		s = nx[i] * nx[i] + ny[i] * ny[i];
		s = sqrt(s);
		segment_len[i] = s;
		nx[i] /= s;
		ny[i] /= s;
	}

	double min_dist = 0.0, dist = 0.0, tmp = 0.0, alpha = 0.0;
	Point2f return_force = { 0.0,0.0 }, force = { 0.0,0.0 }, follow_force = { 0.0,0.0 };

	min_dist = sqrt((current.x - end.x) * (current.x - end.x) + (current.y - end.y) * (current.y - end.y)) + 1e-12;
	return_force.x = -(current.x - end.x) / min_dist;
	return_force.y = -(current.y - end.y) / min_dist;
	follow_force = return_force;


	/*for (int i = 0; i < num_of_segment; ++i) {
		force.x = x[i] - current.x;
		force.y = y[i] - current.y;
		tmp = force.x * nx[i] + force.y * ny[i];
		force.x = force.x - tmp * nx[i];
		force.y = force.y - tmp * ny[i];
		dist = sqrt(force.x * force.x + force.y * force.y) + 1e-12;
		if (dist < min_dist && tmp <= 0 && abs(tmp) <= segment_len[i]) {
			min_dist = dist;
			return_force.x = force.x / dist;
			return_force.y = force.y / dist;
			follow_force.x = nx[i];
			follow_force.y = ny[i];
		}

		dist = sqrt((current.x - x[i]) * (current.x - x[i]) + (current.y - y[i]) * (current.y - y[i])) + 1e-12;
		if (dist < min_dist) {
			return_force.x = -(current.x - x[i]) / dist;
			return_force.y = -(current.y - y[i]) / dist;
			follow_force.x = nx[i];
			follow_force.y = ny[i];
			min_dist = dist;
		}

	}*/

	if (min_dist < FLOWFIELD_SCOPE) {
		//double k1 = 1.0 / 100, k2 = 1.0;
		//alpha = exp(-min_dist * k1);
		//return_force = fplus(fmultiply(return_force, (1 - alpha)), fmultiply(follow_force, (k2 * alpha)));
	}
	else {
		//return_force.x = 0;
		//return_force.y = 0;
	}

	return return_force;
}

Point2f DipoleForce(Point2f loc_agent, Point2f moment_agent, Point2f loc_obj, Point2f moment_obj) {
	float reversed_angles = M_PI + M_PI / 4;
	float mx, my;
	float fx = 0, fy = 0;
	float rx, ry, r2;
	mx = moment_agent.x;
	my = moment_agent.y;
	float m_obj_x = moment_obj.x;
	float m_obj_y = moment_obj.y;
	rx = loc_obj.x - loc_agent.x; //a vector from object to the agent
	ry = loc_obj.y - loc_agent.y;
	r2 = rx * rx + ry * ry + 1e-12; //a small term 1e-12 is added into the denominator to avoid singularities
	rx = rx / sqrt(r2);
	ry = ry / sqrt(r2);

	fx = 0;
	fy = 0;

	fx += (m_obj_x * rx + m_obj_y * ry) * mx;
	fy += (m_obj_x * rx + m_obj_y * ry) * my;

	fx += (mx * rx + my * ry) * m_obj_x;
	fy += (mx * rx + my * ry) * m_obj_y;

	fx += (mx * m_obj_x + my * m_obj_y) * rx;
	fy += (mx * m_obj_x + my * m_obj_y) * ry;

	fx -= 5 * (mx * rx + my * ry) * (m_obj_x * rx + m_obj_y * ry) * rx;
	fy -= 5 * (mx * rx + my * ry) * (m_obj_x * rx + m_obj_y * ry) * ry;

	fx = 1000 * fx / pow(r2 * r2, GAMA);
	fy = 1000 * fy / pow(r2 * r2, GAMA);

	float sign = (fx * rx + fy * ry);
	float angle = atan2(fy, fx);

	if (sign > 0) { //In some case it has an opposite direction of a
		//vector pointing from an agent k to an agent j, need to be reversed by rotation
		/*float new_fx = fx * cos(reversed_angles) - fx * sin(reversed_angles);
		float new_fy = fx * sin(reversed_angles) + fx * cos(reversed_angles);
		fx = new_fx;
		fy = new_fy;*/
		fx = -fx;
		fy = -fy;
	}

	Point2f r = { fx,fy };

	return r;
}

void collisionAvoidanceDFF(int print, int obsNum, vector<Point> obsPosition, vector<VehicleState2f> obsStates,
	Point avPosition, VehicleState2f avStates, Point goalPosition, Point path[], int pathLen, VehicleState2f* goalStates)
{
	Point2f current = { avPosition.x, avPosition.y }, obsCurrent;
	Point2f currentVelocity = { (avStates.speed) * cos(avStates.heading), (avStates.speed) * sin(avStates.heading) }, obsVelocity;
	vector<Point> path_;
	for (int i = 0; i < pathLen; i++)
	{
		path_.push_back(path[i]);
	}

	Point2f force1 = Theta2FlowForce(current, path_);

	Point2f force2 = { 0.0,0.0 };
	Point2f staticForce = fplus(force1, force2);
	float s = staticForce.x * staticForce.x + staticForce.y * staticForce.y + 1e-12;
	s = sqrt(s);
	staticForce.x /= s; 
	staticForce.x /= 10;
	staticForce.y /= s; 
	staticForce.y /= 10;
#ifdef DEBUG
	if (print)
	{
		printf("static force: %f,%f\n", staticForce.x, staticForce.y);
	}
#endif // DEBUG


	Point2f dipoleForce = { 0.0,0.0 };
	for (int i = 0; i < obsNum; i++)
	{
		obsCurrent = { (double)obsPosition[i].x, (double)obsPosition[i].y };
		obsVelocity = { (obsStates[i].speed) * cos(obsStates[i].heading), (obsStates[i].speed) * sin(obsStates[i].heading) };
		Point2f force3 = DipoleForce(current, currentVelocity, obsCurrent, obsVelocity);
		Point2f force4 = fplus(staticForce, force3);
		dipoleForce = fplus(dipoleForce, force4);
#ifdef DEBUG
		if (print == 1)
		{
			printf("dipole force: %f,%f\n", force3.x, force3.y);
			printf("combined force: %f,%f\n", dipoleForce.x, dipoleForce.y);
		}
#endif // DEBUG
	}
	if (dipoleForce.x != 0.0 && dipoleForce.y != 0.0)
	{
		goalStates->heading = atan2(dipoleForce.y, dipoleForce.x);
	}
	else
	{
		//
	}
}

double turn(Point obsPosition, Point avPosition, double currentHeading, VehicleState2f goalStates2f)
{
	double newHeading = goalStates2f.heading;
	double distance = getDisPointToPoint(avPosition.x, avPosition.y, obsPosition.x, obsPosition.y);
	Point newPosition = { avPosition.x + getProjection(goalStates2f.speed, (int)(newHeading * ANGULARSCALE), X),
					      avPosition.y + getProjection(goalStates2f.speed, (int)(newHeading * ANGULARSCALE), Y) };
	double newDistance = getDisPointToPoint(newPosition.x, newPosition.y, obsPosition.x, obsPosition.y);
#ifdef DEBUG
	printf("The original new heading: %f, original distance: %f, new distance: %f\n", newHeading, distance, newDistance);
#endif // DEBUG
	if (newDistance < distance)
	{
		newHeading = currentHeading;
		newPosition = { avPosition.x + getProjection(goalStates2f.speed, (int)(newHeading * ANGULARSCALE), X),
						  avPosition.y + getProjection(goalStates2f.speed, (int)(newHeading * ANGULARSCALE), Y) };
		newDistance = getDisPointToPoint(newPosition.x, newPosition.y, obsPosition.x, obsPosition.y);
#ifdef DEBUG
		printf("The turned new heading: %f\n", newHeading);
#endif // DEBUG
		while (newDistance < distance)
		{
			newHeading = newHeading - M_PI / 2;
			if (newHeading < -M_PI)
			{
				newHeading = 2 * M_PI + newHeading;
			}
			newPosition = { avPosition.x + getProjection(goalStates2f.speed, (int)(newHeading * ANGULARSCALE), X),
							  avPosition.y + getProjection(goalStates2f.speed, (int)(newHeading * ANGULARSCALE), Y) };
			newDistance = getDisPointToPoint(newPosition.x, newPosition.y, obsPosition.x, obsPosition.y);
#ifdef DEBUG
			printf("The turned new heading: %f\n", newHeading);
#endif // DEBUG
		}
	}

	return newHeading;
}

void collisionAvoidance(int totalObsNum, int* appeared, int staticObsNum, Point *staticObsVerdices, int* probe, Point* obsPosition, RotationalState* rotationalObsStates,
	LinearState* linearObsStates, Point* avPosition, RotationalState* rotationalAVStates, LinearState* linearAVStates,
	Point* goalPosition, Point* path, int pathLen, RotationalState* rotationalGoalStates, LinearState* linearGoalStates)
{
	int actualObsNum = 0;
	vector<Point> obsPosition2f;
	vector<VehicleState2f> obsStates2f;

	Point avPositionlocal = { avPosition->x, avPosition->y };
	Point goalPositionlocal = { goalPosition->x, goalPosition->y };
	VehicleState2f avStates2f = { (double)rotationalAVStates->w, ((double)(rotationalAVStates->heading)) / ANGULARSCALE,
			(double)linearAVStates->acc, (double)linearAVStates->speed };
	VehicleState2f goalStates2f = { (double)rotationalGoalStates->w, ((double)(rotationalGoalStates->heading)) / ANGULARSCALE,
				(double)linearGoalStates->acc, (double)linearGoalStates->speed };
	int print = 0;
	double distance = 0.0;
	if (linearObsStates[0].speed != 0)
	{
		distance = getDisPointToPoint(avPositionlocal.x, avPositionlocal.y, obsPosition[0].x, obsPosition[0].y);
		if (distance < 5 * avStates2f.speed)
		{
			print = 1;
		}
	}
#ifdef DEBUG
	if (print == 1)
	{
		printf("------------------------------------------------------------------------------------------------\n");
		printf("Distance:%f\n", distance);
	}
#endif // DEBUG

	for (int i = 0; i < totalObsNum; i++)
	{
#ifdef DEBUG
		//printf("original obs #%d: (%d,%d), w:%d, h:%d, acc:%d, v:%d\n", i, obsPosition[i].x, obsPosition[i].y,
		//	rotationalObsStates[i].w, rotationalObsStates[i].heading, linearObsStates[i].acc, linearObsStates[i].speed);
#endif
		if (appeared[i] == 1)
		{
			obsPosition2f.push_back(obsPosition[i]);
			VehicleState2f temp = { (double)rotationalObsStates[i].w, ((double)(rotationalObsStates[i].heading)) / ANGULARSCALE,
				(double)linearObsStates[i].acc, (double)linearObsStates[i].speed };
			obsStates2f.push_back(temp);
			actualObsNum++;
#ifdef DEBUG
			if (print == 1)
			{
				printf("obs #%d: (%d,%d), w:%f, h:%f, acc:%f, v:%f\n", i, obsPosition2f[i].x, obsPosition2f[i].y,
					obsStates2f[i].w, obsStates2f[i].heading, obsStates2f[i].acc, obsStates2f[i].speed);
			}
#endif
		}
	}
#ifdef DEBUG
	if (print == 1)
	{
		printf("av: (%d,%d), w:%f, h:%f, acc:%f, v:%f\n", avPositionlocal.x, avPositionlocal.y,
			avStates2f.w, avStates2f.heading, avStates2f.acc, avStates2f.speed);
		/*for (int i = 0; i < pathLen; i++)
		{
			printf("node #%d:(%d,%d),", i, path[i].x, path[i].y);
		}
		printf("\n");*/
		printf("goal: w:%d, h:%d, acc:%d, v:%d\n", rotationalGoalStates->w, rotationalGoalStates->heading,
			linearGoalStates->acc, linearGoalStates->speed);
	}
#endif
	collisionAvoidanceDFF(print, actualObsNum, obsPosition2f, obsStates2f, avPositionlocal, avStates2f, goalPositionlocal, path, pathLen, &goalStates2f);
	//collisionAvoidanceNaive(obsNum, obsPosition2f, obsStates2f, avPositionlocal, avStates2f, goalPositionlocal, &goalStates2f);
	/*double newHeading = 0.0;
	if (distance <= 5 * avStates2f.speed)
	{
		newHeading = turn(obsPosition[0], avPosition[0], avStates2f.heading, goalStates2f);
		if (newHeading != goalStates2f.heading)
		{
			probe[0] = 1;
			goalStates2f.heading = newHeading;
		}
	}*/

	setHeading(rotationalAVStates[0], rotationalGoalStates, (int)(goalStates2f.heading * ANGULARSCALE));
	linearGoalStates->acc = (int)goalStates2f.acc;
	linearGoalStates->speed = (int)goalStates2f.speed;
#ifdef DEBUG
	if (print == 1)
	{
		printf("************************************************************************************************\n\n");
	}
#endif
}
