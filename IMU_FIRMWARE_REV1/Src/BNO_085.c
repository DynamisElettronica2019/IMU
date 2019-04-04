#include <BNO_085.h>
#include "math.h"
#include "i2c.h"

/*This function converts a quaternion reading into an euler reading*/
eulerType quaternionToEuler (quaternionType quaternion)
{
	eulerType output;
	float sqi,sqj,sqk;
	float test;

	test = quaternion.qi*quaternion.qj + quaternion.qk*quaternion.qw;

	if (test > THRESHOLD) 
	{
		output.yaw = 2 * atan2(quaternion.qi,quaternion.qw);
		output.pitch = M_PI/2;
		output.roll = 0;
		output.yaw = radToDeg(output.yaw);
		output.pitch = radToDeg(output.pitch);
		output.roll = radToDeg(output.roll);
		return output;
	}

	if (test < -THRESHOLD)
	{
		output.yaw = -2 * atan2(quaternion.qi,quaternion.qw);
		output.pitch = - M_PI/2;
		output.roll = 0;
		output.yaw = radToDeg(output.yaw);
		output.pitch = radToDeg(output.pitch);
		output.roll = radToDeg(output.roll);
		return output;
	}

    sqi = quaternion.qi*quaternion.qi;
    sqj = quaternion.qj*quaternion.qj;
    sqk = quaternion.qk*quaternion.qk;

    output.yaw = atan2(2*quaternion.qj*quaternion.qw-2*quaternion.qi*quaternion.qk , 1 - 2*sqj - 2*sqk);
	output.pitch = asin(2*test);
	output.roll = atan2(2*quaternion.qi*quaternion.qw-2*quaternion.qj*quaternion.qk , 1 - 2*sqi - 2*sqk);
	output.yaw = radToDeg(output.yaw);
	output.pitch = radToDeg(output.pitch);
	output.roll = radToDeg(output.roll);
	

	return output;
}

/*This function converts radians into degrees*/
float radToDeg(float angle)
{
	return ((angle/M_PI)*180.00);
}
