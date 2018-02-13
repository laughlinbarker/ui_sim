/*
 * spherical_test.cpp
 *
 *  Created on: Jul 4, 2016
 *      Author: laughlin
 */


#include <gazebo/common/common.hh>

#include <ignition/math/Angle.hh>
#include <ignition/math/Vector3.hh>

int main(){

	const ignition::math::Vector3d far_x(1000, 0, 0);
	const ignition::math::Vector3d far_y(0,1000, 0);

	const ignition::math::Angle lat(0), lon(0), hdg(0);
	double ele = 0;

	gazebo::math::Vector3 lat_long;

	const gazebo::common::SphericalCoordinates::SurfaceType earth = (gazebo::common::SphericalCoordinates::SurfaceType)1;

	gazebo::common::SphericalCoordinates sphericalCords(earth,lat,lon,ele,hdg);

	//assuming ENU coordinates, pos x should be positive longitude
	lat_long = sphericalCords.SphericalFromLocal(far_x);
	std::cout << "Far x: [" << far_x << "] ---> [" << lat_long << "] \n";

	//positive y should be positive latitude in ENU
	lat_long = sphericalCords.SphericalFromLocal(far_y);
	std::cout << "Far y: [" << far_y << "] ---> [" << lat_long << "] \n";
	return 1;
}
