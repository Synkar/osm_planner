///
// Created by controller on 11/13/18.
//

#include "osm_planner/coordinates_converters/wgs_84_elipsoid.h"
#include <memory>

namespace osm_planner {

    namespace coordinates_converters {

        WGS84Elipsoid::WGS84Elipsoid() : CoordinatesConverterBase(),tf_listener_(tf_buffer_) {}

        WGS84Elipsoid::WGS84Elipsoid(std::string map_frame, std::string earth_frame, GeoNode ltp_origin) : CoordinatesConverterBase(map_frame, earth_frame, ltp_origin),tf_listener_(tf_buffer_) {
            navsat_conversions_ = std::make_shared<synkar_navsat::NavSatConversions>(
                map_frame,
                earth_frame,
                synkar_navsat::WGS84Utils::geodetic{ltp_origin.latitude, ltp_origin.longitude, ltp_origin.altitude}
            );
        }

        double WGS84Elipsoid::getDistance(double latitude1, double longitude1, double latitude2, double longitude2) {

            geometry_msgs::Point p1 = getGeoPoint(latitude1, longitude1);
            geometry_msgs::Point p2 = getGeoPoint(latitude2, longitude2);
            return sqrt(pow(p1.x - p2.x, 2.0) + pow(p1.y - p2.y, 2.0));
        }

        double WGS84Elipsoid::getBearing(double latitude1, double longitude1, double latitude2, double longitude2) {

            geometry_msgs::Point p1 = getGeoPoint(latitude1, longitude1);
            geometry_msgs::Point p2 = getGeoPoint(latitude2, longitude2);
            
            return atan2( p1.y - p2.y, p1.x - p2.x);
        }

        geometry_msgs::Point WGS84Elipsoid::getGeoPoint(double latitude, double longitude){

            geometry_msgs::Point point;
            double N = a/(sqrt(1 - pow(e, 2.0)*pow(sin(latitude), 2.0)));

            point.x = (N + 0) * cos(latitude) * cos(longitude);
            point.y = (N + 0) * cos(latitude) * sin(longitude);
            point.z = (N * (1 - pow(e, 2.0) ) + 0) * sin(latitude);

            return point;
        }


        geometry_msgs::Point WGS84Elipsoid::convertLatLngToMapXY(double latitude, double longitude){
            geometry_msgs::Point point;
            double x, y;
            navsat_conversions_->convertLatLngToMapXY(latitude, longitude, x, y);
            ROS_DEBUG("Converted latitude %f, longitude %f to map XY %f, %f", latitude, longitude, x, y);
            point.x = x;
            point.y = y;
            point.z = 0;

            return point;
        }

        void WGS84Elipsoid::getEarth2Map(){
            try
            {
                 tf2::convert(tf_buffer_.lookupTransform("map", "earth", ros::Time(0)).transform, earth_to_map);
                 ROS_DEBUG("Earth to map transform: translation (%f, %f, %f) rotation (%f, %f, %f, %f)", earth_to_map.getOrigin().getX(), earth_to_map.getOrigin().getY(), earth_to_map.getOrigin().getZ(), 
                                        earth_to_map.getRotation().getX(), earth_to_map.getRotation().getY(), earth_to_map.getRotation().getZ(), earth_to_map.getRotation().getW());
                                        has_earth2map_ = true;
            }
            catch (tf2::TransformException &ex)
            {
                 ROS_WARN( "Could not locate static transforms with exception ");
                 has_earth2map_ = false;
            }
        }
    }
}