//
// Created by controller on 11/12/18.
//

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <synkar_navsat/navsat_conversions.h>

#ifndef PROJECT_COORDINATES_CONVERTER_BASE_H
#define PROJECT_COORDINATES_CONVERTER_BASE_H

namespace osm_planner {

    namespace coordinates_converters {

           struct GeoNode {
                double latitude;
                double longitude;
                double altitude;
                double angle;
            };

        class CoordinatesConverterBase {

        public:

            CoordinatesConverterBase() : offset_(OFFSET) {}

            CoordinatesConverterBase(std::string map_frame, std::string earth_frame, GeoNode ltp_origin) : map_frame_(map_frame), earth_frame_(earth_frame), ltp_origin_(ltp_origin) {}

            virtual double getDistance(double latitude1, double longitude1, double latitude2, double longitude2) = 0;

            virtual double getBearing(double latitude1, double longitude1, double latitude2, double longitude2) = 0;

            virtual geometry_msgs::Point convertLatLngToMapXY(double latitude, double longitude) = 0;

            void setOrigin(double latitude, double longitude) {
                origin_.latitude = latitude;
                origin_.longitude = longitude;
            }

            void setOffset(double offset) {
                this->offset_ = offset;
            }

            template<class N1>
            void setOrigin(N1 node) {
                setOrigin(node.latitude, node.longitude);
            }

            template<class N1, class N2>
            double getDistance(N1 node1, N2 node2) {
                geometry_msgs::Point p1 = convertLatLngToMapXY(node1.latitude, node1.longitude);
                geometry_msgs::Point p2 = convertLatLngToMapXY(node2.latitude, node2.longitude);
                return sqrt(pow(p2.x - p1.x, 2.0) + pow(p2.y - p1.y, 2.0));; 

            };

            template<class N1, class N2>
            double getDistance(N1 node) {
                geometry_msgs::Point p1 = convertLatLngToMapXY(origin_.latitude, origin_.longitude);
                geometry_msgs::Point p2 = convertLatLngToMapXY(node.latitude, node.longitude);

                 return sqrt(pow(p2.x - p1.x, 2.0) + pow(p2.y - p1.y, 2.0));
            };

            template<class N1, class N2>
            double getBearing(N1 node1, N2 node2) {
                geometry_msgs::Point p1 = convertLatLngToMapXY(node1.latitude, node1.longitude);
                geometry_msgs::Point p2 = convertLatLngToMapXY(node2.latitude, node2.longitude);
                return atan2( p1.y - p2.y, p1.x - p2.x);
            };

            template<class N>
            double getBearing(N node) {
                geometry_msgs::Point p1 = convertLatLngToMapXY(origin_.latitude, origin_.longitude);
                geometry_msgs::Point p2 = convertLatLngToMapXY(node.latitude, node.longitude);
                return atan2( p1.y - p2.y, p1.x - p2.x);
            };

            template<class N>
            std::vector<double> getCoordinates(N node) {
                geometry_msgs::Point p2 = convertLatLngToMapXY(node.latitude, node.longitude);
                return {p2.x, p2.y};
            };

            template<class N1, class N2>
            std::vector<double> getCoordinates(N1 node1, N2 node2) {
                geometry_msgs::Point p1 = convertLatLngToMapXY(node1.latitude, node1.longitude);
                geometry_msgs::Point p2 = convertLatLngToMapXY(node2.latitude, node2.longitude);
                return {p2.x, p2.y};
            };

            template<class N>
            double getCoordinateX(N node) {
		        return getCoordinates(node).at(0);
            };

            template<class N1, class N2>
            double getCoordinateX(N1 node1, N2 node2) {
                return getCoordinates(node1, node2).at(0);
            };

            double getCoordinateX(double latitude, double longitude) {
                GeoNode node;
                node.latitude = latitude;
                node.longitude = longitude;
              	return getCoordinateX(node);
            };

            double getCoordinateX(double latitude1, double longitude1, double latitude2, double longitude2) {
                GeoNode node1, node2;
                node1.latitude = latitude1;
                node1.longitude = longitude1;
                node2.latitude = latitude2;
                node2.longitude = longitude2;
                return getCoordinateX(node1, node2);
            };

            template<class N>
            double getCoordinateY(N node) {
               return getCoordinates(node).at(1);
            };

            template<class N1, class N2>
            double getCoordinateY(N1 node1, N2 node2) {
		        return getCoordinates(node1, node2).at(1);
            };

            double getCoordinateY(double latitude, double longitude) {
                GeoNode node;
                node.latitude = latitude;
                node.longitude = longitude;
                return getCoordinateY(node);
            };

            double getCoordinateY(double latitude1, double longitude1, double latitude2, double longitude2) {
                GeoNode node1, node2;
                node1.latitude = latitude1;
                node1.longitude = longitude1;
                node2.latitude = latitude2;
                node2.longitude = longitude2;
                return getCoordinateY(node1, node2);
            };


        protected:
            GeoNode origin_;
            double offset_;
            std::string map_frame_;
            std::string earth_frame_;
            GeoNode ltp_origin_;

        private:

            constexpr static double OFFSET = M_PI / 2.0;
           // double offset_;
        };
    }
}

#endif //PROJECT_COORDINATES_CONVERTER_BASE_H
