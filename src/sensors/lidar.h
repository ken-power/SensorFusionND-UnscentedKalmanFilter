#ifndef LIDAR_H
#define LIDAR_H

#include <ctime>
#include <chrono>

#include "../render/render.h"

const double PI = 3.1415;

struct Ray
{
    Vect3 origin_;
    double resolution_;
    Vect3 direction_;
    Vect3 cast_position_;
    double cast_distance_;

    // parameters:
    // setOrigin: the starting position from where the ray is cast
    // horizontalAngle: the angle of direction the ray travels on the xy plane
    // verticalAngle: the angle of direction between xy plane and ray
    // 				  for example 0 radians is along xy plane and pi/2 radians is stright up
    // resolution: the magnitude of the ray's step, used for ray casting, the smaller the more accurate but the more expensive

    Ray(Vect3 setOrigin, double horizontalAngle, double verticalAngle, double setResolution)
            : origin_(setOrigin), resolution_(setResolution),
              direction_(resolution_ * cos(verticalAngle) * cos(horizontalAngle),
                         resolution_ * cos(verticalAngle) * sin(horizontalAngle),
                         resolution_ * sin(verticalAngle)),
              cast_position_(origin_), cast_distance_(0)
    {}

    void rayCast(const std::vector<Car> & cars,
                 double minDistance,
                 double maxDistance,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                 double slopeAngle,
                 double sderr)
    {
        // reset ray
        cast_position_ = origin_;
        cast_distance_ = 0;

        bool collision = false;

        while(!collision && cast_distance_ < maxDistance &&
              (cast_position_.y <= 6 && cast_position_.y >= -6 && cast_position_.x <= 50 && cast_position_.x >= -15))
        {

            cast_position_ = cast_position_ + direction_;
            cast_distance_ += resolution_;

            // check if there is any collisions with ground slope
            collision = (cast_position_.z <= cast_position_.x * tan(slopeAngle));

            // check if there is any collisions with cars
            if(!collision && cast_distance_ < maxDistance)
            {
                for(Car car : cars)
                {
                    collision |= car.checkCollision(cast_position_);
                    if(collision)
                        break;
                }
            }
        }

        if((cast_distance_ >= minDistance) && (cast_distance_ <= maxDistance) &&
           (cast_position_.y <= 6 && cast_position_.y >= -6 && cast_position_.x <= 50 && cast_position_.x >= -15))
        {
            // add noise based on standard deviation error
            double rx = ((double) rand() / (RAND_MAX));
            double ry = ((double) rand() / (RAND_MAX));
            double rz = ((double) rand() / (RAND_MAX));
            cloud->points.push_back(pcl::PointXYZ(cast_position_.x + rx * sderr,
                                                  cast_position_.y + ry * sderr,
                                                  cast_position_.z + rz * sderr));
        }

    }

};

struct Lidar
{

    std::vector<Ray> rays_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
    std::vector<Car> cars_;
    Vect3 position_;
    double ground_slope_;
    double min_distance_;
    double max_distance_;
    double resoultion_;
    double sderr_;

    Lidar(std::vector<Car> setCars, double setGroundSlope)
            : point_cloud_(new pcl::PointCloud<pcl::PointXYZ>()), position_(0, 0, 3.0)
    {
        // TODO:: set minDistance to 5 to remove points from roof of ego car
        min_distance_ = 5;
        max_distance_ = 120;
        resoultion_ = 0.2;

        // TODO:: set sderr to 0.2 to get more interesting pcd files
        sderr_ = 0.2;
        cars_ = setCars;
        ground_slope_ = setGroundSlope;

        // TODO:: increase number of layers to 8 to get higher resolution pcd
        int num_layers = 64;

        // the steepest vertical angle
        double steepest_angle = 24.8 * (-PI / 180);
        double angle_range = 26.8 * (PI / 180);

        // TODO:: set to pi/64 to get higher resolution pcd
        double horizontal_angle_inc = PI / 2250;

        double angle_increment = angle_range / num_layers;

        for(double angle_vertical = steepest_angle;
            angle_vertical < steepest_angle + angle_range; angle_vertical += angle_increment)
        {
            for(double angle = 0; angle <= 2 * PI; angle += horizontal_angle_inc)
            {
                Ray ray(position_, angle, angle_vertical, resoultion_);
                rays_.push_back(ray);
            }
        }
    }

    ~Lidar()
    {
        // pcl uses boost smart pointers for cloud pointer so we don't have to worry about manually freeing the memory
    }

    void UpdateCars(std::vector<Car> setCars)
    {
        cars_ = setCars;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr Scan()
    {

        point_cloud_->points.clear();
        auto startTime = std::chrono::steady_clock::now();
        for(Ray ray : rays_)
            ray.rayCast(cars_, min_distance_, max_distance_, point_cloud_, ground_slope_, sderr_);
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        cout << "ray casting took " << elapsedTime.count() << " milliseconds" << endl;
        point_cloud_->width = point_cloud_->points.size();
        point_cloud_->height = 1; // one dimensional unorganized point cloud dataset
        return point_cloud_;
    }
};

#endif