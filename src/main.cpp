#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <random>

void publishSamplePoint(const std::vector<Eigen::Vector3d> sample_bucket, ros::Publisher *publisher, bool rotated);
// void publishSamplePointRotated(const std::vector<Eigen::Vector3d> sample_bucket, ros::Publisher *publisher);
void publishVector(const Eigen::Vector3d &previous_center, const Eigen::Vector3d &guide_point, ros::Publisher *publisher);
geometry_msgs::Point vect2Point(const Eigen::Vector3d &vect);
int vector_id_count = 0;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sampler");
    ros::NodeHandle n("~");
    double x_variance, y_variance;
    int sample_num;

    double previous_center_x, previous_center_y, previous_center_z;
    double guide_x, guide_y, guide_z;

    n.param<double>("x_variance", x_variance, 0.1);
    n.param<int>("sample_number", sample_num, 1000);
    n.param<double>("previous_center_x", previous_center_x, 0.0);
    n.param<double>("previous_center_y", previous_center_y, 0.0);
    n.param<double>("previous_center_z", previous_center_z, 0.0);
    n.param<double>("guide_x", guide_x, 0.0);
    n.param<double>("guide_y", guide_y, 0.0);
    n.param<double>("guide_z", guide_z, 0.0);

    ros::Publisher samplePoint_pub = n.advertise<visualization_msgs::MarkerArray>("/sample_point", 100);
    ros::Publisher samplePoint_rotated_pub = n.advertise<visualization_msgs::MarkerArray>("/sample_point_rotated", 100);
    ros::Publisher vector_pub = n.advertise<visualization_msgs::Marker>("/sample_x_direction", 100);

    int current_sample = 0;

    std::vector<Eigen::Vector3d> sample_bucket, sample_bucket_rotated;
    sample_bucket.reserve(sample_num);
    sample_bucket_rotated.reserve(sample_num);

    Eigen::Vector3d previous_center(previous_center_x, previous_center_y, previous_center_z);
    Eigen::Vector3d guide_point(guide_x, guide_y, guide_z);
    Eigen::Vector3d sample_x_vector = previous_center - guide_point;
    Eigen::Vector3d sample_x_direction = sample_x_vector / sample_x_vector.norm();
    Eigen::Vector3d global_x_direction(1, 0, 0);
    double angle_between = acos(sample_x_direction.dot(global_x_direction));

    Eigen::Vector3d rotation_axis;
    Eigen::Matrix3d rotation_matrix;
    

    if ((0 <= angle_between && angle_between < 0.017) || (0 >= angle_between && angle_between > -0.017))
    {
        rotation_matrix << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    }

    else if ((3.124 <= angle_between && angle_between < 3.159) || (-3.124 >= angle_between && angle_between > -3.159))
    {
        rotation_matrix << -1, 0, 0, 0, -1, 0, 0, 0, 1;
    }

    else
    {
        rotation_axis = global_x_direction.cross(sample_x_direction);
        rotation_axis = rotation_axis / rotation_axis.norm();
        rotation_matrix = Eigen::AngleAxisd(angle_between, rotation_axis);
    }

    std::cout << "angle between is " << angle_between << std::endl;
    std::cout << "rotation axis is " << rotation_axis.transpose() << std::endl;
    std::cout << "sample axis direction is " << sample_x_direction.transpose() << std::endl;
    std::cout << "rotation_matrix " << std::endl
              << rotation_matrix << std::endl;
    std::random_device rd;
    std::mt19937_64 generator;
    generator = std::mt19937_64(rd());

    std::normal_distribution<double> x_normal_rand_;
    std::normal_distribution<double> y_normal_rand_;
    std::normal_distribution<double> z_normal_rand_;

    std::uniform_real_distribution<double> x_uniform_rand_(-1,1);
    std::uniform_real_distribution<double> y_uniform_rand_(-1,1);
    std::uniform_real_distribution<double> z_uniform_rand_(-1,1);
    x_variance = sample_x_vector.norm() / 3;
    y_variance = 5 * x_variance;
    double x_sd = sqrt(x_variance);
    double y_sd = sqrt(y_variance);
    double z_sd = y_sd;

    std::cout << "sample_num is " << sample_num << std::endl;
    std::cout << "x_variance is " << x_variance << std::endl;
    std::cout << "y_variance is " << y_variance << std::endl;
    std::cout << "x standard dev is " << x_sd << std::endl;
    std::cout << "y standard dev is " << y_sd << std::endl;
    std::cout << "z standard dev is " << z_sd << std::endl;

    x_normal_rand_ = std::normal_distribution<double>(0, x_sd);
    y_normal_rand_ = std::normal_distribution<double>(0, y_sd);
    z_normal_rand_ = std::normal_distribution<double>(0, z_sd);

    Eigen::Vector3d origin(0.0, 0.0, 0.0);
    Eigen::Vector3d x_offset(sample_x_vector.norm(), 0.0, 0.0);
    Eigen::Vector3d y_offset(0.0, 2 * sample_x_vector.norm(), 0.0);

    while (current_sample < sample_num)
    {
        current_sample++;
        // Eigen::Vector3d sample_point(x_normal_rand_(generator), y_normal_rand_(generator), z_normal_rand_(generator));
         Eigen::Vector3d sample_point(x_uniform_rand_(generator), y_uniform_rand_(generator), z_uniform_rand_(generator));
        sample_bucket.emplace_back(sample_point);

        Eigen::Vector3d rotated_sample = rotation_matrix * sample_point;
        rotated_sample = rotated_sample + guide_point;
        sample_bucket_rotated.emplace_back(rotated_sample);
    }

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        publishSamplePoint(sample_bucket, &samplePoint_pub, false);
        publishSamplePoint(sample_bucket_rotated, &samplePoint_rotated_pub, false);
        publishVector(previous_center, guide_point, &vector_pub);
        publishVector(origin, x_offset, &vector_pub);
        publishVector(origin, y_offset, &vector_pub);
        // publishSamplePoint(sample_bucket_rotated, &samplePo#include <corridor_gen.h>int_rotated_pub);

        ros::spinOnce(); // not necessary if there is no callback
        loop_rate.sleep();
    }

    return 0;
}

void publishSamplePoint(const std::vector<Eigen::Vector3d> sample_bucket, ros::Publisher *publisher, bool rotated)
{
    visualization_msgs::MarkerArray samples;
    samples.markers.clear();
    samples.markers.reserve(sample_bucket.size());

    for (int i = 0; i < sample_bucket.size(); i++)
    {
        visualization_msgs::Marker node;
        node.header.frame_id = "map";
        node.header.stamp = ros::Time::now();
        node.id = i;
        node.type = visualization_msgs::Marker::SPHERE;
        node.action = visualization_msgs::Marker::ADD;
        node.pose.position.x = sample_bucket[i].x();
        node.pose.position.y = sample_bucket[i].y();
        node.pose.position.z = sample_bucket[i].z();
        node.pose.orientation.x = 0.0;
        node.pose.orientation.y = 0.0;
        node.pose.orientation.z = 0.0;
        node.pose.orientation.w = 1.0;
        node.scale.x = 0.05;
        node.scale.y = 0.05;
        node.scale.z = 0.05;
        node.color.a = 1.0; // Don't forget to set the alpha!

        if (rotated)
        {
            node.color.r = 1.0;
            node.color.g = 0.0;
            node.color.b = 0.0;
        }
        else
        {
            node.color.r = 0.0;
            node.color.g = 1.0;
            node.color.b = 0.0;
        }

        samples.markers.emplace_back(node);
    }

    publisher->publish(samples);
}

void publishVector(const Eigen::Vector3d &previous_center, const Eigen::Vector3d &guide_point, ros::Publisher *publisher)
{
    vector_id_count++;
    geometry_msgs::Point previous = vect2Point(previous_center);
    geometry_msgs::Point guide = vect2Point(guide_point);

    visualization_msgs::Marker vector;
    vector.header.frame_id = "map";
    vector.header.stamp = ros::Time::now();
    vector.id = vector_id_count;
    vector.type = visualization_msgs::Marker::LINE_STRIP;
    vector.action = visualization_msgs::Marker::ADD;
    vector.scale.x = 0.08;
    vector.color.a = 1.0; // Don't forget to set the alpha!
    vector.color.r = 1.0;
    vector.points.push_back(previous);
    vector.points.push_back(guide);
    publisher->publish(vector);
}

geometry_msgs::Point vect2Point(const Eigen::Vector3d &vect)
{
    geometry_msgs::Point point;
    point.x = vect.x();
    point.y = vect.y();
    point.z = vect.z();

    return point;
}