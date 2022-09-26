#include "ros/publisher.h"
#include <opencv2/core/mat.hpp>
#include <ros/ros.h>
#include <string>
#include <random>
#include <cmath>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
using string = std::string;

double gyro_n, gyro_bias_n, accel_n, accel_bias_n, img_noise;
string img0_topic, img1_topic, imu_topic;
ros::Publisher imgPub0, imgPub1, imuPub;
cv::RNG rng0((unsigned)time(NULL));
cv::RNG rng1((unsigned)time(NULL));
cv_bridge::CvImage cvimg;

ros::Time last_stamp(0);
double gyro_bias_x = 0,
                gyro_bias_y = 0,
                gyro_bias_z = 0,
                accel_bias_x = 0,
                accel_bias_y = 0,
                accel_bias_z = 0;
std::random_device rdev;
std::mt19937 mt(rdev());
std::normal_distribution<double> g(0.0, 1.0);

void image_cb0(const sensor_msgs::ImageConstPtr& img) {
    auto m = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv::Mat noise = m->image.clone();
    noise.convertTo(noise, CV_8UC3);
    rng0.fill(noise, cv::RNG::NORMAL, 0, img_noise);
    m->image += noise;
    sensor_msgs::Image img_noisy;
    m->image.convertTo(m->image, CV_8UC3);
    m->toImageMsg(img_noisy);
    img_noisy.header = img->header;
    img_noisy.height = img->height;
    img_noisy.encoding = img->encoding;
    img_noisy.width = img->width;
    imgPub0.publish(img_noisy);
}

void image_cb1(const sensor_msgs::ImageConstPtr& img) {
    auto m = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv::Mat noise = m->image.clone();
    noise.convertTo(noise, CV_8UC3);
    rng1.fill(noise, cv::RNG::NORMAL, 0, img_noise);
    m->image += noise;
    sensor_msgs::Image img_noisy;
    m->image.convertTo(m->image, CV_8UC3);
    m->toImageMsg(img_noisy);
    img_noisy.header = img->header;
    img_noisy.height = img->height;
    img_noisy.encoding = img->encoding;
    img_noisy.width = img->width;
    imgPub1.publish(img_noisy);
}

void imu_cb(const sensor_msgs::ImuConstPtr& imu) {
    if (last_stamp.toSec() == 0) {
        last_stamp = imu->header.stamp;
        imuPub.publish(*imu.get());
    }
    else {
        sensor_msgs::Imu im = *imu.get();
        double dt = imu->header.stamp.toSec() - last_stamp.toSec();
        if (dt > 0.0001) {
            last_stamp = imu->header.stamp;
            double gyro_nt = gyro_n / sqrt(dt);
            double gyro_rw = gyro_bias_n * sqrt(dt);
            double accel_nt = accel_n / sqrt(dt);
            double accel_rw = accel_bias_n * sqrt(dt);
            im.angular_velocity.x += (gyro_bias_x + gyro_nt*g(mt));
            im.angular_velocity.y += (gyro_bias_y + gyro_nt*g(mt));
            im.angular_velocity.z += (gyro_bias_z + gyro_nt*g(mt));
            gyro_bias_x += gyro_rw * g(mt);
            gyro_bias_y += gyro_rw * g(mt);
            gyro_bias_z += gyro_rw * g(mt);
            im.linear_acceleration.x += (accel_bias_x + accel_nt * g(mt));
            im.linear_acceleration.y += (accel_bias_y + accel_nt * g(mt));
            im.linear_acceleration.z += (accel_bias_z + accel_nt * g(mt));
            accel_bias_x += accel_rw * g(mt);
            accel_bias_y += accel_rw * g(mt);
            accel_bias_z += accel_rw * g(mt);
        } else {
            im.linear_acceleration.x += (accel_bias_x);
            im.linear_acceleration.y += (accel_bias_y);
            im.linear_acceleration.z += (accel_bias_z);
            im.angular_velocity.x += (gyro_bias_x);
            im.angular_velocity.y += (gyro_bias_y);
            im.angular_velocity.z += (gyro_bias_z);
        }
        imuPub.publish(im);
    }

}

int main(int argc, char** argv) {
    string postfix = "_noisy";
    ros::init(argc, argv, "noise_adder_node");
    ros::NodeHandle nh("~");
    
    ros::param::get("~gyroscope_noise_density", gyro_n);
    ros::param::get("~gyroscope_random_walk", gyro_bias_n);
    ros::param::get("~accelerometer_noise_density", accel_n);
    ros::param::get("~accelerometer_random_walk", accel_bias_n);
    ros::param::get("~img_noise_density", img_noise);

    ros::param::get("~img0_topic", img0_topic);
    ros::param::get("~img1_topic", img1_topic);
    ros::param::get("~imu_topic", imu_topic);
    ROS_INFO("subscribing to:%s", img0_topic.c_str());

    auto imgSub0 = nh.subscribe(img0_topic, 50, &image_cb0);
    auto imgSub1 = nh.subscribe(img1_topic, 50, &image_cb1);
    auto imuSub = nh.subscribe(imu_topic, 50, &imu_cb);

    imgPub0 = nh.advertise<sensor_msgs::Image>(img0_topic + postfix, 50);
    imgPub1 = nh.advertise<sensor_msgs::Image>(img1_topic + postfix, 50);
    imuPub = nh.advertise<sensor_msgs::Imu>(imu_topic + postfix, 50);
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}
