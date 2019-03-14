#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <filter.hpp>

class imu_compensate
{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub_imu;
  ros::Publisher pub_imu;

  std::shared_ptr<filter> facc[3];
  double gyro_zero[3];
  int cnt;

  void cb_imu(const sensor_msgs::Imu::Ptr& msg)
  {
    auto imu = *msg;
    double acc = sqrt(pow(imu.linear_acceleration.x, 2.0) + pow(imu.linear_acceleration.y, 2.0) + pow(imu.linear_acceleration.z, 2.0));

    cnt++;
    if (cnt == 2000)
    {
      ROS_ERROR("IMU zero calibrated");
    }
    else if (cnt < 2000)
    {
      double k = 0.01;
      gyro_zero[0] = gyro_zero[0] * (1.0 - k) + imu.angular_velocity.x * k;
      gyro_zero[1] = gyro_zero[1] * (1.0 - k) + imu.angular_velocity.y * k;
      gyro_zero[2] = gyro_zero[2] * (1.0 - k) + imu.angular_velocity.z * k;
    }
    imu.angular_velocity.x -= gyro_zero[0];
    imu.angular_velocity.y -= gyro_zero[1];
    imu.angular_velocity.z -= gyro_zero[2];
    imu.linear_acceleration.x = facc[0]->in(imu.linear_acceleration.x);
    imu.linear_acceleration.y = facc[1]->in(imu.linear_acceleration.y);
    imu.linear_acceleration.z = facc[2]->in(imu.linear_acceleration.z);
    pub_imu.publish(imu);
  }

public:
  imu_compensate()
    : nh("~")
  {
    sub_imu = nh.subscribe("imu_raw", 1, &imu_compensate::cb_imu, this);
    pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 2);
    cnt = 0;
    facc[0].reset(new filter(filter::FILTER_LPF, 10.0, 0.0));
    facc[1].reset(new filter(filter::FILTER_LPF, 10.0, 0.0));
    facc[2].reset(new filter(filter::FILTER_LPF, 10.0, 0.0));
    gyro_zero[0] = gyro_zero[1] = gyro_zero[2] = 0;
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "imu_compensate");

  imu_compensate imu;

  ros::spin();

  return 0;
}
