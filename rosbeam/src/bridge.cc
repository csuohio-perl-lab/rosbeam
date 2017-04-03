#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h> 
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>


#include "drive_command.h"

//Distance between centers of two wheels: 10.27 inch == 0.261 meters 
static int timespec_eq(const struct timespec *a, const struct timespec *b)
{
	return a->tv_sec == b->tv_sec && a->tv_nsec == b->tv_nsec;
}

class bridge_node {
private:
	ros::NodeHandle node;
	ros::Publisher pubOdom, pubTf;
	ros::Subscriber subVel;
	boost::thread odomThread;

	//Debug
	ros::Publisher pubEncoder1, pubEncoder2, pubRawEncoder1, pubRawEncoder2, pubAcc1, pubAcc2, pubAcc3, pubspeedr, pubspeedl, pubActualVel, pubYaw, pubActualAng;

	struct drive_shm *shm;
public:
	bool start() {
		
		int fd = shm_open(DRIVE_SHM_NAME, O_RDWR, 0);

		if (fd < 0) {
			ROS_ERROR("shm_open: %s", strerror(errno));
			return false;
		}

		shm = (struct drive_shm *)mmap(NULL, sizeof(*shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
		if (close(fd) < 0)
			ROS_WARN("close: %s", strerror(errno));

		if (shm == MAP_FAILED) {
			ROS_ERROR("mmap: %s", strerror(errno));
			return false;
		}
		
		subVel = node.subscribe("/cmd_vel", 2, &bridge_node::sub_vel, this);

		odomThread = boost::thread(&bridge_node::process_odometry, this);
		
		pubEncoder1 = node.advertise<std_msgs::Int16>("lwheel", 30);
		pubEncoder2 = node.advertise<std_msgs::Int16>("rwheel", 30);
		//Debug
		pubRawEncoder1 = node.advertise<std_msgs::Int16>("lwheel_raw", 30);
		pubRawEncoder2 = node.advertise<std_msgs::Int16>("rwheel_raw", 30);
		pubAcc1 = node.advertise<std_msgs::Int32>("acc1", 30);
		pubAcc2 = node.advertise<std_msgs::Int32>("acc2", 30);
		pubAcc3 = node.advertise<std_msgs::Int32>("acc3", 30);
		pubspeedr = node.advertise<std_msgs::Int32>("speedr", 30);
		pubspeedl = node.advertise<std_msgs::Int32>("speedl", 30);
		pubActualVel = node.advertise<std_msgs::Int32>("actuallinvel", 30);	
		pubActualAng = node.advertise<std_msgs::Int32>("actualangvel", 30);	
		pubYaw = node.advertise<std_msgs::Int32>("yaw", 30);	
	
		return true;
	}

	void process_odometry() {
		bool first_time = true;
		struct timespec stat_time;
		struct drive_status stat;
		int last_lwheel, last_rwheel, diff_l, diff_r;
		int  new_lwheel, new_rwheel = 0;
		int est; //encoder speed thresold
		int yaw_deg;
		ros::Time current_time, last_time;
		current_time = ros::Time::now();
		last_time = ros::Time::now();

		bool firstTimeStamp = true; 
		node.param("/encoder_speed_threshold", est, 60);
		
		ros::Rate r(30);
		while (ros::ok()) {
			pthread_mutex_lock(&shm->stat_lock);
			if (first_time) {
				first_time = false;
				stat_time = shm->stat_time;
			}
			while (timespec_eq(&stat_time, &shm->stat_time)) {
				pthread_cond_wait(&shm->stat_cond, &shm->stat_lock);
			}
			stat_time = shm->stat_time;
			stat = shm->stat;
			pthread_mutex_unlock(&shm->stat_lock);

			current_time = ros::Time::now();

			//TimeStamp = stat.timeStamp;			
			if(firstTimeStamp){
				firstTimeStamp = false;
				last_time = current_time;
				last_lwheel = stat.encoder_1;
				last_rwheel = stat.encoder_2;
				new_lwheel = stat.encoder_1;
				new_rwheel = stat.encoder_2;
				continue;			
			}	

			diff_l = stat.encoder_1 - last_lwheel;
			diff_r = stat.encoder_2 - last_rwheel;

			if (! ( (stat.speed_1 < -est) || (stat.speed_1 > est) || (stat.speed_2 < -est) || (stat.speed_2 > est) ) ){
				new_lwheel += diff_l;
				new_rwheel += diff_r;
		        }
	
            		pubEncoder1.publish(new_lwheel);
            		pubEncoder2.publish(-new_rwheel);
			
			last_lwheel = stat.encoder_1;
			last_rwheel = stat.encoder_2;

			//Debug
			yaw_deg = stat.integratedYaw / 65536;
			pubRawEncoder1.publish(stat.encoder_1);
			pubRawEncoder2.publish(-stat.encoder_2);
			pubAcc1.publish(stat.accel_1);
			pubAcc2.publish(stat.accel_2);
			pubAcc3.publish(stat.accel_3);
			pubspeedl.publish(stat.speed_1);
			pubspeedr.publish(stat.speed_2);
			pubActualVel.publish(stat.actualLinearVelocity);
			pubActualAng.publish(stat.actualAngularVelocity);
			pubYaw.publish(yaw_deg);
		}
	}

	void join() {
		odomThread.join();
	}

	void sub_vel(const geometry_msgs::Twist::ConstPtr& msg) {
		double lin_vel = msg->linear.x;
		double ang_vel = msg->angular.z;			

		if (lin_vel > 0.5 || ang_vel > 0.5) {
			ROS_WARN("velocity too large, lin:%f ang:%f", lin_vel, ang_vel);
			//return;
		}

		struct drive_command cmd = {DRIVE_COMMAND_MAGIC, 1,
			lin_vel*65536, ang_vel*65536, 0,
			-1, -1};

		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);

		now.tv_nsec += 150000000;
		if (now.tv_nsec >= 1000000000) {
			now.tv_sec++;
			now.tv_nsec -= 1000000000;
		}

		pthread_mutex_lock(&shm->cmd_lock);
		shm->cmd_time = now;
		shm->cmd = cmd;
		pthread_mutex_unlock(&shm->cmd_lock);
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "beam");

	bridge_node bridge;

	if (!bridge.start())
		return 1;

	ROS_INFO("start");
	ros::spin();

	bridge.join();

	return 0;
}
