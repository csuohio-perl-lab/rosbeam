#include <ros/ros.h>
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

		pubOdom = node.advertise<nav_msgs::Odometry>("/odom", 50);
		pubTf = node.advertise<tf2_msgs::TFMessage>("/tf", 50);

		odomThread = boost::thread(&bridge_node::process_odometry, this);

		return true;
	}

	void process_odometry() {
		bool first_time = true;
		struct timespec stat_time;
		struct drive_status stat;

		double x = 0.0;
		double y = 0.0;
  		double th = 0.0;
  		double vx;
  		double vy = 0.0;
  		double vth;
		
		double lastTimeStamp, TimeStamp;
		bool firstTimeStamp = true; 

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

			ros::Time current_time = ros::Time::now();

			TimeStamp = stat.timeStamp;			
			if(firstTimeStamp){
				firstTimeStamp = false;
				lastTimeStamp = TimeStamp;
				continue;			
			}	
	
			if((stat.actualLinearVelocity < 150 && stat.actualLinearVelocity > -150) || (stat.actualAngularVelocity < 150 && stat.actualAngularVelocity > -150))
			{
				stat.actualLinearVelocity = 0;
				stat.actualAngularVelocity = 0;
			} 		

			vx = stat.actualLinearVelocity / 65536.0;
			vth = stat.actualAngularVelocity / 65536.0;		
 
			double dt = (TimeStamp - lastTimeStamp)/ 1000000;
			double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    			double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    			double delta_th = vth * dt;
		
			x += delta_x;
    			y += delta_y;
    			th += delta_th;

			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

			geometry_msgs::TransformStamped trans;
			trans.header.stamp = current_time;
   			trans.header.frame_id = "/odom";
    			trans.child_frame_id = "/base_link";

    			trans.transform.translation.x = x;
    			trans.transform.translation.y = y;
    			trans.transform.translation.z = 0.0;
    			trans.transform.rotation = odom_quat;

    			//send the transform
			tf2_msgs::TFMessage tf2msg;
			tf2msg.transforms.push_back(trans);
			pubTf.publish(tf2msg);

    			//next, we'll publish the odometry message over ROS
    			nav_msgs::Odometry odom;
    			odom.header.stamp = current_time;
    			odom.header.frame_id = "/odom";

    			//set the position
    			odom.pose.pose.position.x = x;
    			odom.pose.pose.position.y = y;
    			odom.pose.pose.position.z = 0.0;
    			odom.pose.pose.orientation = odom_quat;

    			//set the velocity
    			odom.child_frame_id = "/base_link";
    			odom.twist.twist.linear.x = vx;
    			odom.twist.twist.linear.y = 0;
    			odom.twist.twist.angular.z = vth;

    			//publish the message
    			pubOdom.publish(odom);

			lastTimeStamp = TimeStamp;

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
