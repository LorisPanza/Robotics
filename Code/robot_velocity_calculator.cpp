#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include "ros/ros.h"
#include <cmath>

//Creation of a class used to calculate the velocity of the robot using 
//the Ticks or RPMs(rad/min) of each single wheels present in the wheel_states topic.

class velocity_sub{
public:

  velocity_sub(){

    //Given values definition and assignment
    //Calibrated parameters (r, Lx, Ly, N)
    this->r = 0.078;  
    this->Lx = 0.1985;  
    this->Ly = 0.169;
    this->T = 5; 
    this->N = 42; 

    this->flag_TICKS = true; //Set it to false to compute the velocity using the RPMs, true to compute it through Ticks

    this->old_nsec;
    this->old_sec;
    this->old_ticks_fl;
    this->old_ticks_fr;
    this->old_ticks_rl;
    this->old_ticks_rr;
    this->counter = 0;

    //Subscriber to "wheel_states" topic and  "cmd_vel" publisher definition

    this->velocity_subscriber = this->n.subscribe("wheel_states",2, &velocity_sub::velocityCalculatorCallback,this);
    this->velocity_publisher = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 2); 

  }

  void run(){

    ros::spin();

  }

  //Callback function for the "wheel_states" subscriber to compute v and ⍵
  // by using Ticks or RPM depending on the flag_TICKS value

  void velocityCalculatorCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    
    // We use the first message of the wheel_states topic to set the initial time 
    // and the starting tick values
    if(this->counter == 0){

        this->old_sec = msg->header.stamp.sec;
        this->old_nsec = msg->header.stamp.nsec;
        this->old_ticks_rr= msg->position[0];
        this->old_ticks_rl=msg->position[1];
        this->old_ticks_fr=msg->position[2];
        this->old_ticks_fl=msg->position[3];
        this->counter = 1;
        return;        

      }


    float radpm_fl = msg->velocity[0];
    float radpm_fr = msg->velocity[1];
    float radpm_rl = msg->velocity[2];
    float radpm_rr = msg->velocity[3];

    // the angular speed in rad/s of each wheel is calculated as Rad_Min/(60*T)
    double w_fl = radpm_fl/300;
    double w_fr = radpm_fr/300;
    double w_rl = radpm_rl/300;
    double w_rr = radpm_rr/300;


    
    // kinematics of the considered robot
    double Vx= (w_fl+w_fr+w_rl+w_rr)*(this->r/4);
    double Vy= (-w_fl+w_fr+w_rl-w_rr)*(this->r/4);
    double Wz= (-w_fl+w_fr-w_rl+w_rr)*(this->r/(4*(this->Lx+this->Ly)));


    
    // we keep track of the time differencce between the current and previous message detected from the wheel_states topic
    double new_nsec= msg->header.stamp.nsec;
    double conv_old_nsec = (this->old_nsec/1000000000);
    double conv_new_nsec = (new_nsec/1000000000);
    double new_sec = msg->header.stamp.sec;
    double new_time = new_sec + conv_new_nsec;
    double old_time = this->old_sec + conv_old_nsec;
    double dtime =(new_time - old_time);



    float new_ticks_fl= msg->position[0];
    float new_ticks_fr= msg->position[1];
    float new_ticks_rl= msg->position[2];
    float new_ticks_rr= msg->position[3];

    //Difference between the current and previous tick for each wheel 
    float dticks_fl= new_ticks_fl - this->old_ticks_fl;
    float dticks_fr= new_ticks_fr - this->old_ticks_fr;
    float dticks_rl= new_ticks_rl - this->old_ticks_rl;
    float dticks_rr= new_ticks_rr - this->old_ticks_rr;


    // the angular speed in rad/s of each wheel is calculated as (dtick/dtime)*(2*pi/(N*T))
    double w_ticks_fl= (dticks_fl/dtime) * ((2*M_PI)/(this->N * this->T));
    double w_ticks_fr= (dticks_fr/dtime) * ((2*M_PI)/(this->N * this->T));
    double w_ticks_rl= (dticks_rl/dtime)* ((2*M_PI)/(this->N * this->T));
    double w_ticks_rr= (dticks_rr/dtime) * ((2*M_PI)/(this->N * this->T));



    double Vx_ticks= (w_ticks_fl+w_ticks_fr+w_ticks_rl+w_ticks_rr)*(this->r/4);
    double Vy_ticks= (-w_ticks_fl+w_ticks_fr+w_ticks_rl-w_ticks_rr)*(this->r/4);
    double Wz_ticks= (-w_ticks_fl+w_ticks_fr-w_ticks_rl+w_ticks_rr)*(this->r/(4*(Lx+Ly)));

     
      
      
    if(this->flag_TICKS == true){
      
      geometry_msgs::TwistStamped geo_msg_ticks;
      geo_msg_ticks.header = msg->header;
      geo_msg_ticks.twist.linear.x = Vx_ticks;
      geo_msg_ticks.twist.linear.y = Vy_ticks;
      geo_msg_ticks.twist.linear.z = 0;

      geo_msg_ticks.twist.angular.x = 0;
      geo_msg_ticks.twist.angular.y = 0;
      geo_msg_ticks.twist.angular.z = Wz_ticks;

      this->velocity_publisher.publish(geo_msg_ticks);

    } else {
      geometry_msgs::TwistStamped geo_msg_rpm;
      geo_msg_rpm.header = msg->header;
      geo_msg_rpm.twist.linear.x = Vx;
      geo_msg_rpm.twist.linear.y = Vy;
      geo_msg_rpm.twist.linear.z = 0;

      geo_msg_rpm.twist.angular.x = 0;
      geo_msg_rpm.twist.angular.y = 0;
      geo_msg_rpm.twist.angular.z = Wz;
      this->velocity_publisher.publish(geo_msg_rpm);
    
    }
  
    // update of the state variables
    this->old_nsec=new_nsec;
    this->old_sec = new_sec;
    this->old_ticks_rr=new_ticks_rr;
    this->old_ticks_rl=new_ticks_rl;
    this->old_ticks_fr=new_ticks_fr;
    this->old_ticks_fl=new_ticks_fl;
  }

private:

  
  ros::NodeHandle n;
  ros::Subscriber velocity_subscriber;
  double old_sec;
  double old_nsec;
  float old_ticks_fl;     
  float old_ticks_fr;
  float old_ticks_rl;
  float old_ticks_rr;
  float r;
  float Lx;
  float Ly;
  int T;
  int N;
  int counter;
  bool flag_TICKS;

  ros::Publisher velocity_publisher;
  

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "velocity_calculator");
  
  velocity_sub my_velocity_sub;
  my_velocity_sub.run();

  return 0;
}
