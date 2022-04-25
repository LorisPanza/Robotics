#include "std_msgs/String.h"
#include "pub_sub/Num.h"
#include <sensor_msgs/JointState.h>
#include "ros/ros.h"
//#include "math.h"


//void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
//}

/*
int old_sec=0;
int old_nsec=0;
float old_ticks_fl=0;
float old_ticks_fr=0;
float old_ticks_rl=0;
float old_ticks_rr=0;
*/

class velocity_sub{
public:

  velocity_sub(){

    this->r = 0.07;
    this->Lx = 0.2;
    this->Ly = 0.169;
    this->T = 5;
    this->N = 42;
    this->old_nsec = 0;
    this->old_ticks_fl = 0;
    this->old_ticks_fr = 0;
    this->old_ticks_rl = 0;
    this->old_ticks_rr = 0;

    this->velocity_subscriber = this->n.subscribe("wheel_states", 1000, &velocity_sub::counterCallback,this);
    this->velocity_publisher = this->n.advertise<std_msgs::String>("cmd_vel", 1000);

  }

  void run(){

    ros::spin();


  }


  void counterCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    //ROS_INFO("I counted: [%f],[%f], [%f],[%f]", msg->velocity[0], msg->velocity[1],msg->velocity[2], msg->velocity[3]);
    //const sensor_msgs::JointState::ConstPtr& msg

    float radpm_fl = msg->velocity[0];
    float radpm_fr = msg->velocity[1];
    float radpm_rl = msg->velocity[2];
    float radpm_rr = msg->velocity[3];

    float w_fl = radpm_fl/60;
    float w_fr = radpm_fr/60;
    float w_rl = radpm_rl/60;
    float w_rr = radpm_rr/60;

    //ROS_INFO("I computed: [%f],[%f], [%f],[%f]", w_fl,w_fr,w_rl,w_rr);

    

    float Vx= (w_fl+w_fr+w_rl+w_rr)*(this->r/4);
    float Vy= (-w_fl+w_fr+w_rl-w_rr)*(this->r/4);
    float Wz= (-w_fl+w_fr-w_rl+w_rr)*(this->r/(4*(this->Lx+this->Ly)));



    ROS_INFO("I computed: [%f],[%f], [%f]", Vx,Vy,Wz);


    //ROS_INFO("I counted: [%d]", msg->header.stamp.nsec);


    
      double new_nsec= msg->header.stamp.nsec;
      double dtime =(new_nsec - this->old_nsec)/1000000000;
      float new_ticks_fl= msg->position[0];
      float new_ticks_fr= msg->position[1];
      float new_ticks_rl= msg->position[2];
      float new_ticks_rr= msg->position[3];
      float dticks_fl= new_ticks_fl - this->old_ticks_fl;
      float dticks_fr= new_ticks_fr - this->old_ticks_fr;
      float dticks_rl= new_ticks_rl - this->old_ticks_rl;
      float dticks_rr= new_ticks_rr - this->old_ticks_rr;

      float w_ticks_fl= (dticks_fl/dtime) * (6.28/(this->N * this->T));
      float w_ticks_fr= (dticks_fr/dtime) * (6.28/(this->N * this->T));
      float w_ticks_rl= (dticks_rl/dtime)* (6.28/(this->N * this->T));
      float w_ticks_rr= (dticks_rr/dtime) * (6.28/(this->N * this->T));

      float Vx_ticks= (w_ticks_fl+w_ticks_fr+w_ticks_rl+w_ticks_rr)*(this->r/4);
      float Vy_ticks= (-w_ticks_fl+w_ticks_fr+w_ticks_rl-w_ticks_rr)*(this->r/4);
      float Wz_ticks= (-w_ticks_fl+w_ticks_fr-w_ticks_rl+w_ticks_rr)*(this->r/(4*(Lx+Ly)));

      ROS_INFO("d_ticks_fl_1: [%f]",dticks_fl);
      ROS_INFO("d_time: [%g]",dtime);


      std_msgs::String string;
      string.data = "Ciao";
      this->velocity_publisher.publish(string);

      old_nsec=new_nsec;
      old_ticks_rr=new_ticks_rr;
      old_ticks_rl=new_ticks_rl;
      old_ticks_fr=new_ticks_fr;
      old_ticks_fl=new_ticks_fl;

      //ROS_INFO("old_ticks_fl_2: [%d]", old_ticks_fl);

      ROS_INFO("I computed with ticks: [%f],[%f], [%f]", Vx_ticks,Vy_ticks,Wz_ticks);

    

  }

private:
  
  ros::NodeHandle n;
  ros::Subscriber velocity_subscriber;
  //int old_sec;
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

  ros::Publisher velocity_publisher;
  

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "velocity_calculator");
  
  velocity_sub my_velocity_sub;
  my_velocity_sub.run();

  return 0;
}
