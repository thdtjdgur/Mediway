#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include <math.h>

class change_vel {
    public:
        change_vel() {
            ros::NodeHandle nh;
            ros::NodeHandle private_nh("~");
            cmd_sub = nh.subscribe("cmd_vel", 10, &change_vel::change_value, this);
            new_cmd_pub = nh.advertise<geometry_msgs::Twist>("new_cmd_vel", 10);

            wait_start_time = ros::Time::now();  // 여기서 초기화
        }

        void spin() {
            ros::Rate rate(10);//1초에 10번 수행됨
            while (ros::ok()) {
                ros::spinOnce();
                changing();
                rate.sleep();
            }
        }

    private:
        ros::Subscriber cmd_sub;
        ros::Publisher new_cmd_pub;
        float linearx, lineary, angularz;
        float arduino_linearx, arduino_lineary, arduino_angularx, arduino_angulary, arduino_angularz;
        int front = 1;//처음 시작할는 무조건 턴 상황이라고 인식하게 변수 설정

        ros::Time wait_start_time;  // 대기 시작 시간

        void change_value(const geometry_msgs::Twist::ConstPtr& msg) {
            linearx = msg->linear.x;
            lineary = msg->linear.y;
            angularz = msg->angular.z;
        }

        void changing() {
            ros::Time now = ros::Time::now(); // now는 계속 증가하는 중임
            if ((now - wait_start_time).toSec() <= 3.0) { //직진->턴, 턴->직진으로 막 바뀌고, 바뀐 시간이 3초 미만일때 함수 나감
                ROS_INFO("waiting... %.1f left", 3.0 - (now - wait_start_time).toSec());
                return;
            }


            if(front == 1){//직진상황일때(오른쪽, 왼쪽 포함)
                if(linearx>=0 && angularz<=0){//직진이고 오른쪽일때
                    arduino_linearx = 1;
                    arduino_lineary = 1;
                    arduino_angularx = 23;//23arduino_angularx는 실제 로봇의 속도에 관여하므로 linearx의 값을 넣어줌
                    arduino_angularz = angularz;//얼마나 곡선을 크게 돌건지 결정함.
                    front = 1;
                }
                else if(linearx>=0 && angularz>0){//직진이고 왼쪽일때
                    arduino_linearx = 1;
                    arduino_lineary = -1;
                    arduino_angularx = 23;
                    arduino_angularz = angularz;
                    front = 1;
                }
                else{//직진 도중에 후진상황이 생겼을때->멈추고 3초간 대기에 들어감
                    arduino_linearx = 0;
                    arduino_lineary = 0;
                    arduino_angularx = 0;
                    arduino_angularz = 0;
                    wait_start_time = ros::Time::now();  // wait_start_time에 현재 시간 저장 //턴->직선 상황임
                    front = 0;
                }
            }

            else{//후진상황일
                if(linearx<=0 && angularz>=0){//후진이고 오른쪽일때
                    arduino_linearx = -1;
                    arduino_lineary = 1;
                    arduino_angularx = 23;
                    arduino_angularz = angularz;
                    front = 0;//턴 상황 on
                }
                else if(linearx<=0 && angularz<0){//후진이고 왼쪽일때
                    arduino_linearx = -1;
                    arduino_lineary = -1;
                    arduino_angularx = 23;
                    arduino_angularz = angularz;
                    front = 0;//턴 상황 on
                }
                else{//후진 도중에 직진상황이 생겼을때->멈추고 3초간 대기에 들어감
                    arduino_linearx = 0;
                    arduino_lineary = 0;
                    arduino_angularx = 0;
                    arduino_angularz = 0;
                    wait_start_time = ros::Time::now();  // wait_start_time에 현재 시간 저장 //턴->직선 상황임
                    front = 1;
                }
            }
            
            // 변경된 값 publish
            geometry_msgs::Twist change_msg;
            change_msg.linear.x = arduino_linearx;
            change_msg.linear.y = arduino_lineary;
            change_msg.angular.x = arduino_angularx;
            change_msg.angular.z = arduino_angularz;

            new_cmd_pub.publish(change_msg);
            //ROS_INFO("linearx = %f, angularz = %f", linearx, angularz);
            ROS_INFO("turn_dir = %f, linear_vel = %f, turn_vel = %f", change_msg.linear.x, change_msg.angular.x, change_msg.angular.y);
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "change_vel");
    change_vel change;
    change.spin();
    return 0;
}


