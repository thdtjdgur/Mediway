#include <ros/ros.h>
#include <std_msgs/Int32.h>
// #include <sensor_msgs/Imu.h> // [제거] IMU 헤더를 사용하지 않으므로 주석 처리
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include <math.h>

class EncoderIMULidarOdometry {
public:
//이 영역에 선언된 멤버들은 클래스 외부에서도 자유롭게 접근할 수 있음.
//예를 들어, 클래스 객체를 생성한 후, public 멤버 함수나 변수에 직접 접근하여 사용가능함.
    EncoderIMULidarOdometry() {
    //main함수에서 EncoderIMULidarOdometry odometry;를 실행할 때,
    //컴파일러는 내부적으로 EncoderIMULidarOdometry() 생성자를 호출하여 객체를 초기화함.
    //즉, 생성자 안에 작성된 초기화 코드들이 실행되어, 노드 핸들 생성, 파라미터 읽기, Subscriber와 Publisher 설정 등이 이루어짐.
        ros::NodeHandle nh;
        //NodeHandle은 ROS에서 노드를 관리하는 클래스임.
        //ROS 노드(Node)는 독립적으로 실행되는 프로그램이고, NodeHandle은 이 노드가 ROS 시스템과 통신할 수 있도록 도와주는 역할을 함.
        //NodeHandle을 사용하면 토픽을 발행(publish)하거나, 구독(subscribe)하거나, 서비스 요청을 처리하는 등의 작업을 할 수 있음.
        //nh라는 이름의 NodeHandle 객체를 생성
        ros::NodeHandle private_nh("~");
        //노드의 이름 아래에 속한 파라미터를 관리함
        //ros::는 **ROS 네임스페이스(namespace)**를 의미함.
        //ROS에서 제공하는 기능들은 ros 네임스페이스 안에 정의되어 있으며, 이를 통해 ROS 관련 클래스, 함수, 객체 등을 사용할 수 있음.
        //즉, ros::NodeHandle, ros::init(), ros::Publisher 같은 것들은 모두 ros 네임스페이스에 속해 있는 요소들임.

        private_nh.param("wheel_radius", wheel_radius_, 0.04705); // 바퀴 반지름 (m)
        private_nh.param("wheel_base", wheel_base_, 0.21200);// 바퀴 간 거리 (m)
        private_nh.param("pulses_per_revolution", pulses_per_rev_, 1612);// 펄스 수
        distance_per_pulse_ = (2 * M_PI * wheel_radius_) / pulses_per_rev_;//펄스당 이동거리(m)

        left_encoder_ = right_encoder_ = 0;
        prev_left_encoder_ = prev_right_encoder_ = 0;
        x_ = y_ = theta_ = 0.0;
        linear_velocity_ = angular_velocity_ = 0.0;
        // imu_yaw_ = 0.0; // [제거] IMU 관련 변수 주석 처리
        
        // Subscriber
        encoder_sub_ = nh.subscribe("encoder_pulse", 10, &EncoderIMULidarOdometry::encoderCallback, this);//this는 현재 객체를 가리키는 포인터임
        //encoderCallback은 클래스 내부의 멤버 함수이기 때문에, 퍼블리시노드에세 encoderCallback가 어떤 객체(EncoderOdometry)의 함수인지 알려줘야함.
        //이 문장은 main 문에서 EncoderOdometry 클래스의 객체가 생성될 때 한 번만 실행되고, 그 후에는 ROS 시스템이 자동으로 구독을 관리함.
        //그리고 그 후에는 ros::spinOnce()가 메시지를 받을 때마다 자동으로 콜백 함수(encoderCallback)가 호출됨.
        //encoder_sub_는 있으나 마나임
        // imu_sub_ = nh.subscribe("imu/data", 10, &EncoderIMULidarOdometry::imuCallback, this); // [제거] IMU 구독자 주석 처리
        //lidar_sub_ = nh.subscribe("scan", 10, &EncoderIMULidarOdometry::lidarCallback, this);
        

        // Publisher
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);//odom은 nav_msgs::Odometry 타입의 메시지 객체임
        //nav_msgs는 ROS에서 제공하는 메시지 패키지 중 하나로, 내비게이션 관련 메시지를 포함함.
        //이 패키지는 로봇의 위치, 속도, 회전 등과 같은 내비게이션 관련 정보를 표현하는 메시지 형식을 포함하고 있음.
        //Odometry는 로봇의 오도메트리 정보를 나타내는 메시지 타입임. 이 메시지에는 위치, 속도, 회전각 등이 포함됨.
        //즉, Odometry는 로봇의 현재 위치와 속도 정보를 담는 구조체임.
        //scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan", 10);

        prev_time_ = ros::Time::now();
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            calculateOdometry();//오도메트리 계산하고 퍼블리시
            publishSensorTF();//tf변환관계 방송함
            rate.sleep();
        }
    }

private://private:
//이 영역에 선언된 멤버들은 클래스 외부에서 접근할 수 없음.
//주로 내부 데이터나 구현 세부사항을 감추기 위해 사용되며, 클래스 내부의 멤버 함수만이 접근할 수 있음.
//이렇게 캡슐화(encapsulation)를 통해 데이터의 무분별한 접근을 막고, 클래스 내부 상태의 안정성을 보장할 수 있음.
    double wheel_radius_, wheel_base_, distance_per_pulse_;
    int pulses_per_rev_;
    int left_encoder_, right_encoder_, prev_left_encoder_, prev_right_encoder_;
    // double imu_yaw_; // [제거] IMU 관련 변수 주석 처리
    double x_, y_, theta_, linear_velocity_, angular_velocity_;
    ros::Time prev_time_;
    ros::Subscriber encoder_sub_, imu_sub_, lidar_sub_;
    ros::Publisher odom_pub_, scan_pub_;
    tf::TransformBroadcaster odom_broadcaster_;

    void encoderCallback(const geometry_msgs::Twist::ConstPtr& msg) {//"encoder_pulse" 주제로부터 메시지를 받으면 호출됨
        //geometry_msgs::Twist는 로봇의 선형 속도와 각속도를 표현하는 메시지 타입임.
        //linear와 angular라는 두 가지 속성을 포함하고 있음
        //여기서 Twist는 geometry_msgs 패키지 내에서 정의된 타입임. 즉, 이 함수의 매개변수로 받는 메시지 타입은 Twist 메시지임.
        //ConstPtr는 Twist 객체를 가리키는 상수 포인터임
        left_encoder_ = msg->angular.x;
        right_encoder_ = msg->angular.y;
    }

    // [제거] IMU 콜백 함수 전체를 주석 처리합니다.
    /*
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        double siny_cosp = 2.0 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y);
        double cosy_cosp = 1.0 - 2.0 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z);
        imu_yaw_ = atan2(siny_cosp, cosy_cosp);

        // 정규화
        if (imu_yaw_ < -M_PI) imu_yaw_ += 2 * M_PI;
        if (imu_yaw_ >  M_PI) imu_yaw_ -= 2 * M_PI;
    }
    */

    //void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {//lidarCallback()은 LiDAR 데이터를 받아 그대로 퍼블리시하는 함수
    //    scan_pub_.publish(*msg);
    //}

    void calculateOdometry() {
        ros::Time current_time = ros::Time::now();//현재 시간을 current_time에 저장
        double dt = (current_time - prev_time_).toSec();//이전 시간과의 차이를 dt에 저장. 즉 로봇이 이동한 시간
        prev_time_ = current_time;
        
        if (dt < 0.0001) { // dt가 너무 작은 경우 오류 방지를 위해 계산을 건너뜁니다.
            return;
        }

        int delta_left = left_encoder_ - prev_left_encoder_;//이전 엔코더 값과의 차이 계산
        int delta_right = right_encoder_ - prev_right_encoder_;
        prev_left_encoder_ = left_encoder_;
        prev_right_encoder_ = right_encoder_;

        double d_left = delta_left * distance_per_pulse_;//왼쪽 바퀴가 이동한 거리
        double d_right = delta_right * distance_per_pulse_;//오른쪽 바퀴가 이동한 거리
        double d = (d_left + d_right) / 2.0;//두 바퀴의 평균 이동 거리
        double d_theta = (d_left - d_right) / wheel_base_;//엔코더 값으로 로봇의 각도 변화량을 계산

        // [수정] 이동 후의 각도를 더 정확하게 반영하기 위해, 이동 거리에 현재 각도와 각도 변화량의 절반을 더한 값을 사용합니다.
        x_ += d * cos(theta_ + d_theta / 2.0);
        y_ += d * sin(theta_ + d_theta / 2.0);
        
        // [수정] IMU를 사용하는 대신 엔코더로 계산된 각도 변화량을 theta_에 누적합니다.
        theta_ += d_theta;

        if(theta_ > M_PI) theta_ -= 2 * M_PI;
        if(theta_ < -M_PI) theta_ += 2 * M_PI;

        linear_velocity_ = d / dt;
        angular_velocity_ = d_theta / dt; // [수정] 각속도도 엔코더 기반의 각도 변화량으로 계산합니다.


        ////////////////////////////////////////////////////////////////////////////////////////////로봇의 오도메트리(위치, 회전)를 계산하고 퍼블리시함
        //nav_msgs::Odometry 타입의 메시지를 생성하고, 헤더에 현재 시간을 설정함.
        nav_msgs::Odometry odom;//odom은 Odometry 메세시지 타입의 객체(변수)임. 이 변수는 Odometry 메시지를 담는 컨테이너 역할을 합.
        odom.header.stamp = current_time;//header는 Odometry 메시지의 헤더 부분임. 이 header에는 메시지의 메타데이터(예: 타임스탬프, 프레임 ID 등)가 포함되어 있음. header의 stamp는 타임스탬프를 나타내는 필드입
        odom.header.frame_id = "odom";//frame_id는 odom이라는 메시지가 **어떤 좌표계(frame)**에 대한 정보를 나타내는지 지정하는 필드임.
        odom.pose.pose.position.x = x_;//pose는 로봇의 위치와 방향(자세) 정보를 나타내는 필드임. pose 필드는 다시 pose라는 중첩된 필드를 가지고 있음.
        //position.x는 로봇이 x 축에 대한 위치 값을 설정하는 필드임. x_로봇의 x 좌표를 저장함
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);
        //오도메트리에서 회전을 나타내려면 각도(라디안)(theta_ )를 쿼터니언 형식으로 변환해야함. tf::createQuaternionMsgFromYaw(theta_) 함수가 이 변환을 해줌.
        //theta_ 값(로봇의 현재 회전각도)을 쿼터니언 형식으로 변환하고 odom_quat 변수에 저장하는 코드임.
        odom.child_frame_id = "base_link";//이 오도메트리 데이터(child_frame_id)는 로봇 본체(base_link)의 위치와 속도를 나타내는 데이터임
        odom.twist.twist.linear.x = linear_velocity_;//x축 방향(앞뒤) 선형 속도 (m/s)
        odom.twist.twist.angular.z = angular_velocity_;//평균 회전 속도를 저장
        odom_pub_.publish(odom);//생성한 오도메트리 메시지를 퍼블리시
        //ROS_INFO("\nX_POSITION: %4F, Y_POSITION: %4F\n", x_, y_);
        ////////////////////////////////////////////////////////////////////////////////////////////
    }

    void publishSensorTF() {
        ros::Time current_time = ros::Time::now();
   
        // ? map → odom 변환 제거 (GMapping에서 자동 처리)
            
        // ? odom → base_footprint 변환//odom 프레임에서 base_footprint 프레임으로의 변환 관계를 정의
        //odom은 로봇의 위치를 나타내는 기준 좌표계
        //base_footprint는 로봇의 물리적인 접점(바닥에 닿는 부분) 좌표
        geometry_msgs::TransformStamped odom_to_base_footprint_trans;//odom_to_base_footprint_trans = 내가 임의로 붙여준 이름임
        odom_to_base_footprint_trans.header.stamp = current_time;//해당 tf변환이 언제 기준의 데이터인지를 나타내며, tf시스템이 정확한 시점의 좌표 관계를 계산할 수 있게 해줌
        odom_to_base_footprint_trans.header.frame_id = "odom";//로봇이 출발한 기준좌표계임.
        odom_to_base_footprint_trans.child_frame_id = "base_footprint";//odom좌표계 기준으로 base_footprint 어디 있고 어떻게 움직이고 있는지를 표현한 것, 실제 로봇의 이동을 나타냄
        odom_to_base_footprint_trans.transform.translation.x = x_;//로봇의 x이동(로봇 정면 이동)
        odom_to_base_footprint_trans.transform.translation.y = y_;//로봇의 y이동(로봇 죄우 이동)
        odom_to_base_footprint_trans.transform.translation.z = 0.0;//로봇의 z이동(없음)
        odom_to_base_footprint_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta_);//로봇의 회전
        odom_broadcaster_.sendTransform(odom_to_base_footprint_trans);//이 시점에 odom->base_footprint좌표 변환 관계를 전체 ros시스템에 방송해라는 뜻.
        //이런 변환이 지금 이 시점에 이렇게 다는 사실을 다른 노드들에게 알려주는 역할임. 예를들오 rviz에서 base_footprint가 odom에서 어디에 있는지 몰라서 rviz상에서 로봇이 안 보일 수 있음
   
        //  base_footprint → base_link 변환//base_footprint(로봇의 바닥 기준)에서 base_link(로봇 본체)로 변환
        //일반적으로 base_footprint와 base_link는 같은 위치이므로 변환 없음 (0,0,0)
        geometry_msgs::TransformStamped base_footprint_to_base_link_trans;
        base_footprint_to_base_link_trans.header.stamp = current_time;
        base_footprint_to_base_link_trans.header.frame_id = "base_footprint";
        base_footprint_to_base_link_trans.child_frame_id = "base_link";
        base_footprint_to_base_link_trans.transform.translation.x = 0.0;
        base_footprint_to_base_link_trans.transform.translation.y = 0.0;
        base_footprint_to_base_link_trans.transform.translation.z = 0.0;
        base_footprint_to_base_link_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);
        odom_broadcaster_.sendTransform(base_footprint_to_base_link_trans);

        // [제거] IMU를 사용하지 않으므로 imu_link 관련 TF 발행 부분을 주석 처리합니다.
        /*
        //  base_link → imu_link 변환 (IMU 위치 설정)//IMU 데이터를 정확한 좌표계에서 사용할 수 있도록 설정
        //IMU 센서는 보통 로봇 중심부에 장착되며, 이 위치를 명확히 정의해야 올바른 자세(orientation) 정보가 유지됨
        geometry_msgs::TransformStamped base_link_to_imu_link_trans;
        base_link_to_imu_link_trans.header.stamp = current_time;
        base_link_to_imu_link_trans.header.frame_id = "base_link";
        base_link_to_imu_link_trans.child_frame_id = "imu_link";
        base_link_to_imu_link_trans.transform.translation.x = -0.035;
        base_link_to_imu_link_trans.transform.translation.y = 0.0;
        base_link_to_imu_link_trans.transform.translation.z = 0.215;
        base_link_to_imu_link_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);
        odom_broadcaster_.sendTransform(base_link_to_imu_link_trans);
        */
    }
};

//EncoderIMULidarOdometry는 오도메트리 계산 기능을 담은 클래스의 이름이고
//클래스 내부에는 엔코더 콜백 함수, 오도메트리 계산 함수, 그리고 ROS 토픽에 메시지를 퍼블리시하고 TF 변환 정보를 방송하는 함수들이 포함
//encoder_odometry는 ROS 노드 이름이자, 이 클래스(EncoderOdometry)의 인스턴스(객체)를 나타내는 변수 이름
int main(int argc, char** argv) {//main 함수는 프로그램 시작 시 한 번 실행되고, 이후 무한 루프(spin() 함수 내부에 while(ros::ok()))를 통해 프로그램이 계속 동작
    ros::init(argc, argv, "encoder_imu_lidar_odometry");//ros::init: ROS 노드를 "encoder_imu_lidar_odometry"라는 이름으로 초기화함
    EncoderIMULidarOdometry odometry;//객체가 생성되는 줄임. 이때 private안의 변수들이 초기화됨
    odometry.spin();//생성된 객체의 spin()멤버 함수 호출
    //spin() 함수는 ROS의 콜백 함수 처리와 오도메트리 계산 루프를 실행하여, 클래스 내부에서 정의된 기능들이 지속적으로 수행되도록 함
    return 0;
}