#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <actionlib_msgs/GoalStatusArray.h> // move_base 상태 확인용 추가
#include <vector>
#include <cmath>
#include <algorithm>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

class FrontierExploration {
public:
    FrontierExploration() {
        ros::NodeHandle nh;

        // 맵 및 오도메트리 구독
        map_sub_ = nh.subscribe("/map", 10, &FrontierExploration::mapCallback, this);//map데이터를 sub함(미확인 지역 확인을 위해)
        odom_sub_ = nh.subscribe("/odom", 10, &FrontierExploration::odomCallback, this);//odom데이터를 sub함(로봇의 위치를 알기 위해)
        move_base_status_sub_ = nh.subscribe("/move_base/status", 10, &FrontierExploration::moveBaseStatusCallback, this); // move_base 실행상태 확인

        // 목표 지점 발행
        goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);//미확인 지역을 확인하고 그 좌표를 move_base패키지에 넘김
        marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);//목표좌표(goal_x, goal_y)를 rviz에 표시
        new_cmd_pub = nh.advertise<geometry_msgs::Twist>("new_cmd_vel", 10);

        goal_x = 0.0;
        goal_y = 0.0;
        goal_reached = true; //목표좌표에 도달했는지를 판단함
        move_base_active = false; //frontier노드 시작했을때는 false, move_base패키지 실행하면 true로 변환됨
    }

    void spin() {
        ros::Rate rate(15);  // while문 주기 (15Hz) //콜백함수가 실행되는 속도이기도 함(왜냐면 while문 안에 있음.)
        //즉 퍼블리셔가 10hz로 보내고 ros::Rate rate(1);이면 콜백함수가 호출됬을때, 그 동안 큐에 쌓인 데이터를 한번에 처리함
        while (ros::ok()) {
            ros::spinOnce();//현재까지 요청된 큐에 쌓인 콜백함수를 모두 호출, 실행하고 while문 안의 다음코드의 부분으로 넘어감
            if (goal_reached && move_base_active) { //move_base패키지가 실행되고 처음 실행되었을때는 goal_reached가 true이므로 조건문에 들어감
                calculateGoal(); //목표좌표 설정
                goal_reached = false; //calculateGoal에서 목표좌표가 설정되었으므로 false를 하고 나중에 목표좌표에 도달하면 true로 바뀜
            }
            rate.sleep();//ros::Rate rate(10);일때 한번 루프를 도는 시간은 66.7ms(1초/15)임. 근데 실제로 20ms걸렸다면 66.7ms-20ms를 
                         //기다려야 하기위한 코드임
        }
    }

private:
    int width = 0, height = 0;
    float robot_x = 0.0, robot_y = 0.0;
    float goal_x = 0.0, goal_y = 0.0;
    float resolution = 0.0;
    geometry_msgs::Point origin;
    bool goal_reached = true; //목표좌표에 도달했는지를 판단함
    bool move_base_active = false; //frontier노드 시작했을때는 false, move_base패키지 실행하면 true로 변환됨
    bool non_finish = true;

    struct Point {
        float x;
        float y;
    };

    std::vector<std::vector<Point>> grid;//std::vector는 c++ 표준 라이브러리에서 제공하는 동적배열 자료형임
    //std::vector<std::vector<Point>> grid;는 벡터의 벡터를 정의하는 코드임. 즉 2차원 배열을 정의함
    //Point이 자리는 배열 요소의 타입을 입력하는 자리임
    ros::Subscriber map_sub_, odom_sub_, move_base_status_sub_;
    ros::Publisher goal_pub_, new_cmd_pub;
    ros::Publisher marker_pub_;
    nav_msgs::OccupancyGrid map_data_;
    tf::TransformListener tf_listener_;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        width = msg->info.width; //global_costmap_params파일에서 width와 height는 실제 rviz상에서 픽셀의 개수를 나타냄
        //그리고 msg->info.resolution는 한 픽셀의 크기(여기서는 0.05m)임. 그리고  msg->info.width는 격자무늬안의 실제 셀의 개수임
        //즉 msg->info.width는 15m/0.05m = 300, 즉 가로 세로의 셀의 개수는 300*300임, 근데 실제로 출력하면 384로 나오고 15나0.05를 바꿔도 384로 고정됨
        height = msg->info.height; //map좌표에서 세로픽셀이 몇개인지 판단(미터값 이님). 이 갯수를 바탕으로 실제 월드 좌표 생성.
        resolution = msg->info.resolution;//하나의 셀이 실제 몇 미터(m)인지 나타내는 값임. 그냥 gmapping실행할때 적절히 정해줌
        origin = msg->info.origin.position;//실제 월드 좌표에서 맵의 원점 (좌하단 기준), (-10,-10)임.좌하단은 rvis화면을 돌리면 좌하단을 맞출 수 있음. 정확히 아는 방법은 origin.x+1을 rviz화면에 점으로 표시해서 알 수 있음

        grid.resize(height, std::vector<Point>(width));//;2D 벡터(배열)(grid)를 height x width 크기로 초기화하는 코드
        map_data_ = *msg;
    }

    // odom기준의 로봇 위치를 map위치로 변환하여 목표 지점과의 거리 계산함수
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        tf::Pose odom_pose;//odom 프레임에서 측정된 로봇의 pose를 tf::Pose로 변환하고 그 변환된 값을 담기 위한 변수설정
        tf::poseMsgToTF(msg->pose.pose, odom_pose);// odom 프레임에서 측정된 로봇의 자세를 tf에서 사용 가능한 형식으로 변환
       
        // odomPose를 변환할 결과를 저장할 tf::StampedTransform 객체
        tf::StampedTransform transform;//tf::StampedTransform: 시간 정보(Stamp)와 좌표계(Frame ID)를 포함한 변환(translation과 rotation)을 나타내는 클래스//이 변수에 "map" 프레임으로의 변환 정보를 저장
        try {//try: 이 블록 내의 코드를 실행하는 동안 예외가 발생하면 catch 블록으로 제어가 넘어감
            // "map" 프레임으로의 변환을 기다렸다가 가져옴.
            tf_listener_.waitForTransform("map", msg->header.frame_id, msg->header.stamp, ros::Duration(0.1));//"map"과 odometry 메시지의 frame 사이의 변환 정보가 준비될 때까지 최대 0.1초 동안 대기
            tf_listener_.lookupTransform("map", msg->header.frame_id, msg->header.stamp, transform);// 지정한 시간에 "map" 프레임과 "odom"사이의 변환 정보를 읽어와, transform 객체에 저장
        } catch (tf::TransformException &ex) {
            ROS_WARN("Transform failed: %s", ex.what());
            return;
        }
       
        // 변환: map 프레임에서의 pose를 계산함.
        tf::Pose map_pose = transform * odom_pose;//odom 프레임에서 측정된 로봇의 odom_pose를, 얻은 transform을 통해 "map" 프레임으로 변환하여 map_pose에 저장
       
        // 변환된 map_pose로부터 위치 값을 추출합니다.
        double map_x = map_pose.getOrigin().x();//map상에서 실제 로봇의 좌표값 추출(실제 미터값임)
        double map_y = map_pose.getOrigin().y();//getOrigin는 tf라이브러리에서 제공하는 메서드임. 해당 pose의 y위치를 의미함(미터단위임)
       
        // map 프레임에서 목표(goal_x, goal_y)와의 거리를 계산함.
        float distance = sqrt(pow(map_x - goal_x, 2) + pow(map_y - goal_y, 2));//로봇과 목표좌표와의 실제거리
        //ROS_INFO("dist between ROBOT (in map frame) and GOAL: %f", distance);
        robot_x = map_x;
        robot_y = map_y;
        if (distance < 0.6) {
            if (!goal_reached) {
                goal_reached = true;
            }
        }
    }

    void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
        // move_base의 상태가 비어 있지 않으면 활성화된 상태
            move_base_active = true; //move_base패키지가 방금 막 실행되면 move_base_active를 true로 함
            //ROS_INFO("Move base start!");
    }

    void calculateGoal() {
        if (!move_base_active) return; // move_base가 실행 중이 아니면 목표 계산함수에 안들어감(확인용도)
   
        std::vector<float> length;
        std::vector<Point> unexplored_points; //미확인 지역들의 좌표저장 벡터임
        non_finish = false;
   
        for (int i = 0; i < height; ++i) {//세로
            for (int j = 0; j < width; ++j) {//가로
                int index = i * width + j;
                if (index < 0 || index >= width * height) continue;//continue는 현재 반복을 중단하고 다음 반복을 실행하는 명령어임
   
                if (map_data_.data.empty()) { //미확인 지역이 없다면 calculateGoal함수 빠져나감(지도작성 완료했을때 뜸)
                    ROS_WARN("Map data is empty. Skipping goal calculation.");
                    return;
                }
   
                if (map_data_.data[index] == -1) { //미확인 지역
                    float x = origin.x + j * resolution;//그리드 좌표(grid index)를 실제 월드 좌표(meters)로 변환하는 공식
                    float y = origin.y + i * resolution;//좌하단 기준(origin.x,y)으로 목표좌표 생성(일반 수학에서 쓰는 2차원 평면 생각하셈(1사분면))
                    //나중에 x,y들중 로봇과 목표좌표와의 거리중 가장 작은 x,y값을 goal_x, goal_y에 넣음
                    grid[i][j].x = x;//grid벡터에 x저장(실제 원점에서 해당 지점까지의 실제 미터값임)
                    grid[i][j].y = y;//(x,y)는 실제 원점(-10,-10)에서 미확인영역까지 떨어진 거리임.
   
                    float dist = sqrt(pow(robot_x - x, 2) + pow(robot_y - y, 2));//로봇과 현재 검사중인 좌표까지의 거리
                   

                    /////////////////////////////////////////////////////////////현재 검사중인 셀로부터 50cm이내에 장애물이 있으면 바로 탈출(목표좌표로 설정 안하기 위함)
                    //아래 조건문(for)들은 미확인좌표로부터 2칸 떨어진 범위를 검사하다가 장애물을 감지하면 해당 좌표safe변수를 false로 함
                    bool safe = true; //해당 좌표가 안전한지 여부를 나타내는 변수임
                    int nearby_clear_cells = 0; // 50cm 범위 내의 확인된 좌표의 수를 셈
                    for (int di = -2; di <= 2; di++) {  // 50cm 범위를 검사 (해상도 0.05 → 6셀)(세로칸 움직임)
                        for (int dj = -2; dj <= 2; dj++) {//(가로칸 움직임)
                            int obs_index = (i + di) * width + (j + dj);//현재 검사중인 셀의 index임
                            if (obs_index >= 0 && obs_index < width * height) { //맵 바깥쪽을 참조하면 안되므로 0이상. 맵 전체크기보다 작으면 조건문에 들어감
                                int map_value = map_data_.data[obs_index];
                                if (map_value == 100) { // 장애물(100) 감지하면 바로 조건문 탈출함
                                    safe = false;
                                    break;
                                }
                            }
                        }
                        if (!safe) break;
                    }
                    /////////////////////////////////////////////////////////////

 

                    /////////////////////////////////////////////////////////////위 for문에서 safe=false이면 이 조건문도 바로 빠져나감, 맵 바깥쪽(벽)에 좌표가 생성되지 않도록 nearby_clear_cells를 체크함
                    for (int di = -2; di <= 2; di++) {  // 20cm 범위를 검사 (해상도 0.05 → 4셀)
                        for (int dj = -2; dj <= 2; dj++) {
                            int obs_index = (i + di) * width + (j + dj);//현재 검사중인 셀의 index임
                            if (obs_index >= 0 && obs_index < width * height) { //맵 바깥쪽을 참조하면 안되므로 0이상. 맵 전체크기보다 작으면 조건문에 들어감
                                int map_value = map_data_.data[obs_index];
                                if (map_value == 0) { // 미확인 좌표로부터 2칸이내에 확인된 좌표가 몇개인지 확인
                                    nearby_clear_cells++;
                                }
                            }
                        }
                        if (!safe) break;
                    }
                    /////////////////////////////////////////////////////////////



                    /////////////////////////////////////////////////////////////첫번째 for문에서 safe = true(검사중인 좌표의 5칸이내에 장매물 없을때)인 상태로 내려오면(2번째 for문에서 근처 확인된 좌표개수 파악된 상태임) 이 조건문에 들어옴
                    if (safe && dist > 0.6 && nearby_clear_cells >= 8) {  //장애물주위로 2칸 떨어진곳에 장애물이 없을때, 그리고 로봇과 목표와의 거리 60cm 이상일때,
                        length.push_back(dist);                            //그리고 목표좌표 주위 2칸이내에 확인된 좌표가 14개 이상일때
                        unexplored_points.push_back({x, y});
                        non_finish = true;
                    }
                    /////////////////////////////////////////////////////////////
                }
            }
        }//지도의 모든 좌표가 검사 된 후 나옴

        if(non_finish == false){//지도 작성이 끝난 상태, non_finish가 true로 안바껴있으면 검사할 좌표가 없단 뜻-->지도작성 완료
            geometry_msgs::Twist change_msg;
            change_msg.linear.x = 0;
            change_msg.linear.y = 0;
            change_msg.angular.x = 0;
            change_msg.angular.z = 20;

            new_cmd_pub.publish(change_msg);
        }

        if (!length.empty()) {
            auto min_it = std::min_element(length.begin(), length.end());//검사중인 좌표들까지의 dist들중 가장 작은 dist선정
            int min_index = std::distance(length.begin(), min_it);//가장 작은 dist가 속한 index뽑음

            goal_x = unexplored_points[min_index].x;//dist가 가장 작은 지역의 실제 월드좌표를 목표좌표로 설정(goal_x)
            goal_y = unexplored_points[min_index].y;//나중에 goal_x는 odomCallback함수에서 로봇이 목표하는 좌표에 도달여부를 판단하는 기준이 됨

            //ROS_INFO("New goal set: x = %f, y = %f, distance = %f", goal_x, goal_y, *min_it);

            // 목표 지점 발행
            geometry_msgs::PoseStamped goal_msg;
            goal_msg.header.stamp = ros::Time::now();
            goal_msg.header.frame_id = "map";
            goal_msg.pose.position.x = goal_x;
            goal_msg.pose.position.y = goal_y;
            goal_msg.pose.orientation.w = 1.0;

            goal_pub_.publish(goal_msg);//목표좌표 발행

            // Marker로 목표 지점 표시
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "goals";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::SPHERE;//구 형태로 표시
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = goal_x;
            marker.pose.position.y = goal_y;
            marker.pose.position.z = 0.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 0.0;//빨강
            marker.color.g = 0.0;//초록
            marker.color.b = 1.0;//파랑
            marker.color.a = 1.0;//투명도

            marker_pub_.publish(marker);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_exploration");
    FrontierExploration goal;
    goal.spin();
    return 0;
}







