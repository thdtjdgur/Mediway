#include <ros/ros.h>//ros기본 헤더파일임. ros::init(), ros::NodeHandle 등의 ROS 관련 기능을 사용하기 위해 필요함
#include <geometry_msgs/Twist.h>//ROS에서 제공하는 메시지 타입 중 하나인 geometry_msgs/Twist를 포함함.
//로봇의 선형 속도(linear velocity)와 각속도(angular velocity)를 제어할 때 사용됨.
//ROS에서 이동하는 로봇을 제어할 때 cmd_vel 토픽을 통해 Twist 메시지를 보내는 방식이 일반적임.
#include <stdio.h>//printf(), scanf() 같은 입출력 함수를 사용할 수 있도록 함
#include <unistd.h>//sleep() 또는 usleep() 같은 함수를 사용하여 프로그램의 실행을 일시 중지하는 기능을 제공함
#include <termios.h>//터미널 입출력을 제어하는 기능을 제공함
#include <map>//map 클래스는 std 라는 네임스페이스 안에 들어 있음
//사용하려면 using namespace를 하거나, std::map 이런식으로 네임스페이스를 명시해 주어야 함

//std::map: C++ 표준 라이브러리의 연관 컨테이너로, 키(key)와 값(value)을 저장하는 자료구조
//char: map의 키(key) 타입 → 단일 문자 (예: 'w', 's', 'a', 'd' 등)
//std::vector<float>: map의 값(value) 타입 → float 타입을 원소로 가지는 벡터(동적 배열)
//moveBindings: std::map<char, std::vector<float>> 타입의 변수 선언 (이름: moveBindings)
std::map<char, std::vector<float>> moveBindings
{
  {'w', { 1,  0,  0}},
  {'a', { 0, -1,  0}},
  {'s', { 0,  0,  0}},
  {'d', { 0,  1,  0}},
  {'x', {-1,  0,  0}} 
};

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
{
  {'i', { 0.5,  0}},
  {'o', {-0.5,  0}},
  {'k', { 0,  1.0}},
  {'l', { 0, -1.0}}
};

// Reminder message
const char* msg = R"(

Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
        w    
   a    s    d
        x

velocity
i : up   (+0.5) m/s
o : down (-0.5) m/s

turn weight
k : up   (+1,0) rad/s
l : down (-1,0) rad/s

anything else : stop

CTRL-C to quit

)";

// Init variables
float velocity(1.0); // Linear velocity (m/s)
float turn(1.0); // Angular velocity (rad/s)
float x(0), y(0), z(0), th(0); // Forward/backward/neutral direction vars
char key(' ');

// For non-blocking keyboard inputs
int getch(void)//문자를 입력받을때 enter없이 키 입력을 즉시 감지할 떄 유용함
{
  int ch;//ch: 입력받은 문자를 저장할 변수
  struct termios oldt;//oldt: 기존 터미널 설정을 저장할 구조체 (termios 타입)
  struct termios newt;//newt: 새로운 터미널 설정을 저장할 구조체

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);//tcgetattr: 현재 터미널 설정을 가져오는 함수, 
  //STDIN_FILENO: 표준 입력(키보드)의 파일 디스크립터 (0)
  //&oldt: oldt 변수의 주소를 전달 (터미널 설정 저장)
  newt = oldt;//기존 설정을 복사하여 newt에 저장

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);//newt.c_lflag: 터미널의 로컬 설정(local flags)
  //ICANON: Canonical 모드 (줄 단위 입력) 활성화
  //ECHO: 입력한 문자를 화면에 표시하는 기능 활성화
  //&= ~(...): 해당 플래그를 비활성화 (비트 연산)
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;//VMIN = 1: 최소 1개의 문자가 입력되어야 read()가 반환됨
  newt.c_cc[VTIME] = 0;//VTIME = 0:	타임아웃 없음 (무한 대기)
  tcsetattr(fileno(stdin), TCSANOW, &newt);//tcsetattr(...): 터미널 속성을 변경하는 함수
  //fileno(stdin): 표준 입력(stdin)의 파일 디스크립터 (0)
  //TCSANOW: 설정을 즉시 적용
  //&newt: newt의 변경된 값을 적용

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);//TCSANOW: 즉시 적용

  return ch;//입력받은 문자를 반환
}

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "teleop_twist_keyboard");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  // 최대 1개의 메시지만 저장 가능(1은 큐의 크기임)
  //구독자가 메시지를 받기 전에 새로운 메시지가 들어오면 이전 메시지는 사라짐 (덮어씌워짐)
  //최신 메시지만 유지되므로 실시간성이 중요한 경우 유리

  // Create Twist message
  geometry_msgs::Twist twist;

  printf("%s", msg);//위에서 정의한 설명문 출력
  printf("\rCurrent: velocity %f\tturn %f | Awaiting command...\r", velocity, turn);

  while(true)
{
    key = getch();//getch() 함수는 기본적으로 키보드 입력을 받는 역할

    // If the key corresponds to a key in moveBindings
    if (moveBindings.count(key) == 1)//count: moveBindings라는 변수에 key(키보드로 입력받음)가 있으면 1을 반환함.
    {
      // Grab the direction data
      x = moveBindings[key][0];
      y = moveBindings[key][1];
      z = moveBindings[key][2];
      
      if (key == 's')
      {
      velocity = 21;
      turn = 21;
      }
    }

    // Otherwise if it corresponds to a key in speedBindings
    else if (speedBindings.count(key) == 1)
    {
      // Grab the speed data
      velocity = velocity + speedBindings[key][0];
      turn = turn + speedBindings[key][1];
    }

    // Otherwise, set the robot to stop
    else
    {
      velocity = 21;
      turn = 21;
      
      if (key == '\x03')//Ctrl + C를 입력하면
        break;//노드 종료
    }
    
    printf("\rCurrent: velocity %f m/s\tturn %f rad/s\tLast command: %c   ", velocity, turn, key);
    
    // Update the Twist message
    twist.linear.x = x;//로봇이 어느방향으로 갈지에 대한 벡터정보 넘김
    twist.linear.y = y;//나중에 아두이노ide로 pub하면 거기서 백터정보를 바탕으로 모터를 움직임
    twist.linear.z = 0;

    twist.angular.x = velocity;//선속도(직선속도)
    twist.angular.y = turn;//각속도(턴속도)
    twist.angular.z = 0;

    // Publish it and resolve any remaining callbacks
    pub.publish(twist);
    ros::spinOnce();
  }

  return 0;
}
