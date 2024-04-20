#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
{
  {'w', {2.0, 0.0}},
  {'a', {0.0, 0.2}},
  {'d', {0.0, -0.2}},
  {'s', {0.0, 0.0}},
  {'x', {-2.0, 0.0}},
};

// Reminder message
const char* msg = R"(

Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
        w     
   a    s    d
        x     

CTRL-C to quit

)";

// Init variables
float speed(1.0); // Linear velocity (m/s) 초기 스피드 
float turn(1.0); // Angular velocity (rad/s)
float x(0), y(0), z(0), th(0); // Forward/backward/neutral direction vars
char key(' ');

// , 유닉스 계열 시스템에서 터미널 입력 설정을 변경하여 한 문자를 사용자로부터 비블로킹(non-blocking) 방식으로 입력받는 기능
int getch(void)//getch 함수는 터미널 설정을 변경하여 키보드에서 직접 입력을 받을 수 있게 함. 
//이는 기본적으로 입력 후 엔터를 누르는 것을 요구하는 표준 입력 방식과 달리, 키를 누르는 즉시 입력을 받을 수 있게 해줌
{// 종합하면, 터미널 설정을 임시로 변경하는 함수. 
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  //한 문자씩 입력받는데, 위에서 설정한 터미널 설정으로 인해
  //엔터 키 없이 즉시 문자 받을 수 있음 
  ch = getchar();

  //원래 터미널 세팅 복구 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "chang_teleop_pub");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // Create Twist message
  geometry_msgs::Twist twist;

  printf("%s", msg);
  printf("\rCurrent: speed %f\t     turn %f| Awaiting command...\r", speed, turn);

  while(true){

    // 키보드 입력을 key변수에 저장 
    key = getch();

    // 입력된 키가 speedbing map에 존재할 경우 mapping된 속도로 update.
    if (speedBindings.count(key) == 1)
    {
      // speedBinding map에 있는 키 + speed와 합하여 update
      // speedbinding map에 있는 키 + turn과 합하여 update
      speed = speed+speedBindings[key][0];
      turn = turn+speedBindings[key][1];

      //display
      printf("\rCurrent: speed %.2f\t     turn %.2f| Last command: %c   ", speed, turn, key);
    }
    // Otherwise, set the robot to stop
    else
    {
      x = 0;
      y = 0;
      z = 0;
      th = 0;

      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03')
      {
        printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
        break;
      }

      printf("\rCurrent: speed %.3f\t  turn %.3f | Invalid command! %c", speed, turn, key);
    }
    if(key == 115)//S키(ASCII 코드 115) ==> STOP 
    {
      speed = 0;
      turn = 0;
    }

    //Twist message를 update 선속도=> x축, 각속도z축 
    twist.linear.x = speed;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = turn;

    // Publish it and resolve any remaining callbacks
    pub.publish(twist);
    ros::spinOnce();
  }

  return 0;
}