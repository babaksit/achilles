#include "ros/ros.h"
#include "QCoreApplication"
#include "csignal"
#include "achilles_controller/achilles_controller.h"

using namespace std;

void signalHandler( int signum )
{
   qDebug() << "Interrupt signal (" << signum << ") received.\n";
   exit(signum);
}


int main(int argc, char  **argv)
{
    QCoreApplication app(argc,argv);
    ros::init(argc,argv,"achilles_controller");
    ros::NodeHandle nh;
    Controller controller(&nh);

    signal(SIGINT,signalHandler);

    return app.exec();

}
