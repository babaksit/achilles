#include "ros/ros.h"
#include "QCoreApplication"
#include "csignal"
#include "QDebug"
#include "achilles_navigation/achilles_navigation.h"

using namespace std;

void signalHandler( int signum )
{
    qDebug() << "Interrupt signal (" << signum << ") received.\n";
    exit(signum);
}

int main(int argc, char  **argv)
{
    QCoreApplication app(argc,argv);
    ros::init(argc,argv,"achilles_navigation");
    ros::NodeHandle nh;

    Navigation navigation(&nh);

    signal(SIGINT,signalHandler);

    return app.exec();

}
