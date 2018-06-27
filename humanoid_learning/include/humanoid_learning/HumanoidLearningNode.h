#ifndef HUMANOIDLEARNINGNODE_H
#define HUMANOIDLEARNINGNODE_H


#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include "humanoid_general/State.h"
#include "humanoid_general/Enums.h"
#include "humanoid_general/MathUtils.h"
#include "humanoid_loadmap/Mapping.h"

#include "ros/ros.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <humanoid_msgs/HumanoidControlMsg.h>
#include <humanoid_msgs/LearningMsg.h>
#include <humanoid_msgs/PerformanceMsg.h>
#include <humanoid_msgs/HumanoidPropertiesMsg.h>
#include <sensor_msgs/Imu.h>
#include <humanoid_msgs/LipParamsMsg.h>
#include <humanoid_msgs/JointStateMsg.h>
#include <humanoid_msgs/LoadMapConfigsSrv.h>


#include <dynamic_reconfigure/server.h>
#include <humanoid_msgs/HumanoidLearningConfig.h>



class HumanoidLearningNode
{
public:

    //ROS Node
    ros::NodeHandle    nh;
    ros::NodeHandle    nh_private;

    //ROS Subcriber
    typedef humanoid_msgs::JointStateMsg                JointStateMsg;
    typedef message_filters::Subscriber<JointStateMsg>  JointStateSub;
    boost::shared_ptr<JointStateSub>                    jointStateSubPtr;
    JointStateMsg  jointStateMsg;

    typedef sensor_msgs::Imu                            ImuMsg;
    typedef message_filters::Subscriber<ImuMsg>         ImuSub;
    boost::shared_ptr<ImuSub>                           imuEulerSubPtr;

    CArray   imuEulerMsgXData;
    CArray   imuEulerMsgYData;
    CArray   imuEulerMsgZData;

    CArray   imuEulerMsgXFreq;
    CArray   imuEulerMsgYFreq;
    CArray   imuEulerMsgZFreq;

    typedef humanoid_msgs::LipParamsMsg                       LipParamsMsg;
    typedef message_filters::Subscriber<LipParamsMsg>         LipParamsSub;
    boost::shared_ptr<LipParamsSub>                           lipParamsSubPtr;
    LipParamsMsg  lipParamsMsg;

    typedef humanoid_msgs::HumanoidPropertiesMsg              HumanoidPropsMsg;
    typedef message_filters::Subscriber<HumanoidPropsMsg>     HumanoidPropsSub;
    boost::shared_ptr<HumanoidPropsSub>                       humanoidPropsSubPtr;
    HumanoidPropsMsg humanoidPropsMsg;


    //ROS Publishers
    ros::Publisher                     controlTopic;
    humanoid_msgs::HumanoidControlMsg  controlMsg;
    ros::Publisher                     learningTopic;
    humanoid_msgs::LearningMsg         learningMsg;


    //ROS Dynamic Reconfigure Server
    typedef humanoid_msgs::HumanoidLearningConfig                 HumanoidLearningConfig;
    typedef dynamic_reconfigure::Server<HumanoidLearningConfig>   HumanoidLearningConfigServer;
    boost::shared_ptr<HumanoidLearningConfigServer>               config_server;
    boost::mutex mutex;



    //ROS  Services
    ros::ServiceClient                 mapCli;
    humanoid_msgs::LoadMapConfigsSrv   mapMsg;

    //Timers
    ros::Timer runTimer;

    //Members
    Mapping map;
    int     robotDOF;
    int     urdfDOF;
    int     ikDOF;
    double  dt;
    double  df;
    double  testTime;
    int     wPoints;
    int     wUpdate;
    bool    testUpdate;
    int     testPoint;
    int     imuCount;
    bool    imuInit;
    bool    lipParamsUpdate;

    double maxXAmp;
    double maxYAmp;
    double maxZAmp;
    double maxXFreq;
    double maxYFreq;
    double maxZFreq;

    double meanX;
    double meanY;
    double meanZ;


    HumanoidLearningNode(ros::NodeHandle nh_,ros::NodeHandle nh_private_);
    ~HumanoidLearningNode();

    //Methods
    void loadMap();
    int  calcWindowPoints(double totalDt,double dt);
    void resizeData(int nSamples);
    void getBiggerFreq(CArray &fft, double &max, double &maxIndex, double freqLower, double freqUpper);
    void calcPerformance();

    //Callbacks
    void runCallBack(const ros::TimerEvent&);
    void imuEulerCallback(const sensor_msgs::ImuPtr &msg);
    void lipParamsCallback(const humanoid_msgs::LipParamsMsgPtr &msg);
    void humanoidPropsCallback(const humanoid_msgs::HumanoidPropertiesMsgPtr &msg);
    void jointStateCallback(const humanoid_msgs::JointStateMsgPtr &msg);
    void reconfigCallback(humanoid_msgs::HumanoidLearningConfig& config, uint32_t level);


    void send2Topic();

};

#endif // HUMANOIDLEARNINGNODE_H

