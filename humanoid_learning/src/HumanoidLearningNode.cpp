#include "humanoid_learning/HumanoidLearningNode.h"


HumanoidLearningNode::HumanoidLearningNode(ros::NodeHandle nh_, ros::NodeHandle nh_private_) : nh(nh_) , nh_private(nh_private_)
{
    //Parameter Server
    if (!nh_private.getParam ("dt", dt))
        dt =  0.008; //s


    //Publisher Stuff
    controlTopic  = nh.advertise<humanoid_msgs::HumanoidControlMsg>("humanoid_control/cmd", 1000);
    learningTopic  = nh.advertise<humanoid_msgs::LearningMsg>("humanoid_learning/status", 1000);

    //Subscriber Stuffs
    imuEulerSubPtr.reset(new ImuSub(nh,"humanoid_control/imu_euler",1));
    imuEulerSubPtr->registerCallback(&HumanoidLearningNode::imuEulerCallback, this);

    lipParamsSubPtr.reset(new LipParamsSub(nh,"humanoid_walking/walking_params_state",1));
    lipParamsSubPtr->registerCallback(&HumanoidLearningNode::lipParamsCallback, this);

    humanoidPropsSubPtr.reset(new HumanoidPropsSub(nh,"humanoid_model/humanoid_properties",1));
    humanoidPropsSubPtr->registerCallback(&HumanoidLearningNode::humanoidPropsCallback, this);

    jointStateSubPtr.reset(new JointStateSub(nh,"humanoid_interface/get_jointState",1));
    jointStateSubPtr->registerCallback(&HumanoidLearningNode::jointStateCallback, this);

    //Service Stuff
    mapCli       = nh.serviceClient<humanoid_msgs::LoadMapConfigsSrv>("humanoid_loadmap/load");

    runTimer = nh.createTimer(ros::Duration(dt), &HumanoidLearningNode::runCallBack,this);

    loadMap();

    testUpdate = true;
    testPoint = 0;

    imuInit  = false;
    imuCount = 0;
    wUpdate = 512;
    wPoints = 100;
    df = 1/(wUpdate*dt);
    resizeData(wUpdate);
    ROS_INFO("[HUMANOID_LEARNING] WPoints: %d  df: %4.3f  dt: %4.3f",wPoints,df,dt);
    //Dynamic Reconfigure Server
    config_server.reset(new HumanoidLearningConfigServer(nh_private));
    HumanoidLearningConfigServer::CallbackType f = boost::bind(&HumanoidLearningNode::reconfigCallback, this, _1, _2);
    config_server->setCallback(f);

}


HumanoidLearningNode::~HumanoidLearningNode()
{

}


void HumanoidLearningNode::reconfigCallback(humanoid_msgs::HumanoidLearningConfig& config, uint32_t level)
{
    boost::mutex::scoped_lock lock(mutex);
    learningMsg.x.freqRange = config.freqXRange;
    learningMsg.y.freqRange = config.freqYRange;
    learningMsg.z.freqRange = config.freqZRange;

    learningMsg.x.ampRange = config.ampXRange;
    learningMsg.y.ampRange = config.ampYRange;
    learningMsg.z.ampRange = config.ampZRange;

    learningMsg.x.meanRange = config.meanXRange;
    learningMsg.y.meanRange = config.meanYRange;
    learningMsg.z.meanRange = config.meanZRange;

    learningMsg.x.wFreq = learningMsg.y.wFreq = learningMsg.z.wFreq = config.wFreq;
    learningMsg.x.wAmp = learningMsg.y.wAmp = learningMsg.z.wAmp = config.wAmp;
    learningMsg.x.wMean = learningMsg.y.wMean = learningMsg.z.wMean = config.wMean;


}


void HumanoidLearningNode::resizeData(int nSamples)
{
    Complex  imuEulerMsgSample[nSamples];
    imuEulerMsgXData   = CArray(imuEulerMsgSample,nSamples);
    imuEulerMsgYData   = CArray(imuEulerMsgSample,nSamples);
    imuEulerMsgZData   = CArray(imuEulerMsgSample,nSamples);
    imuEulerMsgXFreq   = CArray(imuEulerMsgSample,nSamples);
    imuEulerMsgYFreq   = CArray(imuEulerMsgSample,nSamples);
    imuEulerMsgZFreq   = CArray(imuEulerMsgSample,nSamples);

    for(int i = 0 ; i < nSamples; i++)
    {
        imuEulerMsgXData[i] = 0;
        imuEulerMsgYData[i] = 0;
        imuEulerMsgZData[i] = 0;
        imuEulerMsgXFreq[i] = 0;
        imuEulerMsgYFreq[i] = 0;
        imuEulerMsgZFreq[i] = 0;
    }

}

void HumanoidLearningNode::loadMap()
{
    mapMsg.request.update = false;
    if(mapCli.call(mapMsg))
    {
        int h = mapMsg.response.idMap.map.layout.dim[0].size;
        int w = mapMsg.response.idMap.map.layout.dim[1].size;
        std::vector<int> data =  mapMsg.response.idMap.map.data;
        Eigen::Map<Eigen::MatrixXi> mat(data.data(), h, w);
        map.map = mat;
        for(int i = 0; i < int(mapMsg.response.idMap.jNames.size());i++)
        {
            map.enumMap.insert(std::make_pair(mapMsg.response.idMap.jNames[i],i));
        }
        robotDOF = mapMsg.response.idMap.robotDOF;
        urdfDOF  = mapMsg.response.idMap.urdfDOF;
        ikDOF    = mapMsg.response.idMap.ikDOF;
        //map.print();
        ROS_INFO("[HUMANOID_INTERFACE] ROBOT_DOF: %d  URDF_DOF: %d  IK_DOF: %d",robotDOF,urdfDOF,ikDOF);
    }

}


void HumanoidLearningNode::runCallBack(const ros::TimerEvent&)
{
    if(testUpdate && wPoints > 0 && imuInit)
    {
        maxXAmp  = 0;
        maxYAmp  = 0;
        maxZAmp  = 0;
        maxXFreq = 0;
        maxYFreq = 0;
        maxZFreq = 0;
        meanX    = 0;
        meanY    = 0;
        meanZ    = 0;

        meanX = MathUtils::mean(imuEulerMsgXFreq);
        meanY = MathUtils::mean(imuEulerMsgYFreq);
        meanZ = MathUtils::mean(imuEulerMsgZFreq);

        MathUtils::fft(imuEulerMsgXFreq);
        MathUtils::fft(imuEulerMsgYFreq);
        MathUtils::fft(imuEulerMsgZFreq);

        getBiggerFreq(imuEulerMsgXFreq,maxXAmp,maxXFreq,0.5,8);
        getBiggerFreq(imuEulerMsgYFreq,maxYAmp,maxYFreq,0.5,8);
        getBiggerFreq(imuEulerMsgZFreq,maxZAmp,maxZFreq,0.5,8);/**/

        send2Topic();
        imuInit = false;
    }
}

void HumanoidLearningNode::getBiggerFreq(CArray &fft,double &max,double &maxIndex,double freqLower,double freqUpper)
{
   //Skips the first which is the DC
    int upIndex  = ceil(freqUpper/df);
    int lowIndex = ceil(freqLower/df);
    max = MathUtils::norm(fft[lowIndex]);
    maxIndex = lowIndex;
    for(int i = lowIndex + 1; i < upIndex;i++)
    {
        if(MathUtils::norm(fft[i].real()) > max)
        {
            maxIndex = i;
            max = MathUtils::norm(fft[i].real());
        }
    }
}

void HumanoidLearningNode::calcPerformance()
{
    learningMsg.x.freqPerf = MathUtils::absf(learningMsg.x.freq - learningMsg.x.freqRef)/(learningMsg.x.freqRange);
    learningMsg.y.freqPerf = MathUtils::absf(learningMsg.y.freq - learningMsg.y.freqRef)/(learningMsg.y.freqRange);
    learningMsg.z.freqPerf = MathUtils::absf(learningMsg.z.freq - learningMsg.z.freqRef)/(learningMsg.z.freqRange);

    learningMsg.x.ampPerf  = MathUtils::absf(learningMsg.x.amp - learningMsg.x.ampRef)/(learningMsg.x.ampRange);
    learningMsg.y.ampPerf  = MathUtils::absf(learningMsg.y.amp - learningMsg.y.ampRef)/(learningMsg.y.ampRange);
    learningMsg.z.ampPerf  = MathUtils::absf(learningMsg.z.amp - learningMsg.z.ampRef)/(learningMsg.z.ampRange);

    learningMsg.x.meanPerf = MathUtils::absf(learningMsg.x.mean - learningMsg.x.meanRef)/(learningMsg.x.meanRange);
    learningMsg.y.meanPerf = MathUtils::absf(learningMsg.y.mean - learningMsg.y.meanRef)/(learningMsg.y.meanRange);
    learningMsg.z.meanPerf = MathUtils::absf(learningMsg.z.mean - learningMsg.z.meanRef)/(learningMsg.z.meanRange);


    learningMsg.x.perf     = (learningMsg.x.freqPerf*learningMsg.x.wFreq + learningMsg.x.ampPerf*learningMsg.x.wAmp  + learningMsg.x.meanPerf*learningMsg.x.wMean)/(learningMsg.x.wFreq + learningMsg.x.wAmp + learningMsg.x.wMean);
    learningMsg.y.perf     = (learningMsg.y.freqPerf*learningMsg.y.wFreq + learningMsg.y.ampPerf*learningMsg.y.wAmp  + learningMsg.y.meanPerf*learningMsg.y.wMean)/(learningMsg.y.wFreq + learningMsg.y.wAmp + learningMsg.y.wMean);
    learningMsg.z.perf     = (learningMsg.z.freqPerf*learningMsg.z.wFreq + learningMsg.z.ampPerf*learningMsg.z.wAmp  + learningMsg.z.meanPerf*learningMsg.z.wMean)/(learningMsg.z.wFreq + learningMsg.z.wAmp + learningMsg.z.wMean);

    learningMsg.perf       = (learningMsg.x.perf + learningMsg.y.perf + learningMsg.z.perf)/3.;

}

void HumanoidLearningNode::send2Topic()
{
    learningMsg.header.stamp = ros::Time::now();
    learningMsg.x.bins.clear();
    learningMsg.y.bins.clear();
    learningMsg.z.bins.clear();
    for(int i  = 0; i < wPoints;i++)
    {
        learningMsg.x.bins.push_back(MathUtils::norm(imuEulerMsgXFreq[i]));
        learningMsg.y.bins.push_back(MathUtils::norm(imuEulerMsgYFreq[i]));
        learningMsg.z.bins.push_back(MathUtils::norm(imuEulerMsgZFreq[i]));
    }
    learningMsg.x.freq = maxXFreq*df;
    learningMsg.y.freq = maxYFreq*df;
    learningMsg.z.freq = maxZFreq*df;
    learningMsg.x.amp = maxXAmp*(2./wUpdate);
    learningMsg.y.amp = maxYAmp*(2./wUpdate);
    learningMsg.z.amp = maxZAmp*(2./wUpdate);
    learningMsg.x.mean = meanX;
    learningMsg.y.mean = meanY;
    learningMsg.z.mean = meanZ;
    learningMsg.df = df;
    calcPerformance();
    learningTopic.publish(learningMsg);
}


int HumanoidLearningNode::calcWindowPoints(double totalDt,double dt)
{
   int windowPoints = 0;
   windowPoints = (2*totalDt)/dt;
   return windowPoints;
}


void HumanoidLearningNode::imuEulerCallback(const sensor_msgs::ImuPtr &msg)
{
    if(testUpdate)
    {
        imuEulerMsgXData  = imuEulerMsgXData.cshift(1);
        imuEulerMsgXData[imuEulerMsgXData.size() - 1] = msg->orientation.x;

        imuEulerMsgYData  = imuEulerMsgYData.cshift(1);
        imuEulerMsgYData[imuEulerMsgYData.size() - 1] = msg->orientation.y;

        imuEulerMsgZData  = imuEulerMsgZData.cshift(1);
        imuEulerMsgZData[imuEulerMsgZData.size() - 1] = msg->linear_acceleration.z;


        imuCount++;
        if(imuCount == wPoints)
        {
            imuEulerMsgXFreq  = imuEulerMsgXData;
            imuEulerMsgYFreq  = imuEulerMsgYData;
            imuEulerMsgZFreq  = imuEulerMsgZData;
            imuInit  = true;
            imuCount = 0;
        }
    }
    else
    {
        testPoint = 0;
    }
}

void HumanoidLearningNode::lipParamsCallback(const humanoid_msgs::LipParamsMsgPtr &msg)
{
    this->lipParamsMsg = *msg;
    lipParamsUpdate = true;
}

void HumanoidLearningNode::humanoidPropsCallback(const humanoid_msgs::HumanoidPropertiesMsgPtr &msg)
{
    this->humanoidPropsMsg = *msg;
}

void HumanoidLearningNode::jointStateCallback(const humanoid_msgs::JointStateMsgPtr &msg)
{
    this->jointStateMsg = *msg;
}
