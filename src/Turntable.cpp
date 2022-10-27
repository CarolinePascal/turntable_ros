#include "turntable_ros/Turntable.h"
#include "Adgpib.h"
#include <stdexcept>
#include <math.h>

Turntable::Turntable(int turntablePAD, std::string turntablePrefix)
{
    m_turntablePrefix = turntablePrefix;
    m_turntableID = ibdev(BOARD_DESC, turntablePAD, TURNTABLE_SAD, T3s, 1, 0);
    if(ibsta & ERR)
    {
        throw std::runtime_error("CANNOT FIND TURNTABLE");
    }
    ROS_INFO("Turntable ID : %d", m_turntableID);

    m_turntableAccServer = m_nodeHandle.advertiseService(m_turntablePrefix+std::string("set_acceleration"),&Turntable::setAcceleration,this);
    m_turntableAbsPosServer = m_nodeHandle.advertiseService(m_turntablePrefix+std::string("set_abs_position"),&Turntable::setAbsPosition,this);
    m_turntableRelPosServer = m_nodeHandle.advertiseService(m_turntablePrefix+std::string("set_rel_position"),&Turntable::setRelPosition,this);
    m_turntableMaxServer = m_nodeHandle.advertiseService(m_turntablePrefix+std::string("set_max360"),&Turntable::setMax360,this);
    m_turntableRefServer = m_nodeHandle.advertiseService(m_turntablePrefix+std::string("set_0_ref"),&Turntable::set0reference,this);
    m_turntableStartRotServer = m_nodeHandle.advertiseService(m_turntablePrefix+std::string("start_rotation"),&Turntable::startRotation,this);
    m_turntableStopRotServer = m_nodeHandle.advertiseService(m_turntablePrefix+std::string("stop_rotation"),&Turntable::stopRotation,this);

    ibwrt(m_turntableID,"acc. 1",6);
    ibwrt(m_turntableID,"max_360 on",10);
    ibwrt(m_turntableID,"set 0 deg",9);

    if(ibsta & ERR)
    {
        throw std::runtime_error("CANNOT SEND INITIALISATION COMMAND TO TURNTABLE");
    }

    m_turntableAcceleration = 1;
    m_max360 = true;
    m_turntablePosition = 0;
}

Turntable::Turntable(std::string turntablePrefix) : Turntable::Turntable(Turntable::findPAD(),turntablePrefix){}

Turntable::~Turntable(){}

int Turntable::findPAD()
{
    int turntablePAD = 0;

    Addr4882_t AllAddresses[MAX_PAD+1];
    Addr4882_t ListenAddresses[MAX_PAD];

    for(int i = 0; i < MAX_PAD-1; i++)
    {
        AllAddresses[i] = (Addr4882_t)(i+1);
    }
    AllAddresses[MAX_PAD-1] = NOADDR;

    int boardID = ibfind("gpib0");

    if(boardID == -1)
    {
        throw std::runtime_error("CANNOT FIND ADLINK ADAPTATOR");
    }

    ibrsc(boardID,1);
    ibpad(boardID,0);
    ibeot(boardID,1);
    SendIFC(boardID);

    FindLstn(boardID, AllAddresses, ListenAddresses, MAX_PAD);

    if(ibcntl < 1)
    {
        throw std::runtime_error("CANNOT FIND TURNTABLE");
    }
    else
    {
        turntablePAD = (int)ListenAddresses[0];
        if(ibcntl > 1)
        {
            ROS_WARN("Multiple turntable found ! Keeping the first one at GPIB primary address : %d",turntablePAD);
        }
    }

    return(turntablePAD);
}

int Turntable::getPosition()
{
    return(m_turntablePosition);
}

int Turntable::getAcceleration()
{
    return(m_turntableAcceleration);
}

bool Turntable::isMax360()
{
    return(m_max360);
}

bool Turntable::setAcceleration(turntable_ros::Int::Request &req, turntable_ros::Int::Response &res)
{
    std::string acceleration = std::to_string(req.value);
    std::string cmd = std::string("acc. ") + acceleration;

    ibwrt(m_turntableID,cmd.c_str(),cmd.length());

    if(ibsta & ERR)
    {
        ROS_ERROR("CANNOT SEND COMMAND TO TURNTABLE");
        res.success = false;
        return(false);
    }
    else
    {
        m_turntableAcceleration = req.value;
        res.success = true;
        return(true);
    }  
}

bool Turntable::setAbsPosition(turntable_ros::Int::Request &req, turntable_ros::Int::Response &res)
{
    std::string position = std::to_string(req.value);
    std::string cmd = std::string("turn_abs ") + position;

    ibwrt(m_turntableID,cmd.c_str(),cmd.length());
    ibwrt(m_turntableID,"start",5);

    if(ibsta & ERR)
    {
        ROS_ERROR("CANNOT SEND COMMAND TO TURNTABLE");
        res.success = false;
        return(false);
    }
    else
    {
        m_turntablePosition = req.value;
        res.success = true;
        return(true);
    }
}

bool Turntable::setRelPosition(turntable_ros::Int::Request &req, turntable_ros::Int::Response &res)
{
    std::string position = std::to_string(req.value);
    std::string cmd = std::string("turn_rel ") + position;

    ibwrt(m_turntableID,cmd.c_str(),cmd.length());
    ibwrt(m_turntableID,"start",5);

    if(ibsta & ERR)
    {
        ROS_ERROR("CANNOT SEND COMMAND TO TURNTABLE");
        res.success = false;
        return(false);
    }
    else
    {
        m_turntablePosition += req.value;
        res.success = true;
        return(true);
    }
}

bool Turntable::setMax360(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    if(req.data == m_max360)
    {
        res.success = true;
        return(true);
    }
    else
    {
        if(req.data)
        {
            ibwrt(m_turntableID,"max_360 on",10);
        }
        else
        {
            ibwrt(m_turntableID,"max_360 off",11);
        }

        if(ibsta & ERR)
        {
            ROS_ERROR("CANNOT SEND COMMAND TO TURNTABLE");
            return(false);
        }

        else
        {
            m_max360 = req.data;
            res.success = true;
            return(true);
        }  
    }
}

bool Turntable::set0reference(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    ibwrt(m_turntableID,"set 0 deg",9);

    if(ibsta & ERR)
    {
        ROS_ERROR("CANNOT SEND COMMAND TO TURNTABLE");
        res.success = false;
        return(false);
    }
    else
    {
        res.success = true;
        return(true);
    }
}

bool Turntable::startRotation(turntable_ros::Float::Request &req, turntable_ros::Float::Response &res)
{
    std::string invertedSpeed = std::to_string(round(100/req.value)/100);
    std::string cmd = std::string("cont. ") + invertedSpeed;

    ibwrt(m_turntableID,cmd.c_str(),cmd.length());
    ibwrt(m_turntableID,"start",5);

    if(ibsta & ERR)
    {
        ROS_ERROR("CANNOT SEND COMMAND TO TURNTABLE");
        res.success = false;
        return(false);
    }

    else
    {
        res.success = true;
        return(true);
    }
}

bool Turntable::stopRotation(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    ibwrt(m_turntableID,"stop",4);

    if(ibsta & ERR)
    {
        ROS_ERROR("CANNOT SEND COMMAND TO TURNTABLE");
        res.success = false;
        return(false);
    }
    else
    {
        res.success = true;
        return(true);
    }
}