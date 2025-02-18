/**
 * \file Turntable.h
 * \brief Header file of the turntable ROS service server class
 *
 * Header file of the turntable ROS service server class - Defines the attributes and methods used to send commands to the turntable
 *
 */

#define TURNTABLE_PAD 10
#define TURNTABLE_SAD 0
#define BOARD_DESC 0
#define MAX_PAD 31

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include "turntable_ros/Float.h"
#include "turntable_ros/Int.h"

#include <string>

/*! \class Turntable
* \brief Turntable class
*/
class Turntable
{
  public:
 
        /*!
        *  \brief Constructor
        *  \param turntablePAD Turntable GPIB primary address - default = 10.
        *  \param propellerServerName The prefix of the turntable ROS service server.
        */
        Turntable(int turntablePAD, std::string TurntablePrefix = "turntable_");

        /*!
        *  \brief Constructor with bus scanningto fing GPIB primary address.
        *  \param propellerServerName The prefix of the turntable ROS service server.
        */
        Turntable(std::string TurntablePrefix = "turntable_");
    
        /*!
        *  \brief Destructor.
        */
        ~Turntable();

        /*!
         *  \brief Returns turntable status.
         *  \return Turntable status : true if ready, false otherwise.
         */
        bool getStatus();
    
        /*!
        *   \brief Get turntable position.
        *   \return Turntable position in degrees.
        */
        int getPosition();

        /*!
        *   \brief Get turntable acceleration.
        *   \return Turntable acceleration, ranging from 1 to 10.
        */
        int getAcceleration();

        /*!
         *  \brief Wether the turntable stops at 360° or not.
         *  \return True if the turntable stops at 360°, False otherwise.
         */
        bool isMax360();

    private:

        /*!
         *  \brief Find GPIB primary address
         *  \return Primary GPIB address
         */
        int findPAD();

        /*!
         *  \brief Set turntable acceleration
         *  \param acceleration Turntable acceleration, ranging from 1 to 10
         */
        bool setAcceleration(turntable_ros::Int::Request &req, turntable_ros::Int::Response &res);

        /*!
         *  \brief Set absolute turntable position
         *  \param position Turntable absolute position in degrees
         */
        bool setAbsPosition(turntable_ros::Int::Request &req, turntable_ros::Int::Response &res);

        /*!
         *  \brief Set turntable position relatively to its current position
         *  \param position Turntable position variation in degrees
         */
        bool setRelPosition(turntable_ros::Int::Request &req, turntable_ros::Int::Response &res);

        /*!
         *  \brief Set turntable rotation limitation
         */
        bool setMax360(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

        /*!
         *  \brief Set turntable 0° reference
         */
        bool set0reference(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        /*!
         *  \brief Start turntable rotation
         */
        bool startRotation(turntable_ros::Float::Request &req, turntable_ros::Float::Response &res);

        /*!
         *  \brief Stop turntable rotation
         */
        bool stopRotation(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    ros::NodeHandle m_nodeHandle; /*!< ROS node handle */
    ros::ServiceServer m_turntableAccServer; /*!< Turntable acceleration ROS service server */
    ros::ServiceServer m_turntableAbsPosServer; /*!< Turntable position ROS service server */
    ros::ServiceServer m_turntableRelPosServer; /*!< Turntable position ROS service server */
    ros::ServiceServer m_turntableMaxServer; /*!< Turntable max 360° ROS service server */
    ros::ServiceServer m_turntableRefServer; /*!< Turntable 0° reference ROS service server */
    ros::ServiceServer m_turntableStartRotServer; /*!< Turntable start rotation ROS service server */
    ros::ServiceServer m_turntableStopRotServer; /*!< Turntable stop rotation ROS service server */

    std::string m_turntablePrefix; /*!< Turntable ROS service server prefix */

    int m_turntablePosition; /*! Turntable position */
    int m_turntableAcceleration;    /*! Turntable acceleration */
    bool m_max360;  /*! Turtable 360° limitation */

    int m_turntableID;  /*! Turntable ID */
    int m_boardID;  /*! Board ID (adaptator) */
};

