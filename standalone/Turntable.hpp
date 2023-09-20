/**
 * \file Turntable.hpp
 * \brief Turntable class implemeting the methods needed to use the turntable without ROS
 *
 * Turntable class implemeting the methods needed to use the turntable without ROS
 *
 */

#define TURNTABLE_PAD 10
#define TURNTABLE_SAD 0
#define BOARD_DESC 0
#define MAX_PAD 31

#include "Adgpib.h"

#include <stdexcept>
#include <math.h>
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
        */
        Turntable(int turntablePAD)
        {
            m_turntableID = ibdev(BOARD_DESC, turntablePAD, TURNTABLE_SAD, T3s, 1, 0);
            if(ibsta & ERR)
            {
                throw std::runtime_error("CANNOT FIND TURNTABLE");
            }
            printf("Turntable ID : %d", m_turntableID);

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

        /*!
        *  \brief Constructor with bus scanningto fing GPIB primary address.
        */
        Turntable() : Turntable(Turntable::findPAD()){}
    
        /*!
        *  \brief Destructor.
        */
        ~Turntable(){}
    
        /*!
        *   \brief Get turntable position.
        *   \return Turntable position in degrees.
        */
        int getPosition(){return(m_turntablePosition);}

        /*!
        *   \brief Get turntable acceleration.
        *   \return Turntable acceleration, ranging from 1 to 10.
        */
        int getAcceleration(){return(m_turntableAcceleration);}

        /*!
         *  \brief Wether the turntable stops at 360° or not.
         *  \return True if the turntable stops at 360°, False otherwise.
         */
        bool isMax360(){return(m_max360);}

        /*!
         *  \brief Set turntable acceleration
         *  \param acceleration Turntable acceleration, ranging from 1 to 10
         *  \return True on success, False otherwise
         */
        bool setAcceleration(int acceleration)
        {
            std::string accelerationCmd = std::to_string(acceleration);
            std::string cmd = std::string("acc. ") + accelerationCmd;

            ibwrt(m_turntableID,cmd.c_str(),cmd.length());

            if(ibsta & ERR)
            {
                printf("CANNOT SEND COMMAND TO TURNTABLE");
                return(false);
            }
            else
            {
                m_turntableAcceleration = acceleration;
                return(true);
            }  
        }

        /*!
         *  \brief Set absolute turntable position
         *  \param position Turntable absolute position in degrees
         *  \return True on success, False otherwise
         */
        bool setAbsPosition(int position)
        {
            std::string positionCmd = std::to_string(position);
            std::string cmd = std::string("turn_abs ") + positionCmd;

            ibwrt(m_turntableID,cmd.c_str(),cmd.length());
            ibwrt(m_turntableID,"start",5);

            if(ibsta & ERR)
            {
                printf("CANNOT SEND COMMAND TO TURNTABLE");
                return(false);
            }
            else
            {
                m_turntablePosition = position;
                return(true);
            }
        }

        /*!
         *  \brief Set turntable position relatively to its current position
         *  \param position Turntable position variation in degrees
         *  \return True on success, False otherwise
         */
        bool setRelPosition(int position)
        {
            std::string positionCmd = std::to_string(position);
            std::string cmd = std::string("turn_rel ") + positionCmd;

            ibwrt(m_turntableID,cmd.c_str(),cmd.length());
            ibwrt(m_turntableID,"start",5);

            if(ibsta & ERR)
            {
                printf("CANNOT SEND COMMAND TO TURNTABLE");
                return(false);
            }
            else
            {
                m_turntablePosition += position;
                return(true);
            }
        }

        /*!
         *  \brief Set turntable rotation limitation
         *  \param max360 True if the turntable stops at 360°, False otherwise
         *  \return True on success, False otherwise
         */
        bool setMax360(bool max360)
        {
            if(max360 == m_max360)
            {
                return(true);
            }
            else
            {
                if(max360)
                {
                    ibwrt(m_turntableID,"max_360 on",10);
                }
                else
                {
                    ibwrt(m_turntableID,"max_360 off",11);
                }

                if(ibsta & ERR)
                {
                    printf("CANNOT SEND COMMAND TO TURNTABLE");
                    return(false);
                }

                else
                {
                    m_max360 = max360;
                    return(true);
                }  
            }
        }

        /*!
         *  \brief Set turntable 0° reference
         *  \return True on success, False otherwise
         */
        bool set0reference()
        {
            ibwrt(m_turntableID,"set 0 deg",9);

            if(ibsta & ERR)
            {
                printf("CANNOT SEND COMMAND TO TURNTABLE");
                return(false);
            }
            else
            {
                return(true);
            }
        }

        /*!
         *  \brief Start turntable rotation
         *  \param speed Turntable rotation speed
         *  \return True on success, False otherwise
         */
        bool startRotation(float speed)
        {
            std::string invertedSpeed = std::to_string(round(100/speed)/100);
            std::string cmd = std::string("cont. ") + invertedSpeed;

            ibwrt(m_turntableID,cmd.c_str(),cmd.length());
            ibwrt(m_turntableID,"start",5);

            if(ibsta & ERR)
            {
                printf("CANNOT SEND COMMAND TO TURNTABLE");
                return(false);
            }

            else
            {
                return(true);
            }
        }

        /*!
         *  \brief Stop turntable rotation
         *  \return True on success, False otherwise
         */
        bool stopRotation()
        {
            ibwrt(m_turntableID,"stop",4);

            if(ibsta & ERR)
            {
                printf("CANNOT SEND COMMAND TO TURNTABLE");
                return(false);
            }
            else
            {
                return(true);
            }
        }

    private:

        /*!
         *  \brief Find GPIB primary address
         *  \return Primary GPIB address
         */
        static int findPAD()
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
                    printf("Multiple turntable found ! Keeping the first one at GPIB primary address : %d",turntablePAD);
                }
            }

            return(turntablePAD);
        };

        int m_turntablePosition; /*! Turntable position */
        int m_turntableAcceleration;    /*! Turntable acceleration */
        bool m_max360;  /*! Turtable 360° limitation */

        int m_turntableID;  /*! Turntable ID */
};

