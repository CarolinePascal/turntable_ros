#include "Turntable.hpp"
#include <iostream>

int main( int argc, char **argv)
{
    int turntablePAD = TURNTABLE_PAD;

    if(argc >= 2)
    {
        turntablePAD = std::stoi(argv[1]);
    }

	Turntable turntable(turntablePAD);

    std::string input;

    std::cout << "TURNTABLE INTERFACE\n" << std::endl;
    
    while (true) {
        std::cout << "Enter a command (type 'quit' to exit): ";
        std::getline(std::cin, input);

        if (input == "quit") {
            std::cout << "\nExiting the program." << std::endl;
            break;
        }

        else if(input.find("getPosition") != std::string::npos)
        {
            std::cout << turntable.getPosition() << " degrees" << std::endl;
        }

        else if(input.find("getAcceleration") != std::string::npos)
        {
            std::cout << turntable.getAcceleration() << "/10" << std::endl;
        }

        else if(input.find("isMax360") != std::string::npos)
        {
            if(turntable.isMax360())
            {
                std::cout << "yes" << std::endl;
            }
            else
            {
                std::cout << "no" << std::endl;
            }
        }

        else if(input.find("setAcceleration") != std::string::npos)
        {
            size_t found = input.find("setAcceleration");
            input.erase(found,std::string("setAcceleration").length());

            int arg;
            try
            {
                arg = std::stoi(input);
            }
            catch(const std::exception& ex)
            {
                std::cout << "Invalid acceleration argument !" << std::endl;
            }

            if(arg < 1){arg = 1;}
            else if(arg > 10){arg = 10;}

            if(turntable.setAcceleration(arg))
            {
                std::cout << "Setting acceleration to " << arg << std::endl;
            }
            else
            {
                std::cout << "Acceleration setting failed !" << std::endl;
            }
        }

        else if(input.find("setAbsPosition") != std::string::npos)
        {
            size_t found = input.find("setAbsPosition");
            input.erase(found,std::string("setAbsPosition").length());

            int arg;
            try
            {
                arg = std::stoi(input);
            }
            catch(const std::exception& ex)
            {
                std::cout << "Invalid absolute position argument !" << std::endl;
            }

            if(turntable.setAbsPosition(arg))
            {
                std::cout << "Setting absolute position to " << arg << std::endl;
            }
            else
            {
                std::cout << "Absolute position setting failed !" << std::endl;
            }
        }

        else if(input.find("setRelPosition") != std::string::npos)
        {
            size_t found = input.find("setRelPosition");
            input.erase(found,std::string("setRelPosition").length());

            int arg;
            try
            {
                arg = std::stoi(input);
            }
            catch(const std::exception& ex)
            {
                std::cout << "Invalid relative position argument !" << std::endl;
            }

            if(turntable.setRelPosition(arg))
            {
                std::cout << "Setting relative position to " << arg << std::endl;
            }
            else
            {
                std::cout << "Relative position setting failed !" << std::endl;
            }
        }

        else if(input.find("setMax360") != std::string::npos)
        {
            size_t found = input.find("setMax360");
            input.erase(found,std::string("setMax360").length());

            int arg;
            try
            {
                arg = std::stoi(input);
            }
            catch(const std::exception& ex)
            {
                std::cout << "Invalid relative position argument !" << std::endl;
            }

            if(arg == 0 && turntable.setMax360(false))
            {
                std::cout << "Desactivating max360" << std::endl;
            }
            else if(arg == 1 && turntable.setMax360(true))
            {
                std::cout << "Activating max360" << std::endl;
            }
            else
            {
                std::cout << "max360 setting failed !" << std::endl;
            }
        }

        else if(input.find("set0reference") != std::string::npos)
        {
            if(turntable.set0reference())
            {
                std::cout << "0 reference set" << std::endl;
            }
            else
            {
                std::cout << "0 reference setting failed !" << std::endl;
            }
        }

        else if(input.find("startRotation") != std::string::npos)
        {
            size_t found = input.find("startRotation");
            input.erase(found,std::string("startRotation").length());

            float arg;
            try
            {
                arg = std::stof(input);
            }
            catch(const std::exception& ex)
            {
                std::cout << "Invalid rotation speed argument !" << std::endl;
            }

            if(turntable.startRotation(arg))
            {
                std::cout << "Starting rotation with speed " << arg << std::endl;
            }
            else
            {
                std::cout << "Rotation startup failed !" << std::endl;
            }
        }

        else if(input.find("stopRotation") != std::string::npos)
        {
            if(turntable.stopRotation())
            {
                std::cout << "Rotation stopped" << std::endl;
            }
            else
            {
                std::cout << "Rotation shutdown failed !" << std::endl;
            }
        }

        else
        {
            std::cout << "Invalid command !" << std::endl;
        }

    }

	return(0);
}
