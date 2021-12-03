//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
// SMARTIES 
// Simulation Modules for Aircraft Real-Time Embedded Systems
// 
// XPLANE HLA PLUGIN
//
// Copyright (C) 2018-2022  ISAE-SUPAERO
//
// Author: Jean-Baptiste Chaudron
// email:  jean-baptiste.chaudron@isae-supaero.fr
//
//----------------------------------------------------------------------
//----------------------------------------------------------------------

#include <XplaneFederateHla1516e.hh>
#include <limits>
#include <chrono>
#ifndef _WIN32
#include <sys/time.h>
#endif
#include <thread>
//#include "ClientTestInterface.h"

using namespace std::chrono_literals;

void launch(XplaneFederateHla1516e &federate)
{
    std::string command("");
    double positionX{0.};
    double positionY{0.};
    double positionZ{0.};
    bool continuing = true;

    while (continuing) {
        std::cout << "Enter a command ('help' to print the available commands)" << std::endl;
        std::cin >> command;
        if (command == "help" || command == "h") {
            // Print help
            std::cout << "command possibles:" << std::endl;
            std::cout << "    h, help          : Print this help." << std::endl;
            std::cout << "    x, double value  : set value X coordinate." << std::endl;
            std::cout << "    y, double value  : set value Y coordinate." << std::endl;
            std::cout << "    z, double value  : set value Z coordinate." << std::endl;
            std::cout << "    s, send          : Send value to certi" << std::endl;
            std::cout << "    u, update        : Update current position to certi" << std::endl;
            std::cout << "    p, print         : Print the current value of the Mini objects of all federates." << std::endl;
            std::cout << "    t, tick          : send a tick to advance in time." << std::endl;
            std::cout << "    q, quit          : Quit the application." << std::endl;
        } else if (command == "X" || command == "x") {
            std::cin >> positionX;
        } else if (command == "Y" || command == "y") {
            std::cin >> positionY;
        } else if (command == "Z" || command == "z") {
            std::cin >> positionZ;
        } else if (command == "send" || command == "s") {
//            federate.sendMyPosition();
//            federate.sendMyCommands();
//            if(federate.isManualTimeManagement())
//            {
//                federate.updateAttributesPosition();
//                federate.updateAttributeCommands();
//                federate.step();
//            }
//            // If there is time management enabled, you have tp advance manually
//            if(federate.isManualTimeManagement())
//                federate.step();
        } else if (command == "update" || command == "u") {
//            federate.forceAllOtherPlanesPosition(-24, 133, 1250);
//            if(federate.isManualTimeManagement())
//            {
//                federate.sendInteractionForcePosition();
//                federate.step();
//            }
//            federate.printPositions();
        } else if (command == "print" || command == "p") {
            federate.printOtherPlanes();
//            federate.printPositions();
        } else if (command == "tick" || command == "t") {
            federate.runOneStep();
        } else if (command == "quit" || command == "q") {
            continuing = false;
        }

    }
}

int main(int argc, char *argv[])
{
//    std::wstring wideFederateName(L"TEST");
//    bool isPlane{false};
//    ClientTestInterface callbackClientTest;
//    Federate f(wideFederateName, &callbackClientTest, isPlane);
//    f.sendCommands(L"xplane", PlaneInterface::COMMAND::PITCH_UP);
//    f.step();

    std::cout << "Welcome to the Federation!" << std::endl;

    std::string federateName;
    std::cout << "Enter your federate name:" << std::endl;
    std::cin >> federateName;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    std::wstring wideFederateName(federateName.begin(), federateName.end());

    std::string isPlaneStr;
    bool isPlane;
    std::cout << "Do you have a plane ?" << std::endl;
    std::cin >> isPlaneStr;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    isPlane = !(isPlaneStr == "no" || isPlaneStr == "No" || isPlaneStr == "n" || isPlaneStr == "N" || isPlaneStr == "0");

    if (isPlane) {
        XplaneFederateHla1516e federate;
        federate.connect(L"Federation", wideFederateName, L"isae_prise_hla1516e.xml");
        federate.initializationFederate();
        launch(federate);
    } else {
        XplaneFederateHla1516e federate;
        federate.connect(L"Federation", wideFederateName, L"isae_prise_hla1516e.xml");
        federate.initializationFederate();
        std::string command("");
        double latitude{0.};
        double longitude{0.};
        double altitude{0.};
        std::string direction("");
        int nbCommand{0};

        bool continuing = true;

        while (continuing) {
            std::cout << "Enter a command ('help' to print the available commands)" << std::endl;
            std::cin >> command;
            if (command == "help" || command == "h") {
                // Print help
                std::cout << "command possibles:" << std::endl;
                std::cout << "    h, help                        : Print this help." << std::endl;
                std::cout << "    s, set planeName lat lon alt   : set the position of the plane selected by its name (* to force all planes position)." << std::endl;
                std::cout << "    p, print                       : Print the plane names and thieir positions." << std::endl;
                std::cout << "    t, tick                        : send a tick to advance in time." << std::endl;
                std::cout << "    q, quit                        : Quit the application." << std::endl;
                std::cout << "    o, observe                     : Switch to observer mode. Can not come back." << std::endl;
                std::cout << "    c, command planeName direction nbCommand  : Command a plane with a direction ( PU => pitch up, PD => pitch down, RL => roll left, RR => roll right, YL => Yaw left, YR => Yaw right." << std::endl;
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            } else if (command == "s" || command == "set") {

                std::cin >> command;
                std::cin >> latitude;
                std::cin >> longitude;
                std::cin >> altitude;
                std::wstring planeName(command.begin(), command.end());
                federate.sendInteractionForcePosition(latitude, longitude, altitude);
                federate.runOneStep();
//                if(planeName == L"*")
//                    f.forceAllOtherPlanesPosition(posX, posY, posZ);
//                else
//                    f.forcePositionOtherPlane(planeName, posX, posY, posZ);

//                if(f.isManualTimeManagement())
//                {
//                    f.sendInteractionForcePosition();
//                    f.step();
//                }
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            } else if (command == "c" || command == "command") {

                std::cin >> command;
                std::cin >> direction;
                std::cin >> nbCommand;

//                std::cout << command << "\n"
//                          << direction << "\n"
//                          << nbCommand << "\n";

                std::wstring planeName(command.begin(), command.end());

                int cptCommand = 0;
//                while(cptCommand < nbCommand)
//                {
//                    cptCommand++;
//                    if(direction == "PU")
//                        f.setCommands(planeName, PlaneInterface::COMMAND::PITCH_UP);
//                    else if(direction == "PD")
//                        f.setCommands(planeName, PlaneInterface::COMMAND::PITCH_DOWN);
//                    else if(direction == "RL")
//                        f.setCommands(planeName, PlaneInterface::COMMAND::ROLL_LEFT);
//                    else if(direction == "RR")
//                        f.setCommands(planeName, PlaneInterface::COMMAND::ROLL_RIGHT);
//                    else if(direction == "YL")
//                        f.setCommands(planeName, PlaneInterface::COMMAND::YAW_LEFT);
//                    else if(direction == "YR")
//                        f.setCommands(planeName, PlaneInterface::COMMAND::YAW_RIGHT);

//                    if(f.isManualTimeManagement())
//                    {
//                        f.sendInteractionCommand();
//                        f.step();
//                    }
//                }
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            } else if (command == "print" || command == "p") {
                federate.runOneStep();
                federate.printOtherPlanes();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            } else if (command == "tick" || command == "t") {
                federate.runOneStep();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            } else if (command == "quit" || command == "q") {
                continuing = false;
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            } else if (command == "o" || command == "observe") {
                while (true) {
                    std::cout << "\r\e[K" << std::flush;
                    for (int i = 0; i < 40; i++) {
                        std::cout << "\e[A";
                        std::cout << "\r\e[K" << std::flush;
                        // If there is time management enabled, you have advance to manually
                        federate.runOneStep();
                        federate.printOtherPlanes();
                    }

                    std::this_thread::sleep_for(0.1s);
                }
            }

        }
    }

    return 0;
}
