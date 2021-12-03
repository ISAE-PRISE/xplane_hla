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


#ifndef __REMOTE_AIRCRAFT_TO_DRAW_HH__
#define __REMOTE_AIRCRAFT_TO_DRAW_HH__

#include <RTI/encoding/HLAfixedRecord.h>
#include <RTI/encoding/HLAfixedArray.h>
#include <RTI/encoding/BasicDataElements.h>
#include <RTI/encoding/EncodingExceptions.h>

#include <string.h>
#include <math.h>
#include <iostream>
#include <cstdlib>
#include <map>
#ifdef XPLANE_PLUGIN
#include "XPLMPlanes.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMGraphics.h"
#endif

using rti1516e::HLAfloat32LE;
using rti1516e::HLAfloat64LE;
using rti1516e::HLAfixedArray;

const	double	kMaxPlaneDistance = 5280.0 / 3.2 * 10.0;
const	double	kFullPlaneDist = 5280.0 / 3.2 * 3.0;

const int kNbGearDeploy = 5;
const int kNbThrottle = 8;

//namespace AircraftModelsFile {
//    const std::string AEROLITE_103 = "Aerolite_103.acf";
//    const std::string ASK_21 = "ASK21.acf";
//    const std::string BARON_B58 = "Baron_58.acf";
//    const std::string B737 = "b738.acf";
//    const std::string B747 = "747-400.acf";
//    const std::string CESSNA_172SP = "Cessna_172SP.acf";
//    const std::string CIRRUS_SF50 = "CirrusSF50";
//}

//namespace AircraftModelsPaths {
//    const std::string AEROLITE_103 = "Aerolite 103/";
//    const std::string ASK_21 = "Ask 21/";
//    const std::string BARON_B58 = "Baron B58/";
//    const std::string B737 = "Boeing B737-800/";
//    const std::string B747 = "Boeing B747-400/";
//    const std::string CESSNA_172SP = "Cessna 172SP/";
//    const std::string CIRRUS_SF50 = "Cirrus SF-50/";
//}

// Aircraft class, allows access to an AI aircraft
// It's an header only class because Xplane can't find the implementation of methods
class RemoteAircraftToDraw
{
	private:
#ifdef XPLANE_PLUGIN
		XPLMDataRef	_dr_plane_x;
		XPLMDataRef	_dr_plane_y;
		XPLMDataRef	_dr_plane_z;
		XPLMDataRef	_dr_plane_the;
		XPLMDataRef	_dr_plane_phi;
		XPLMDataRef	_dr_plane_psi;
		XPLMDataRef	_dr_plane_gear_deploy;
		XPLMDataRef	_dr_plane_throttle;
		XPLMDataRef	_dr_plane_lat;
		XPLMDataRef	_dr_plane_lon;
		XPLMDataRef	_dr_plane_el;
        XPLMDataRef	_dr_plane_vx;
        XPLMDataRef	_dr_plane_vy;
        XPLMDataRef	_dr_plane_vz;
#endif
	public:
        int         aircraftNo = 0;
        double		plane_x = .0;
        double		plane_y = .0;
        double		plane_z = .0;
        float		plane_the = .0f;
        float		plane_phi = .0f;
        float		plane_psi = .0f;
        float		plane_gear_deploy[kNbGearDeploy] = {.0f};
        float		plane_throttle[kNbThrottle] = {.0f};
        double      plane_lat = .0;
        double      plane_lon = .0;
        double      plane_el = .0;
        float       plane_vx = .0;
        float       plane_vy = .0;
        float       plane_vz = .0;
        std::string aircraft_type="";
        std::string aircraft_name="";
        std::map<std::string, std::string> modelsPath;
		
		// constructor & destructor
        RemoteAircraftToDraw()
        {
#ifdef XPLANE_PLUGIN
            _dr_plane_x = NULL;
            _dr_plane_y = NULL;
            _dr_plane_z = NULL;
            _dr_plane_the = NULL;
            _dr_plane_phi = NULL;
            _dr_plane_psi = NULL;
            _dr_plane_gear_deploy = NULL;
            _dr_plane_throttle = NULL;
            _dr_plane_lat = NULL;
            _dr_plane_lon = NULL;
            _dr_plane_el = NULL;
            _dr_plane_vx = NULL;
            _dr_plane_vy = NULL;
            _dr_plane_vz = NULL;
#endif
//            initModelsPath();
        }

//        void initModelsPath()
//        {
//            if(getenv("PLUGIN_PATH"))
//            {
//                const std::string AIRCRAFT_SUBPATH = std::string(getenv("PLUGIN_PATH")) + "/../../Aircraft/Laminar Research/";
//                modelsPath[AircraftModelsFile::AEROLITE_103] = AIRCRAFT_SUBPATH + AircraftModelsPaths::AEROLITE_103;
//                modelsPath[AircraftModelsFile::ASK_21] = AIRCRAFT_SUBPATH + AircraftModelsPaths::ASK_21;
//                modelsPath[AircraftModelsFile::B737] = AIRCRAFT_SUBPATH + AircraftModelsPaths::B737;
//                modelsPath[AircraftModelsFile::B747] = AIRCRAFT_SUBPATH + AircraftModelsPaths::B747;
//                modelsPath[AircraftModelsFile::BARON_B58] = AIRCRAFT_SUBPATH + AircraftModelsPaths::BARON_B58;
//                modelsPath[AircraftModelsFile::CESSNA_172SP] = AIRCRAFT_SUBPATH + AircraftModelsPaths::CESSNA_172SP;
//                modelsPath[AircraftModelsFile::CIRRUS_SF50] = AIRCRAFT_SUBPATH + AircraftModelsPaths::CIRRUS_SF50;
//            }
//        }

        virtual ~RemoteAircraftToDraw()
        {

        }

        // construct function to set proper datarefs
        void ContructRemoteAircraftToDraw(int AircraftNo, const std::string& AircraftType, const std::string& AircraftName)
        {
            aircraftNo = AircraftNo;
            aircraft_type = AircraftType;
            aircraft_name = AircraftName;

            char	x_str[80];
            char	y_str[80];
            char	z_str[80];
            char	the_str[80];
            char	phi_str[80];
            char	psi_str[80];
            char	gear_deploy_str[80];
            char	throttle_str[80];
            char    lat_str[80];
            char    lon_str[80];
            char    el_str[80];
            char    vx_str[80];
            char    vy_str[80];
            char    vz_str[80];
#ifdef XPLANE_PLUGIN
            strcpy(x_str, "sim/multiplayer/position/planeX_x");
            strcpy(y_str,	"sim/multiplayer/position/planeX_y");
            strcpy(z_str,	"sim/multiplayer/position/planeX_z");
            strcpy(the_str,	"sim/multiplayer/position/planeX_the");
            strcpy(phi_str,	"sim/multiplayer/position/planeX_phi");
            strcpy(psi_str,	"sim/multiplayer/position/planeX_psi");
            strcpy(gear_deploy_str,	"sim/multiplayer/position/planeX_gear_deploy");
            strcpy(throttle_str, "sim/multiplayer/position/planeX_throttle");
            strcpy(lat_str,	"sim/multiplayer/position/planeX_lat");
            strcpy(lon_str,	"sim/multiplayer/position/planeX_lon");
            strcpy(el_str,	"sim/multiplayer/position/planeX_el");
            strcpy(vx_str,	"sim/multiplayer/position/planeX_v_x");
            strcpy(vy_str,	"sim/multiplayer/position/planeX_v_y");
            strcpy(vz_str,	"sim/multiplayer/position/planeX_v_z");

            char cTemp = (aircraftNo + 0x30);
            x_str[30]			=	cTemp;
            y_str[30]			=	cTemp;
            z_str[30]			=	cTemp;
            the_str[30]			=	cTemp;
            phi_str[30]			=	cTemp;
            psi_str[30]			=	cTemp;
            gear_deploy_str[30] =	cTemp;
            throttle_str[30]	=	cTemp;
            lat_str[30]			=	cTemp;
            lon_str[30]			=	cTemp;
            el_str[30]			=	cTemp;
            vx_str[30]			=	cTemp;
            vy_str[30]			=	cTemp;
            vz_str[30]			=	cTemp;

            _dr_plane_x				= XPLMFindDataRef(x_str);
            _dr_plane_y				= XPLMFindDataRef(y_str);
            _dr_plane_z				= XPLMFindDataRef(z_str);
            _dr_plane_the			= XPLMFindDataRef(the_str);
            _dr_plane_phi			= XPLMFindDataRef(phi_str);
            _dr_plane_psi			= XPLMFindDataRef(psi_str);
            _dr_plane_gear_deploy	= XPLMFindDataRef(gear_deploy_str);
            _dr_plane_throttle		= XPLMFindDataRef(throttle_str);
            _dr_plane_lat			= XPLMFindDataRef(lat_str);
            _dr_plane_lon			= XPLMFindDataRef(lon_str);
            _dr_plane_el			= XPLMFindDataRef(el_str);
            _dr_plane_vx            = XPLMFindDataRef(vx_str);
            _dr_plane_vy            = XPLMFindDataRef(vy_str);
            _dr_plane_vz            = XPLMFindDataRef(vz_str);

//            std::string path_plane = modelsPath[aircraft_type] + aircraft_type;
//            std::cout << "Load " << path_plane << " for aircraft n° " << aircraftNo << " and type " << aircraft_type << std::endl;
//            XPLMSetAircraftModel(aircraftNo, path_plane.c_str());

#endif
        }

		// functions
        void GetAircraftData(void)
        {
#ifdef XPLANE_PLUGIN
            plane_x = XPLMGetDatad(_dr_plane_x);
            plane_y = XPLMGetDatad(_dr_plane_y);
            plane_z = XPLMGetDatad(_dr_plane_z);
            plane_the = XPLMGetDataf(_dr_plane_the);
            plane_phi = XPLMGetDataf(_dr_plane_phi);
            plane_psi = XPLMGetDataf(_dr_plane_psi);
            XPLMGetDatavf(_dr_plane_gear_deploy, plane_gear_deploy, 0, 5);
            XPLMGetDatavf(_dr_plane_throttle, plane_throttle, 0, 8);
            plane_lat = XPLMGetDatad(_dr_plane_lat);
            plane_lon = XPLMGetDatad(_dr_plane_lon);
            plane_el = XPLMGetDatad(_dr_plane_el);
            plane_vx = XPLMGetDataf(_dr_plane_vx);
            plane_vy = XPLMGetDataf(_dr_plane_vy);
            plane_vz = XPLMGetDataf(_dr_plane_vz);
#endif
        }

        void SetAircraftData(void)
        {
#ifdef XPLANE_PLUGIN
            XPLMSetDatad(_dr_plane_x, plane_x);
            XPLMSetDatad(_dr_plane_y, plane_y);
            XPLMSetDatad(_dr_plane_z, plane_z);
            XPLMSetDataf(_dr_plane_the, plane_the);
            XPLMSetDataf(_dr_plane_phi, plane_phi);
            XPLMSetDataf(_dr_plane_psi, plane_psi);
            XPLMSetDatavf(_dr_plane_gear_deploy, plane_gear_deploy, 0, 5);
            XPLMSetDatavf(_dr_plane_throttle, plane_throttle, 0, 8);
            XPLMSetDataf(_dr_plane_vx, plane_vx);
            XPLMSetDataf(_dr_plane_vy, plane_vy);
            XPLMSetDataf(_dr_plane_vz, plane_vz);
            //        XPLMSetDatad(_dr_plane_lat, plane_lat);
            //        XPLMSetDatad(_dr_plane_lon, plane_lon);
            //        XPLMSetDatad(_dr_plane_el, plane_el);
#endif
        }

        void RefreshFromHLAFlightDynamicsModelData(const rti1516e::HLAfixedRecord &recordFDM)
        {
        #ifdef DEBUG_HLA_XPLANE_REFLECT
            std::wcout << L"RemoteAircraftToDraw.cc -> RefreshFromHlaFixedRecord(): Start" << std::endl;
        #endif

            HLAfloat64LE latitutdeDeg = static_cast<const HLAfloat64LE&>(recordFDM.get(0));
            HLAfloat64LE longitudeDeg = static_cast<const HLAfloat64LE&>(recordFDM.get(1));
            HLAfloat64LE altitudeMSLMeters = static_cast<const HLAfloat64LE&>(recordFDM.get(2));
            HLAfloat32LE phiDeg = static_cast<const HLAfloat32LE&>(recordFDM.get(4));
            HLAfloat32LE thetaDeg = static_cast<const HLAfloat32LE&>(recordFDM.get(5));
            HLAfloat32LE psiDeg = static_cast<const HLAfloat32LE&>(recordFDM.get(6));
            HLAfloat32LE velocityX = static_cast<const HLAfloat32LE&>(recordFDM.get(23));
            HLAfloat32LE velocityY = static_cast<const HLAfloat32LE&>(recordFDM.get(24));
            HLAfloat32LE velocityZ = static_cast<const HLAfloat32LE&>(recordFDM.get(25));


            double x = .0;
            double y = .0;
            double z = .0;
#ifdef XPLANE_PLUGIN
            XPLMWorldToLocal(latitutdeDeg, longitudeDeg, altitudeMSLMeters, &x, &y, &z);
#else
            HLAfloat64LE local_x = static_cast<const HLAfloat64LE&>(recordFDM.get(20));
            HLAfloat64LE local_y = static_cast<const HLAfloat64LE&>(recordFDM.get(21));
            HLAfloat64LE local_z = static_cast<const HLAfloat64LE&>(recordFDM.get(22));
            x = local_x.get();
            y = local_y.get();
            z = local_z.get();
        #endif
            plane_x = x;
            plane_y = y;
            plane_z = z;
            plane_el = altitudeMSLMeters;
            plane_lat = latitutdeDeg;
            plane_lon = longitudeDeg;
            plane_phi = phiDeg;
            plane_the = thetaDeg;
            plane_psi = psiDeg;
            plane_vx = velocityX;
            plane_vy = velocityY;
            plane_vz = velocityZ;

            SetAircraftData();

        #ifdef DEBUG_HLA_XPLANE_REFLECT
            std::wcout << L"RemoteAircraftToDraw.cc -> RefreshFromHlaFixedRecord(): End" << std::endl;
        #endif
        }

        void RefreshFromHLAEngineActuatorsModelData(const rti1516e::HLAfixedRecord &recordEAM)
        {
        #ifdef DEBUG_HLA_XPLANE_REFLECT
            std::wcout << L"RemoteAircraftToDraw.cc -> RefreshFromHLAEngineActuatorsModelData(): Start" << std::endl;
        #endif
        #ifndef CLIENT_TEST

            HLAfixedArray arrayThrottle = static_cast<const HLAfixedArray&>(recordEAM.get(0));

            if(arrayThrottle.size() != kNbThrottle)
                throw rti1516e::EncoderException(L"The arrayThrottle is : " + std::to_wstring(arrayThrottle.size())
                                                 + L" instead of " + std::to_wstring(kNbThrottle));

            for(unsigned int i=0; i < kNbThrottle; i++)
            {
                 plane_throttle[i] = static_cast<const HLAfloat32LE&>(arrayThrottle.get(i));
            }

            SetAircraftData();

        #endif
        #ifdef DEBUG_HLA_XPLANE_REFLECT
            std::wcout << L"RemoteAircraftToDraw.cc -> RefreshFromHLAEngineActuatorsModelData(): End" << std::endl;
        #endif
        }

        void print()
        {
        #ifdef DEBUG_HLA_XPLANE_REFLECT
            std::wcout << L"RemoteAircraftToDraw.cc -> print(): Start" << std::endl;
        #endif
            std::cout << "Aircraft type: \t\t" << aircraft_type << std::endl;
            std::wcout << L"aircraftNo : \t\t" << aircraftNo << std::endl;
            std::wcout << L"X : \t\t" << plane_x << std::endl;
            std::wcout << L"Y : \t\t" << plane_y << std::endl;
            std::wcout << L"Z : \t\t" << plane_z << std::endl;
            std::wcout << L"phi : \t\t" << plane_phi << std::endl;
            std::wcout << L"the : \t\t" << plane_the << std::endl;
            std::wcout << L"psi : \t\t" << plane_psi << std::endl;
            std::wcout << L"vx : \t\t" << plane_vx << std::endl;
            std::wcout << L"vy : \t\t" << plane_vy << std::endl;
            std::wcout << L"vz : \t\t" << plane_vz << std::endl;
            std::wcout << L"gear_deploy : \t\t";
            for(int i=0; i<kNbGearDeploy; i++)
                std::wcout << plane_gear_deploy[i] << " ";
            std::wcout << std::endl;
            std::wcout << L"throttle : \t\t";
            for(int i=0; i<kNbThrottle; i++)
                std::wcout << plane_throttle[i] << " ";
            std::wcout << std::endl;
        #ifdef DEBUG_HLA_XPLANE_REFLECT
            std::wcout << L"RemoteAircraftToDraw.cc -> print(): End" << std::endl;
        #endif
        }
};

#endif
