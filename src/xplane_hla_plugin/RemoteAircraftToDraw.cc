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


//#include "RemoteAircraftToDraw.hh"

//#include <string.h>
//#include <math.h>
//#include "XPLMPlanes.h"
//#include "XPLMDataAccess.h"
//#include "XPLMProcessing.h"
//#include "XPLMGraphics.h"

//using rti1516e::HLAfloat32LE;
//using rti1516e::HLAfloat64LE;

//// ----------------------------------------------------------------------------
//// Constructor
//RemoteAircraftToDraw::RemoteAircraftToDraw()
//{
//    _dr_plane_x = NULL;
//    _dr_plane_y = NULL;
//    _dr_plane_z = NULL;
//    _dr_plane_the = NULL;
//    _dr_plane_phi = NULL;
//    _dr_plane_psi = NULL;
//    _dr_plane_gear_deploy = NULL;
//    _dr_plane_throttle = NULL;
//    _dr_plane_lat = NULL;
//    _dr_plane_lon = NULL;
//    _dr_plane_el = NULL;
//}

//// ----------------------------------------------------------------------------
//// Constructor
//void RemoteAircraftToDraw::ContructRemoteAircraftToDraw(int AircraftNo)
//{
//    char	x_str[80];
//    char	y_str[80];
//    char	z_str[80];
//    char	the_str[80];
//    char	phi_str[80];
//    char	psi_str[80];
//    char	gear_deploy_str[80];
//    char	throttle_str[80];
//    char    lat_str[80];
//    char    lon_str[80];
//    char    el_str[80];

//    strcpy(x_str, "sim/multiplayer/position/planeX_x");
//    strcpy(y_str,	"sim/multiplayer/position/planeX_y");
//    strcpy(z_str,	"sim/multiplayer/position/planeX_z");
//    strcpy(the_str,	"sim/multiplayer/position/planeX_the");
//    strcpy(phi_str,	"sim/multiplayer/position/planeX_phi");
//    strcpy(psi_str,	"sim/multiplayer/position/planeX_psi");
//    strcpy(gear_deploy_str,	"sim/multiplayer/position/planeX_gear_deploy");
//    strcpy(throttle_str, "sim/multiplayer/position/planeX_throttle");
//    strcpy(lat_str,	"sim/multiplayer/position/planeX_lat");
//    strcpy(lon_str,	"sim/multiplayer/position/planeX_lon");
//    strcpy(el_str,	"sim/multiplayer/position/planeX_el");

//    char cTemp = (AircraftNo + 0x30);
//    x_str[30]			=	cTemp;
//    y_str[30]			=	cTemp;
//    z_str[30]			=	cTemp;
//    the_str[30]			=	cTemp;
//    phi_str[30]			=	cTemp;
//    psi_str[30]			=	cTemp;
//    gear_deploy_str[30] =	cTemp;
//    throttle_str[30]	=	cTemp;
//    lat_str[30]			=	cTemp;
//    lon_str[30]			=	cTemp;
//    el_str[30]			=	cTemp;

//    _dr_plane_x				= XPLMFindDataRef(x_str);
//    _dr_plane_y				= XPLMFindDataRef(y_str);
//    _dr_plane_z				= XPLMFindDataRef(z_str);
//    _dr_plane_the			= XPLMFindDataRef(the_str);
//    _dr_plane_phi			= XPLMFindDataRef(phi_str);
//    _dr_plane_psi			= XPLMFindDataRef(psi_str);
//    _dr_plane_gear_deploy	= XPLMFindDataRef(gear_deploy_str);
//    _dr_plane_throttle		= XPLMFindDataRef(throttle_str);
//    _dr_plane_lat			= XPLMFindDataRef(lat_str);
//    _dr_plane_lon			= XPLMFindDataRef(lon_str);
//    _dr_plane_el			= XPLMFindDataRef(el_str);
//}

//// ----------------------------------------------------------------------------
////
//void RemoteAircraftToDraw::GetAircraftData(void)
//{
//    plane_x = XPLMGetDatad(_dr_plane_x);
//    plane_y = XPLMGetDatad(_dr_plane_y);
//    plane_z = XPLMGetDatad(_dr_plane_z);
//    plane_the = XPLMGetDataf(_dr_plane_the);
//    plane_phi = XPLMGetDataf(_dr_plane_phi);
//    plane_psi = XPLMGetDataf(_dr_plane_psi);
//    XPLMGetDatavf(_dr_plane_gear_deploy, plane_gear_deploy, 0, 5);
//    XPLMGetDatavf(_dr_plane_throttle, plane_throttle, 0, 8);
//    plane_lat = XPLMGetDatad(_dr_plane_lat);
//    plane_lon = XPLMGetDatad(_dr_plane_lon);
//    plane_el = XPLMGetDatad(_dr_plane_el);
//}

//// ----------------------------------------------------------------------------
////
//void RemoteAircraftToDraw::SetAircraftData(void)
//{
//    XPLMSetDatad(_dr_plane_x, plane_x);
//    XPLMSetDatad(_dr_plane_y, plane_y);
//    XPLMSetDatad(_dr_plane_z, plane_z);
//    XPLMSetDataf(_dr_plane_the, plane_the);
//    XPLMSetDataf(_dr_plane_phi, plane_phi);
//    XPLMSetDataf(_dr_plane_psi, plane_psi);
//    XPLMSetDatavf(_dr_plane_gear_deploy, plane_gear_deploy, 0, 5);
//    XPLMSetDatavf(_dr_plane_throttle, plane_throttle, 0, 8);
//    //        XPLMSetDatad(_dr_plane_lat, plane_lat);
//    //        XPLMSetDatad(_dr_plane_lon, plane_lon);
//    //        XPLMSetDatad(_dr_plane_el, plane_el);
//}

//void RemoteAircraftToDraw::RefreshFromHLAFlightDynamicsModelData(const rti1516e::HLAfixedRecord &recordFDM)
//{
//#ifdef DEBUG_HLA_XPLANE_REFLECT
//    std::wcout << L"RemoteAircraftToDraw.cc -> RefreshFromHlaFixedRecord(): Start" << std::endl;
//#endif

//    HLAfloat64LE latitutdeDeg = static_cast<const HLAfloat64LE&>(recordFDM.get(0));
//    HLAfloat64LE longitudeDeg = static_cast<const HLAfloat64LE&>(recordFDM.get(1));
//    HLAfloat64LE altitudeMSLMeters = static_cast<const HLAfloat64LE&>(recordFDM.get(2));
//    HLAfloat32LE phiDeg = static_cast<const HLAfloat32LE&>(recordFDM.get(4));
//    HLAfloat32LE thetaDeg = static_cast<const HLAfloat32LE&>(recordFDM.get(5));
//    HLAfloat32LE psiDeg = static_cast<const HLAfloat32LE&>(recordFDM.get(6));

//    double x = .0;
//    double y = .0;
//    double z = .0;

//    XPLMWorldToLocal(latitutdeDeg, longitudeDeg, altitudeMSLMeters, &x, &y, &z);

//    plane_x = x;
//    plane_y = y;
//    plane_z = z;
//    plane_phi = phiDeg;
//    plane_the = thetaDeg;
//    plane_psi = psiDeg;

//    SetAircraftData();

//#ifdef DEBUG_HLA_XPLANE_REFLECT
//    std::wcout << L"RemoteAircraftToDraw.cc -> RefreshFromHlaFixedRecord(): End" << std::endl;
//#endif
//}

