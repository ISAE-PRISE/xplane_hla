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

//#define DEBUG_HLA_XPLANE_TIME_STEP
#define DEBUG_HLA_XPLANE_FED
//#define DEBUG_HLA_XPLANE_REFLECT
#if defined(XPLANE_PLUGIN) && !defined(XPLM301) 
    #error This is made to be compiled against the XPLM301 SDK
#endif

//standard includes
#include <chrono>
#if defined(WIN32) || defined(WIN64)
// Ajout de la librairie math pour la définition de M_PI
#define _USE_MATH_DEFINES
#include <cmath>
#endif

// Local includes
#include <XplaneFederateHla1516e.hh>

#include <RTI/encoding/HLAvariableArray.h>

using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

// ----------------------------------------------------------------------------
// Constructor

XplaneFederateHla1516e::XplaneFederateHla1516e()
											  : rti1516e::NullFederateAmbassador()
                                              , _RtiAmb(0), _FederateHandle(), _TimeStep(0.0), _Lookahead(0.0), _RestOfTimeStep(0.0), _LocalTime(0.0), _SimulationEndTime(0.0)
{
    #ifdef DEBUG_HLA_XPLANE_FED
	std::wcout << L"HlaXplaneFederate1516e.cc -> Constructor(): Start" << std::endl;
	#endif
    _FederationName = L"";
    _FederateName = L"";
    _FomFileName = L"";
	_TimeStep = 0.0;
	_Lookahead = 0.0;
	_RestOfTimeStep = 0.0;
	_LocalTime = 0.0;
	_SimulationEndTime = 0000.0;
	_IsTimeReg = _IsTimeConst = _IsTimeAdvanceGrant = _IsSyncAnnonced=  false ;
	_IsCreator = true;
	_SyncRegSuccess = _SyncRegFailed = _InPause = false ;
	_IsOutMesTimespamped = true;
	
	_DiscovObjectInstanceHandleForClassControlCommandModel = false;
	_NewAttributeReceivedForAttrControlCommandModelData = false;
	
	std::string testTag ("test tag");
	_MyTag.setData (testTag.c_str (), testTag.size () + 1);
	
	std::string testSyncTag ("");
	_MySyncTag.setData (testSyncTag.c_str (), testSyncTag.size () + 1);
	
    initializationRecords();
  
  	#ifdef DEBUG_HLA_XPLANE_FED
	std::wcout << L"HlaXplaneFederate1516e.cc -> Constructor(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Destructor
XplaneFederateHla1516e::~XplaneFederateHla1516e()
{
}

void XplaneFederateHla1516e::disableFlightControl()
{
#ifdef XPLANE_PLUGIN
    XPLMSetDatai(_XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrOverRideFlightControl], 1);
#endif
}

void XplaneFederateHla1516e::enableFlightControl()
{
#ifdef XPLANE_PLUGIN
    XPLMSetDatai(_XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrOverRideFlightControl], 0);
#endif
}

void XplaneFederateHla1516e::disableJoystick()
{
#ifdef XPLANE_PLUGIN
    XPLMSetDatai(_XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrOverRideJoystick], 1);
#endif
}

void XplaneFederateHla1516e::enableJoystick()
{
#ifdef XPLANE_PLUGIN
    XPLMSetDatai(_XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrOverRideJoystick], 0);
#endif
}

/**
 * @brief XplaneFederateHla1516e::getLocalHLAFlightDynamicsModelData
 * @return a reference to _LocalHLAFlightDynamicsModelData which store all flights dynamic data
 */
const HLAfixedRecord& XplaneFederateHla1516e::getLocalHLAFlightDynamicsModelData() const
{
    return _LocalHLAFlightDynamicsModelData;
}

/**
 * @brief XplaneFederateHla1516e::getLocalHLAHydraulicActuatorsModelData
 * @return a reference to _LocalHLAHydraulicActuatorsModelData which store all Hydraulic actuators data
 */
const HLAfixedRecord& XplaneFederateHla1516e::getLocalHLAHydraulicActuatorsModelData() const
{
    return _LocalHLAHydraulicActuatorsModelData;
}

/**
 * @brief XplaneFederateHla1516e::getLocalHLAEngineActuatorsModelData
 * @return a reference to _LocalHLAEngineActuatorsModelData which store all Engine actuators data
 */
const HLAfixedRecord& XplaneFederateHla1516e::getLocalHLAEngineActuatorsModelData() const
{
    return _LocalHLAEngineActuatorsModelData;
}

/**
 * @brief XplaneFederateHla1516e::getLocalHLATimeInformationModelData
 * @return a reference to _LocalHLATimeInformationModelData which store all Simulator Time information data
 */
const HLAfixedRecord& XplaneFederateHla1516e::getLocalHLATimeInformationModelData() const
{
    return _LocalHLATimeInformationModelData;
}

// ----------------------------------------------------------------------------
// Initilize all HLAFixedStruct
/**
 * @brief XplaneFederateHla1516e::initHLAFlightDynamicsModelData
 *          Initilisation of the attributes which will store flight dynamics data.
 *          There is one attribute to store local flight dynmaics data (Own plane) and
 *          one attribute to store a remote flight dynamics data (Other plane) just the time of the reception.
 * @param remote To choose if we initialize remote position or local Flight dynamics Data
 */
void XplaneFederateHla1516e::initHLAFlightDynamicsModelData(bool remote)
{
#ifdef DEBUG_HLA_XPLANE_FED
std::wcout << L"HlaXplaneFederate1516e.cc -> initHLAFlightDynamicsModelData(): Start" << std::endl;
#endif

    HLAfixedRecord *hlaFlightDynamicsModelData;
    if(remote)
        hlaFlightDynamicsModelData = &_RemoteHLAFlightDynamicsModelData;
    else
        hlaFlightDynamicsModelData = &_LocalHLAFlightDynamicsModelData;

    hlaFlightDynamicsModelData->appendElement(HLAfloat64LE(.0));
    hlaFlightDynamicsModelData->appendElement(HLAfloat64LE(.0));
    hlaFlightDynamicsModelData->appendElement(HLAfloat64LE(.0));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));

    hlaFlightDynamicsModelData->appendElement(HLAfloat64LE(.0));
    hlaFlightDynamicsModelData->appendElement(HLAfloat64LE(.0));
    hlaFlightDynamicsModelData->appendElement(HLAfloat64LE(.0));

    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));
    hlaFlightDynamicsModelData->appendElement(HLAfloat32LE(.0f));

#ifdef DEBUG_HLA_XPLANE_FED
std::wcout << L"HlaXplaneFederate1516e.cc -> initHLAFlightDynamicsModelData(): End" << std::endl;
#endif
}

/**
 * @brief XplaneFederateHla1516e::initHLAHydraulicActuatorsModelData
 *          Initilisation of the attributes which will store hydraulic actuator data.
 *          There is one attribute to store local hydraulic actuators data (Own plane) and
 *          one attribute to store a remote hydraulic actuators data (Other plane) just the time of the reception.
 * @param remote To choose if we initialize remote position or local hydraulic actuators Data
 */
void XplaneFederateHla1516e::initHLAHydraulicActuatorsModelData(bool remote)
{
#ifdef DEBUG_HLA_XPLANE_FED
std::wcout << L"HlaXplaneFederate1516e.cc -> initHLAHydraulicActuatorsModelData(): Start" << std::endl;
#endif

    HLAfixedRecord *hlaHydraulicActuatorsModelData;
    if(remote)
        hlaHydraulicActuatorsModelData = &_RemoteHLAHydraulicActuatorsModelData;
    else
        hlaHydraulicActuatorsModelData = &_LocalHLAHydraulicActuatorsModelData;

    HLAfloat32LE LeftAileronEffectiveDeflectionDegrees(.0f);
    HLAfloat32LE RightAileronEffectiveDeflectionDegrees(.0f);
    HLAfloat32LE LeftElevatorEffectiveDeflectionDegrees(.0f);
    HLAfloat32LE RightElevatorEffectiveDeflectionDegrees(.0f);
    HLAfloat32LE HorizontalStabilizerEffectiveDeflectionDegrees(.0f);
    HLAfloat32LE FlapsEffectiveDeployment(.0f);
    HLAfloat32LE RudderEffectiveDeflectionDegrees(.0f);
    HLAfloat32LE LeftSpeedbrakesEffectiveDeflectionDegrees(.0f);
    HLAfloat32LE RightSpeedbrakesEffectiveDeflectionDegrees(.0f);
    HLAfloat32LE LeftSpoilersEffectiveDeflectionDegrees(.0f);
    HLAfloat32LE RightSpoilersEffectiveDeflectionDegrees(.0f);

    hlaHydraulicActuatorsModelData->appendElement(LeftAileronEffectiveDeflectionDegrees);
    hlaHydraulicActuatorsModelData->appendElement(RightAileronEffectiveDeflectionDegrees);
    hlaHydraulicActuatorsModelData->appendElement(LeftElevatorEffectiveDeflectionDegrees);
    hlaHydraulicActuatorsModelData->appendElement(RightElevatorEffectiveDeflectionDegrees);
    hlaHydraulicActuatorsModelData->appendElement(HorizontalStabilizerEffectiveDeflectionDegrees);
    hlaHydraulicActuatorsModelData->appendElement(FlapsEffectiveDeployment);
    hlaHydraulicActuatorsModelData->appendElement(RudderEffectiveDeflectionDegrees);
    hlaHydraulicActuatorsModelData->appendElement(LeftSpeedbrakesEffectiveDeflectionDegrees);
    hlaHydraulicActuatorsModelData->appendElement(RightSpeedbrakesEffectiveDeflectionDegrees);
    hlaHydraulicActuatorsModelData->appendElement(LeftSpoilersEffectiveDeflectionDegrees);
    hlaHydraulicActuatorsModelData->appendElement(RightSpoilersEffectiveDeflectionDegrees);

    // FF
    HLAfloat32LE prototype(0.0f);
    HLAfixedArray arrayGEARS(prototype, 10);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayGEARS.set(i, HLAfloat32LE(prototype));
    }
    hlaHydraulicActuatorsModelData->appendElement(arrayGEARS);

#ifdef DEBUG_HLA_XPLANE_FED
std::wcout << L"HlaXplaneFederate1516e.cc -> initHLAHydraulicActuatorsModelData(): End" << std::endl;
#endif
}

/**
 * @brief XplaneFederateHla1516e::initHLAEngineActuatorsModelData
 *          Initilisation of the attributes which will store engine actuator data.
 *          There is one attribute to store local engine actuators data (Own plane) and
 *          one attribute to store a remote engine actuators data (Other plane) just the time of the reception.
 * @param remote To choose if we initialize remote position or local engine actuators Data
 */
void XplaneFederateHla1516e::initHLAEngineActuatorsModelData(bool remote)
{
#ifdef DEBUG_HLA_XPLANE_FED
std::wcout << L"HlaXplaneFederate1516e.cc -> initHLAEngineActuatorsModelData(): Start" << std::endl;
#endif

    HLAfixedRecord *hlaEngineActuatorsModelData;
    if(remote)
        hlaEngineActuatorsModelData = &_RemoteHLAEngineActuatorsModelData;
    else
        hlaEngineActuatorsModelData = &_LocalHLAEngineActuatorsModelData;

    // THrottle
    HLAfloat32LE prototype(0.0f);
    HLAfixedArray arrayThrottle(prototype, 8);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayThrottle.set(i, prototype);
    }
    hlaEngineActuatorsModelData->appendElement(arrayThrottle);
    // Thrust
    HLAfloat32LE prototype2(0.0f);
    HLAfixedArray arrayThrust(prototype2, 8);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayThrust.set(i, prototype2);
    }
    hlaEngineActuatorsModelData->appendElement(arrayThrust);
    // N1
    HLAfloat32LE prototype3(0.0f);
    HLAfixedArray arrayN1(prototype3, 8);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayN1.set(i, prototype3);
    }
    hlaEngineActuatorsModelData->appendElement(arrayN1);
    // N2
    HLAfloat32LE prototype4(0.0f);
    HLAfixedArray arrayN2(prototype4, 8);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayN2.set(i, prototype4);
    }
    hlaEngineActuatorsModelData->appendElement(arrayN2);
    // EGT
    HLAfloat32LE prototype5(0.0f);
    HLAfixedArray arrayEGT(prototype5, 8);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayEGT.set(i, prototype5);
    }
    hlaEngineActuatorsModelData->appendElement(arrayEGT);
    // EPR
    HLAfloat32LE prototype6(0.0f);
    HLAfixedArray arrayEPR(prototype6, 8);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayEPR.set(i, prototype6);
    }
    hlaEngineActuatorsModelData->appendElement(arrayEPR);
    // FF
    HLAfloat32LE prototype7(0.0f);
    HLAfixedArray arrayFF(prototype7, 8);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayFF.set(i, prototype7);
    }
    hlaEngineActuatorsModelData->appendElement(arrayFF);

#ifdef DEBUG_HLA_XPLANE_FED
std::wcout << L"HlaXplaneFederate1516e.cc -> initHLAEngineActuatorsModelData(): End" << std::endl;
#endif
}

/**
 * @brief XplaneFederateHla1516e::initHLATimeInformationModelData
 *          Initilisation of the attributes which will store simulator time information data.
 *          There is one attribute to store local simulator time information data (Own plane) and
 *          one attribute to store a remote simulator time information data (Other plane) just the time of the reception.
 * @param remote To choose if we initialize remote position or local simulator time information Data
 */
void XplaneFederateHla1516e::initHLATimeInformationModelData(bool remote)
{
#ifdef DEBUG_HLA_XPLANE_FED
std::wcout << L"HlaXplaneFederate1516e.cc -> initHLAEngineActuatorsModelData(): Start" << std::endl;
#endif
    HLAfixedRecord *hlaTimeInformationModelData;
    if(remote)
        hlaTimeInformationModelData = &_RemoteHLATimeInformationModelData;
    else
        hlaTimeInformationModelData = &_LocalHLATimeInformationModelData;

    HLAinteger32LE dateDays(0);
    HLAfloat32LE localTimeSec(.0f);

    hlaTimeInformationModelData->appendElement(dateDays);
    hlaTimeInformationModelData->appendElement(localTimeSec);

#ifdef DEBUG_HLA_XPLANE_FED
std::wcout << L"HlaXplaneFederate1516e.cc -> initHLAEngineActuatorsModelData(): End" << std::endl;
#endif
}

/**
 * @brief XplaneFederateHla1516e::initHLAControlCommandModelData
 *          Initilisation of the attributes which will store Control command data.
 *          The attribute _LocalHLAControlCommandModelData is initialized by these method and
 *          it is used to send command interaction (For example, it can be used by a controller federate).
 */
void XplaneFederateHla1516e::initHLAControlCommandModelData()
{
#ifdef DEBUG_HLA_XPLANE_FED
std::wcout << L"HlaXplaneFederate1516e.cc -> initHLAEngineActuatorsModelData(): Start" << std::endl;
#endif

    _LocalHLAControlCommandModelData.appendElement(HLAfloat32LE(.0f));
    _LocalHLAControlCommandModelData.appendElement(HLAfloat32LE(.0f));
    _LocalHLAControlCommandModelData.appendElement(HLAfloat32LE(.0f));
    HLAfloat32LE prototype(0.0f);
    HLAfixedArray arrayThrottle(prototype, 8);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayThrottle.set(i, prototype);
    }
    _LocalHLAControlCommandModelData.appendElement(arrayThrottle);
    _LocalHLAControlCommandModelData.appendElement(HLAfloat32LE(.0f));
    _LocalHLAControlCommandModelData.appendElement(HLAboolean(false));
    _LocalHLAControlCommandModelData.appendElement(HLAfloat32LE(.0f));
    _LocalHLAControlCommandModelData.appendElement(HLAfloat32LE(.0f));
    _LocalHLAControlCommandModelData.appendElement(HLAfloat32LE(.0f));

#ifdef DEBUG_HLA_XPLANE_FED
std::wcout << L"HlaXplaneFederate1516e.cc -> initHLAControlCommandModelData(): End" << std::endl;
#endif
}

/**
 * @brief XplaneFederateHla1516e::initHLAForcePosition
 *          Initilisation of the attributes which will store position data.
 *          The attribute _InteractionForcePositionData is initialized by these method and
 *          it is used to send interaction to force position of another plane.
 */
void XplaneFederateHla1516e::initHLAForcePosition()
{
#ifdef DEBUG_HLA_XPLANE_FED
std::wcout << L"HlaXplaneFederate1516e.cc -> initHLAForcePosition(): Start" << std::endl;
#endif
    _InteractionForcePositionData.appendElement(HLAfloat64LE(.0));
    _InteractionForcePositionData.appendElement(HLAfloat64LE(.0));
    _InteractionForcePositionData.appendElement(HLAfloat64LE(.0));
#ifdef DEBUG_HLA_XPLANE_FED
std::wcout << L"HlaXplaneFederate1516e.cc -> initHLAForcePosition(): End" << std::endl;
#endif
}

/**
 * @brief XplaneFederateHla1516e::initHLAVolFormation
 *          Initilisation of the attributes which will store position data for formation flight.
 *          The attribute _InteractionVolFormation is initialized by these method and
 *          it is used to send interaction to force position of another plane to respect a flight formation.
 */
void XplaneFederateHla1516e::initHLAVolFormation()
{
    std::wcout << L"HlaXplaneFederate1516e.cc -> initHLAVolFormation(): Start" << std::endl;
    _InteractionVolFormation.appendElement(HLAfloat64LE(.0));
    _InteractionVolFormation.appendElement(HLAfloat64LE(.0));
    _InteractionVolFormation.appendElement(HLAfloat64LE(.0));
    _InteractionVolFormation.appendElement(HLAfloat64LE(.0));
    _InteractionVolFormation.appendElement(HLAfloat32LE(.0));
    _InteractionVolFormation.appendElement(HLAfloat32LE(.0));
    _InteractionVolFormation.appendElement(HLAfloat32LE(.0));
    _InteractionVolFormation.appendElement(HLAfloat64LE(.0));
    _InteractionVolFormation.appendElement(HLAinteger32LE(0));
    _InteractionVolFormation.appendElement(HLAinteger32LE(0));
    std::wcout << L"HlaXplaneFederate1516e.cc -> initHLAVolFormation(): End" << std::endl;
}

// ----------------------------------------------------------------------------
// Find All Xplane DataRefs that are used
/**
 * @brief XplaneFederateHla1516e::findUsedXplaneDataRefs
 *          Initialize all dataref inside the map _XplaneDatarefs.
 *          This method is disable when client_test is used.
 *          SPECIFIC XPLANE
 */
void XplaneFederateHla1516e::findUsedXplaneDataRefs() 
{
#ifdef DEBUG_HLA_XPLANE_FED
std::wcout << L"HlaXplaneFederate1516e.cc -> findUsedXplaneDataRefs(): Start" << std::endl;
#endif
#ifdef XPLANE_PLUGIN
		_XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrLongitudeDeg] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrLongitudeDeg.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrLatitudeDeg] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrLatitudeDeg.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrAltitudeMSLMeters] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrAltitudeMSLMeters.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrAltitudeAGLMeters] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrAltitudeAGLMeters.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrPhiDeg] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrPhiDeg.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrThetaDeg] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrThetaDeg.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrPsiDeg] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrPsiDeg.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrAlphaDeg] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrAlphaDeg.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrBetaDeg] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrBetaDeg.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrHpathDeg] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrHpathDeg.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrVpathDeg] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrVpathDeg.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrPDegps] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrPDegps.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrQDegps] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrQDegps.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrRDegps] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrRDegps.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrTrueAirspeedMetersps] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrTrueAirspeedMetersps.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrindicatedAirspeedKias] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrindicatedAirspeedKias.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrGroundspeedMetersps] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrGroundspeedMetersps.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrVerticalspeedMetersps] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrVerticalspeedMetersps.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrVmach] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrVmach.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrRho] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrRho.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrLocalX] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrLocalX.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrLocalY] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrLocalY.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrLocalZ] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrLocalZ.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrLocalVxMetersps] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrLocalVxMetersps.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrLocalVyMetersps] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrLocalVyMetersps.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrLocalVzMetersps] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrLocalVzMetersps.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrLocalAxMetersps2] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrLocalAxMetersps2.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrLocalAyMetersps2] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrLocalAyMetersps2.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrLocalAzMetersps2] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrLocalAzMetersps2.c_str());
        
        // Command
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrElevatorDeflectionNormCommand] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrElevatorDeflectionNormCommand.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrAileronDeflectionNormCommand] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrAileronDeflectionNormCommand.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrRudderDeflectionNormCommand] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrRudderDeflectionNormCommand.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrThrottleCommand] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrThrottleCommand.c_str());
		_XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrStabilizerDeflectionDegreesCommand] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrStabilizerDeflectionDegreesCommand.c_str());
		_XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrGearHandleStatusSwitchCommand] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrGearHandleStatusSwitchCommand.c_str());
		_XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrSpeedBrakesRatioCommand] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrSpeedBrakesRatioCommand.c_str());
		_XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrFlapsDeploymentRatioCommand] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrFlapsDeploymentRatioCommand.c_str());
		_XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrParkingBrakesRatioCommand] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrParkingBrakesRatioCommand.c_str());
		
		// Hydraulics Actuators Dataref
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrLeftAileronEffectiveDeflectionDegrees] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrLeftAileronEffectiveDeflectionDegrees.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrRightAileronEffectiveDeflectionDegrees] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrRightAileronEffectiveDeflectionDegrees.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrLeftElevatorEffectiveDeflectionDegrees] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrLeftElevatorEffectiveDeflectionDegrees.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrRightElevatorEffectiveDeflectionDegrees] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrRightElevatorEffectiveDeflectionDegrees.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrHorizontalStabilizerEffectiveDeflectionDegrees] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrHorizontalStabilizerEffectiveDeflectionDegrees.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrFlapsEffectiveDeflectionDegrees] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrFlapsEffectiveDeflectionDegrees.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrRudderEffectiveDeflectionDegrees] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrRudderEffectiveDeflectionDegrees.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrLeftSpeedbrakesEffectiveDeflectionDegrees] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrLeftSpeedbrakesEffectiveDeflectionDegrees.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrRightSpeedbrakesEffectiveDeflectionDegrees] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrRightSpeedbrakesEffectiveDeflectionDegrees.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrLeftSpoilersEffectiveDeflectionDegrees] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrLeftSpoilersEffectiveDeflectionDegrees.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrRightSpoilersEffectiveDeflectionDegrees] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrRightSpoilersEffectiveDeflectionDegrees.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrGearsEffectiveDeploymentRatio] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrGearsEffectiveDeploymentRatio.c_str());
        
        // Engine Actuators Dataref
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrEnginesThrottleUsedRatio] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrEnginesThrottleUsedRatio.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrEnginesThrust] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrEnginesThrust.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrEnginesN1Percent] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrEnginesN1Percent.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrEnginesN2Percent] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrEnginesN2Percent.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrEnginesEGTDegreesCelsius] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrEnginesEGTDegreesCelsius.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrEnginesEPRRatio] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrEnginesEPRRatio.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrEnginesFuelFlowKgSec] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrEnginesFuelFlowKgSec.c_str());

        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrDateDays] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrDateDays.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrLocalTimeSec] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrLocalTimeSec.c_str());
        //_XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrFrameRateFps] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrFrameRateFps.c_str());
		
		_XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrOverRideArtStab] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrOverRideArtStab.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrOverRideFlightControl] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrOverRideFlightControl.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrOverRidePlanePath] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrOverRidePlanePath.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrOverRideThrottle] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrOverRideThrottle.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrOverRideGearBrake] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrOverRideGearBrake.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrOverRideAutopilot] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrOverRideAutopilot.c_str());
        _XplaneDatarefs[XPLANE_DATAREFS_PATH::_DrOverRideJoystick] = XPLMFindDataRef(XPLANE_DATAREFS_PATH::_DrOverRideJoystick.c_str());
#endif
#ifdef DEBUG_HLA_XPLANE_FED
std::wcout << L"HlaXplaneFederate1516e.cc -> findUsedXplaneDataRefs(): End" << std::endl;
#endif
}



// ----------------------------------------------------------------------------
// Create a federation from Federation Name and FomFileName
/**
 * @brief XplaneFederateHla1516e::createFederationExecution
 *          This is the first step to initialize a federate.
 *          these method save if this federate is the creator of the federation or not.
 */
void XplaneFederateHla1516e::createFederationExecution() 
{
	#ifdef DEBUG_HLA_XPLANE_FED
	std::wcout << L"HlaXplaneFederate1516e.cc -> createFederationExecution(): Start" << std::endl;
	#endif
    try 
    {
        _RtiAmb->createFederationExecution( _FederationName
										 , _FomFileName
										 );
        _IsCreator = true;
    } 
    catch ( rti1516e::FederationExecutionAlreadyExists ) 
    {
		std::wcout << L"CFE: Federation \"" << getString(_FederationName).c_str() << "\" already created by another federate." << std::endl;
        _IsCreator = false;
	} 
	catch ( rti1516e::Exception &e ) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	} 
    catch (...)
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    std::wcout << "Creator : " << _IsCreator << std::endl;
    #ifdef DEBUG_HLA_XPLANE_FED
	std::wcout << L"HlaXplaneFederate1516e.cc -> createFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
// Destroy a federation from Federation Name
/**
 * @brief XplaneFederateHla1516e::destroyFederationExecution
 *          Destroy the federation only if the federate is the creator of the federation
 */
void XplaneFederateHla1516e::destroyFederationExecution() 
{
	#ifdef DEBUG_HLA_XPLANE_FED
	std::wcout << L"HlaXplaneFederate1516e.cc -> destroyFederationExecution(): Start" << std::endl;
	#endif
	if (_IsCreator)
	{
		try 
		{
			_RtiAmb->destroyFederationExecution(_FederationName);

		}
		catch (rti1516e::Exception& e) 
		{
			std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
		}
	}
	#ifdef DEBUG_HLA_XPLANE_FED
	std::wcout << L"HlaXplaneFederate1516e.cc -> destroyFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
/**
 * @brief XplaneFederateHla1516e::joinFederationExecution
 *          Join the federation execution. The federation must have be created by a federate before.
 */
void XplaneFederateHla1516e::joinFederationExecution() 
{
	#ifdef DEBUG_HLA_XPLANE_FED
	std::wcout << L"HlaXplaneFederate1516e.cc -> joinFederationExecution(): Start" << std::endl;
	std::wcout << L"Federate Name is: "<< _FederateName << std::endl;
	#endif
    try 
    {
        _FederateHandle = _RtiAmb->joinFederationExecution( _FederateName
														 , L"HlaXplaneFederateEntity"
                                                         , _FederationName
                                                         );
    }
    catch (rti1516e::Exception& e) 
    {
        std::wcout << L"DFE RTI Exception caught Error " << e.what() <<std::endl;
    }
	#ifdef DEBUG_HLA_XPLANE_FED
	std::wcout << L"HlaXplaneFederate1516e.cc -> joinFederationExecution(): End" << std::endl;
	#endif
}

// ----------------------------------------------------------------------------
/**
 * @brief XplaneFederateHla1516e::resignFederationExecution
 */
void XplaneFederateHla1516e::resignFederationExecution() 
{
    #ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"HlaXplaneFederate1516e.cc -> resignFederationExecution(): Start" << std::endl;
    #endif
    try 
    {
		_RtiAmb->resignFederationExecution(rti1516e::CANCEL_THEN_DELETE_THEN_DIVEST);
	}
	catch (rti1516e::Exception& e) 
	{
		std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	}
	#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"HlaXplaneFederate1516e.cc -> resignFederationExecution(): End" << std::endl;
#endif
}


// ----------------------------------------------------------------------------
// Get the federate handle
/**
 * @brief XplaneFederateHla1516e::getFederateHandle
 * @return a reference to the member rti1516e::FederateHandle _FederateHandle
 */
rti1516e::FederateHandle XplaneFederateHla1516e::getFederateHandle() const
{
    return _FederateHandle;
}

// ----------------------------------------------------------------------------
// Set all Time management settings for federate
/**
 * @brief XplaneFederateHla1516e::setHlaTimeManagementSettings
 *          Set all time initilization setup
 * @param TimeStep It's the time between to step when we use TimeAdvanceRequest method
 * @param Lookahead It's the time use when enableTimeRegulation is called
 * @param LocalTime It's the time represention of the federate
 * @param SimulationEndTime It's the maximum time during the federate execution is valid
 */
void XplaneFederateHla1516e::setHlaTimeManagementSettings( RTI1516fedTime TimeStep
																, RTI1516fedTime Lookahead
																, RTI1516fedTime LocalTime
																, RTI1516fedTime SimulationEndTime
																)
{
#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> setHlaTimeManagementSettings(): Start" << std::endl;
#endif
	_TimeStep          = TimeStep;
	_Lookahead         = Lookahead;
	_LocalTime         = LocalTime;
	_SimulationEndTime = SimulationEndTime;
#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> setHlaTimeManagementSettings(): End" << std::endl;
#endif
}

/**
 * @brief XplaneFederateHla1516e::initializationFederate
 *          Initialize all step of the federate. Use by the client_test.
 */
void XplaneFederateHla1516e::initializationFederate()
{
    createFederationExecution();
    joinFederationExecution();
    runOneStep();
    getAllHandles();
    publishAndSubscribe();
    runOneStep();
    registerObjectInstances();
    enableTimeRegulation();
    enableTimeConstrained();
    enableAsynchronousDelivery();
    if(!isCreator()) {
        pause();
    }
    runOneStep();
    setInitialized(true);
}

/**
 * @brief XplaneFederateHla1516e::initializationRecords
 *          Initilaize all fixed record member and datarefs.
 */
void XplaneFederateHla1516e::initializationRecords()
{
    initHLAFlightDynamicsModelData(false);
    initHLAHydraulicActuatorsModelData(false);
    initHLAEngineActuatorsModelData(false);
    initHLATimeInformationModelData(false);
    initHLAFlightDynamicsModelData(true);
    initHLAHydraulicActuatorsModelData(true);
    initHLAEngineActuatorsModelData(true);
    initHLATimeInformationModelData(true);
    initHLAControlCommandModelData();
    initHLAForcePosition();
    initHLAVolFormation();
    findUsedXplaneDataRefs();
}

/**
 * @brief XplaneFederateHla1516e::isInitialized
 *          The federate is initilized when all step of initilization of the federate is done
 * @return if the federate is initilized.
 */
bool XplaneFederateHla1516e::isInitialized()
{
    return _IsInitialized;
}

/**
 * @brief XplaneFederateHla1516e::setInitialized
 * @param isInitialized set if federate is initialized or not because initialization can be done outside.
 */
void XplaneFederateHla1516e::setInitialized(bool isInitialized)
{
    _IsInitialized = isInitialized;
}

/**
 * @brief XplaneFederateHla1516e::isCreator
 * @return if the federate is the creator of the federation
 */
bool XplaneFederateHla1516e::isCreator()
{
    return _IsCreator;
}

/**
 * @brief XplaneFederateHla1516e::close
 */
void XplaneFederateHla1516e::close()
{
#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> close(): Start" << std::endl;
#endif

    unpublishAndUnsubscribe();
    deleteObjectInstances();
    resignFederationExecution();
    destroyFederationExecution();

#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> close(): End" << std::endl;
#endif
}

// ----------------------------------------------------------------------------
// get handles of objet/interaction classes
/**
 * @brief XplaneFederateHla1516e::getAllHandles
 *          Initilize all members handle
 */
void XplaneFederateHla1516e::getAllHandles()
{
	#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> getAllHandles(): Start" << std::endl;
    #endif
	try 
	{
		// Subscribed 
		std::wstring ClassControlCommandModel (L"ClassControlCommandModel");
		std::wstring AttrControlCommandModelData (L"AttrControlCommandModelData");
		_ObjectClassHandleForClassControlCommandModel = _RtiAmb->getObjectClassHandle(ClassControlCommandModel);
		_AttributeHandleForAttrControlCommandModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleForClassControlCommandModel, AttrControlCommandModelData);
		
		// Published my own flight dynamics Model Parameters + Time
		// Also subsribed to get data from the others xplane/aerofly/flightgears instances
		std::wstring ClassFlightDynamicsModel (L"ClassFlightDynamicsModel");
		std::wstring AttrFlightDynamicsModelData (L"AttrFlightDynamicsModelData");
		_ObjectClassHandleForClassFlightDynamicsModel = _RtiAmb->getObjectClassHandle(ClassFlightDynamicsModel);
		_AttributeHandleForAttrFlightDynamicsModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleForClassFlightDynamicsModel, AttrFlightDynamicsModelData);

		// Hydraulic Actuators
		std::wstring ClassHydraulicActuatorsModel (L"ClassHydraulicActuatorsModel");
		std::wstring AttrHydraulicActuatorsModelData (L"AttrHydraulicActuatorsModelData");
		_ObjectClassHandleForClassHydraulicActuatorsModel = _RtiAmb->getObjectClassHandle(ClassHydraulicActuatorsModel);
		_AttributeHandleForAttrHydraulicActuatorsModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleForClassHydraulicActuatorsModel, AttrHydraulicActuatorsModelData);
		
		// Engine Actuators
		std::wstring ClassEngineActuatorsModel (L"ClassEngineActuatorsModel");
		std::wstring AttrEngineActuatorsModelData (L"AttrEngineActuatorsModelData");
		_ObjectClassHandleForClassEngineActuatorsModel = _RtiAmb->getObjectClassHandle(ClassEngineActuatorsModel);
		_AttributeHandleForAttrEngineActuatorsModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleForClassEngineActuatorsModel, AttrEngineActuatorsModelData);
		
		// Time Information
		std::wstring ClassTimeInformationModel (L"ClassTimeInformationModel");
		std::wstring AttrTimeInformationModelData (L"AttrTimeInformationModelData");
		_ObjectClassHandleForClassTimeInformationModel = _RtiAmb->getObjectClassHandle(ClassTimeInformationModel);
		_AttributeHandleForAttrTimeInformationModelData = _RtiAmb->getAttributeHandle(_ObjectClassHandleForClassTimeInformationModel, AttrTimeInformationModelData);

        // Interaction force position
        std::wstring InteractionForcePosition (L"ForcePosition");
        std::wstring PlaneName(L"PlaneName");
        std::wstring ParameterPosition(L"Position");
        _InteractionHandleForForcePosition = _RtiAmb->getInteractionClassHandle(InteractionForcePosition);
        _ParameterHandleForInteractionPlaneNamePosition = _RtiAmb->getParameterHandle(_InteractionHandleForForcePosition, PlaneName);
        _ParameterHandleForInteractionForcePosition = _RtiAmb->getParameterHandle(_InteractionHandleForForcePosition, ParameterPosition);

        // Interaction Vol Formation
        std::wstring InteractionVolFormation (L"VolFormation");
        std::wstring PlaneNameVolFormation(L"PlaneIdentification");
        std::wstring PositionVolFormation(L"PositionInFormation");
        _InteractionHandleForVolFormation = _RtiAmb->getInteractionClassHandle(InteractionVolFormation);
        _ParameterHandleForInteractionPlaneNameVolFormation = _RtiAmb->getParameterHandle(_InteractionHandleForVolFormation, PlaneNameVolFormation);
        _ParameterHandleForInteractionVolFormation = _RtiAmb->getParameterHandle(_InteractionHandleForVolFormation, PositionVolFormation);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
	#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> getAllHandles(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publish and subscribe
/**
 * @brief XplaneFederateHla1516e::publishAndSubscribe
 *          publish and subscribe for all objects and interactions
 */
void XplaneFederateHla1516e::publishAndSubscribe()
{
	#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> publishAndSubscribe(): Start" << std::endl;
    #endif
	try 
	{
        // For Class/Attributes ControlCommand
        _AttributeHandleSetForAttrControlCommandModelData.insert(_AttributeHandleForAttrControlCommandModelData);
        _RtiAmb->publishObjectClassAttributes(_ObjectClassHandleForClassControlCommandModel,_AttributeHandleSetForAttrControlCommandModelData);
        _AttributeHandleSetForAttrControlCommandModelDataSubscribed.insert(_AttributeHandleForAttrControlCommandModelData);
        _RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleForClassControlCommandModel,_AttributeHandleSetForAttrControlCommandModelDataSubscribed);

        _AttributeHandleSetForAttrFlightDynamicsModelDataSubscribed.insert(_AttributeHandleForAttrFlightDynamicsModelData);
		_RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleForClassFlightDynamicsModel,_AttributeHandleSetForAttrFlightDynamicsModelDataSubscribed);  

		// For Class/Attributes published
        _AttributeHandleSetForAttrFlightDynamicsModelData.insert(_AttributeHandleForAttrFlightDynamicsModelData);
        _RtiAmb->publishObjectClassAttributes(_ObjectClassHandleForClassFlightDynamicsModel,_AttributeHandleSetForAttrFlightDynamicsModelData);
        
        _AttributeHandleSetForAttrHydraulicActuatorsModelData.insert(_AttributeHandleForAttrHydraulicActuatorsModelData);
        _RtiAmb->publishObjectClassAttributes(_ObjectClassHandleForClassHydraulicActuatorsModel,_AttributeHandleSetForAttrHydraulicActuatorsModelData);
        
        _AttributeHandleSetForAttrEngineActuatorsModelData.insert(_AttributeHandleForAttrEngineActuatorsModelData);
        _RtiAmb->publishObjectClassAttributes(_ObjectClassHandleForClassEngineActuatorsModel,_AttributeHandleSetForAttrEngineActuatorsModelData);
        _RtiAmb->subscribeObjectClassAttributes(_ObjectClassHandleForClassEngineActuatorsModel,_AttributeHandleSetForAttrEngineActuatorsModelData);
        
        _AttributeHandleSetForAttrTimeInformationModelData.insert(_AttributeHandleForAttrTimeInformationModelData);
        _RtiAmb->publishObjectClassAttributes(_ObjectClassHandleForClassTimeInformationModel,_AttributeHandleSetForAttrTimeInformationModelData);

        // For Interaction subscribed
        _RtiAmb->subscribeInteractionClass(_InteractionHandleForForcePosition);
        _RtiAmb->publishInteractionClass(_InteractionHandleForForcePosition);

        _RtiAmb->subscribeInteractionClass(_InteractionHandleForVolFormation);
        _RtiAmb->publishInteractionClass(_InteractionHandleForVolFormation);
	} 
	catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
    #ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> publishAndSubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Carry out publications and subscriptions
/**
 * @brief XplaneFederateHla1516e::unpublishAndUnsubscribe
 *          Carry out publications and subscriptions
 */
void XplaneFederateHla1516e::unpublishAndUnsubscribe()
{
	#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> unpublishAndUnsubscribe(): Start" << std::endl;
    #endif
    try 
    {
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleForClassFlightDynamicsModel);
        _RtiAmb->unsubscribeObjectClass(_ObjectClassHandleForClassControlCommandModel);
        _RtiAmb->unsubscribeInteractionClass(_InteractionHandleForForcePosition);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }
       
    try 
    {
        _RtiAmb->unpublishObjectClass(_ObjectClassHandleForClassFlightDynamicsModel);
        _RtiAmb->unpublishObjectClass(_ObjectClassHandleForClassHydraulicActuatorsModel);
        _RtiAmb->unpublishObjectClass(_ObjectClassHandleForClassEngineActuatorsModel);
        _RtiAmb->unpublishObjectClass(_ObjectClassHandleForClassTimeInformationModel);
        _RtiAmb->unpublishObjectClass(_ObjectClassHandleForClassControlCommandModel);
        _RtiAmb->unpublishInteractionClass(_InteractionHandleForForcePosition);
    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }


    #ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> unpublishAndUnsubscribe(): End" << std::endl;
    #endif
}

// ----------------------------------------------------------------------------
// Register an Object instance
/**
 * @brief XplaneFederateHla1516e::registerObjectInstances
 *          Register all objects instance
 */
void XplaneFederateHla1516e::registerObjectInstances()
{
	#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> registerObjectInstances(): Start" << std::endl;
    #endif
	try 
    {
        // must be done using retrieve federate handle
        std::wstring XplaneFlightDynamicsModelData (L"XplaneFlightDynamicsModelData-");
		XplaneFlightDynamicsModelData = XplaneFlightDynamicsModelData + _FederateName;
        _ObjectInstanceHandleForClassFlightDynamicsModel = _RtiAmb->registerObjectInstance(_ObjectClassHandleForClassFlightDynamicsModel,XplaneFlightDynamicsModelData);
        std::wcout << XplaneFlightDynamicsModelData << L" Done" << std::endl;

        std::wstring XplaneHydraulicActuatorsModelData (L"XplaneHydraulicActuatorsModelData-");
        XplaneHydraulicActuatorsModelData = XplaneHydraulicActuatorsModelData + _FederateName;
        _ObjectInstanceHandleForClassHydraulicActuatorsModel = _RtiAmb->registerObjectInstance(_ObjectClassHandleForClassHydraulicActuatorsModel,XplaneHydraulicActuatorsModelData);
        std::wcout << XplaneHydraulicActuatorsModelData << L" Done" << std::endl;

        std::wstring XplaneEngineActuatorsModelData (L"XplaneEngineActuatorsModelData-");
		XplaneEngineActuatorsModelData = XplaneEngineActuatorsModelData + _FederateName;
        _ObjectInstanceHandleForClassEngineActuatorsModel = _RtiAmb->registerObjectInstance(_ObjectClassHandleForClassEngineActuatorsModel,XplaneEngineActuatorsModelData);
        std::wcout << XplaneEngineActuatorsModelData << L" Done" << std::endl;

        std::wstring XplaneTimeInformationModelData (L"XplaneTimeInformationModelData-");
		XplaneTimeInformationModelData = XplaneTimeInformationModelData + _FederateName;
        _ObjectInstanceHandleForClassTimeInformationModel = _RtiAmb->registerObjectInstance(_ObjectClassHandleForClassTimeInformationModel,XplaneTimeInformationModelData);
        std::wcout << XplaneTimeInformationModelData << L" Done" << std::endl;

        std::wstring XplaneControlCommandModelData (L"XplaneControlCommandModelData-");
        XplaneControlCommandModelData = XplaneControlCommandModelData + _FederateName;
        _ObjectInstanceHandleForClassControlCommandModel = _RtiAmb->registerObjectInstance(_ObjectClassHandleForClassControlCommandModel,XplaneControlCommandModelData);
        std::wcout << XplaneControlCommandModelData << L" Done" << std::endl;

    } 
    catch ( rti1516e::Exception &e ) 
    {
        std::wcout << L"RTI Exception caught Error when register object instance" << e.what() <<std::endl;
    } 
    catch ( ... ) 
    {
        std::cerr << "Error: unknown non-RTI exception when register object instance." << std::endl;
    }
	#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> registerObjectInstances(): End" << std::endl;
#endif
}

/**
 * @brief XplaneFederateHla1516e::deleteObjectInstances
 *          Delete all object instances
 */
void XplaneFederateHla1516e::deleteObjectInstances()
{
#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> deleteObjectInstances(): Start" << std::endl;
#endif

    try
    {
        // must be done using retrieve federate handle
        HLAunicodeString XplaneFlightDynamicsModelData(L"XplaneFlightDynamicsModelData_" + _FederateName);
        _RtiAmb->deleteObjectInstance(_ObjectInstanceHandleForClassFlightDynamicsModel, XplaneFlightDynamicsModelData.encode());
        HLAunicodeString XplaneHydraulicActuatorsModelData(L"XplaneHydraulicActuatorsModelData_" + _FederateName);
        _RtiAmb->deleteObjectInstance(_ObjectInstanceHandleForClassHydraulicActuatorsModel, XplaneHydraulicActuatorsModelData.encode());
        HLAunicodeString XplaneEngineActuatorsModelData(L"XplaneEngineActuatorsModelData_" + _FederateName);
        _RtiAmb->deleteObjectInstance(_ObjectInstanceHandleForClassEngineActuatorsModel, XplaneEngineActuatorsModelData.encode());
        HLAunicodeString XplaneTimeInformationModelData (L"XplaneTimeInformationModelData_" + _FederateName);
        _RtiAmb->deleteObjectInstance(_ObjectInstanceHandleForClassTimeInformationModel, XplaneTimeInformationModelData.encode());
    }
    catch ( rti1516e::Exception &e )
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    }
    catch ( ... )
    {
        std::cerr << "Error: unknown non-RTI exception." << std::endl;
    }

#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> deleteObjectInstances(): End" << std::endl;
#endif
}

// ----------------------------------------------------------------------------
// Callback : discover object instance
/**
 * @brief XplaneFederateHla1516e::discoverObjectInstance
 *          This callback is called when an object is register by another federate.
 * @param theObject The handle of the object
 * @param theObjectClass The handle of the class of the object
 * @param theObjectInstanceName The name of the object
 */
void XplaneFederateHla1516e::discoverObjectInstance( rti1516e::ObjectInstanceHandle theObject
                                                   , rti1516e::ObjectClassHandle theObjectClass
                                                   , std::wstring const &theObjectInstanceName
                                                   )
                                             throw ( rti1516e::FederateInternalError )
{
    #ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> discoverObjectInstance(): Start" << std::endl;
    #endif
    // Comparing with something here to avoid that Control Command control mutliple aircraft kind of affiliation mecanism
    unsigned long posBeginName = theObjectInstanceName.find_first_of(L"-") + 1;
    unsigned long posEndName = theObjectInstanceName.size();
    unsigned long posBeginType = theObjectInstanceName.find_last_of(L"-") + 1;
    unsigned long nbCharToTheEndType = theObjectInstanceName.size() - posBeginType;
    unsigned long nbCharToTheEndName = posEndName - posBeginName;
    std::wstring planeType = theObjectInstanceName.substr(posBeginType, nbCharToTheEndType);
    std::wstring federateName = theObjectInstanceName.substr(posBeginName, nbCharToTheEndName);
    std::wcout << L"federateType : " << planeType << std::endl;
    std::wcout << L"The federate name : " << federateName << std::endl;
    std::wcout << L"theObjectInstanceName : " << theObjectInstanceName << std::endl;
    std::wcout << L"theObject : " << theObject << std::endl;
    std::wcout << "theObjectClass : " << theObjectClass << std::endl;
    std::wstring ControlCommandTest (L"ControlCommandTest");
    if(theObjectClass == _ObjectClassHandleForClassFlightDynamicsModel)
	{
        ObjectInstanceInfo objectInstanceInfo;
        objectInstanceInfo.federateName = federateName;
        objectInstanceInfo.type = TypeObjectHandle::FlightDynamicsModel;
        _ObjectsInstanceInfo[theObject] = objectInstanceInfo;
        if(_RemoteAircraftsToDraw.find(federateName) == _RemoteAircraftsToDraw.end())
            _RemoteAircraftsToDraw[federateName].ContructRemoteAircraftToDraw(static_cast<int>(_RemoteAircraftsToDraw.size()), getString(planeType), getString(federateName));
		#ifdef DEBUG_HLA_XPLANE_FED
        std::wcout << L"A new remote _ObjectClassHandleForClassFlightDynamicsModel has been discovered" << std::endl;
		#endif
    }
    else if(theObjectClass == _ObjectClassHandleForClassEngineActuatorsModel) // Whe have to create a consumer for this public object class
    {
        ObjectInstanceInfo objectInstanceInfo;
        objectInstanceInfo.federateName = federateName;
        objectInstanceInfo.type = TypeObjectHandle::EngineActuators;
        _ObjectsInstanceInfo[theObject] = objectInstanceInfo;
        if(_RemoteAircraftsToDraw.find(federateName) == _RemoteAircraftsToDraw.end())
            _RemoteAircraftsToDraw[federateName].ContructRemoteAircraftToDraw(static_cast<int>(_RemoteAircraftsToDraw.size()), getString(planeType), getString(federateName));
        #ifdef DEBUG_HLA_XPLANE_FED
        std::wcout << L"A new remote_ObjectClassHandleForClassEngineActuatorsModel has been discovered" << std::endl;
        #endif
    }
    else if (theObjectClass == _ObjectClassHandleForClassControlCommandModel)/*  &&  (!wcscmp(theObjectInstanceName.c_str(),ControlCommandTest.c_str()))*/
	{
//		_DiscovObjectInstanceHandleForClassControlCommandModel = true;
//		_ObjectInstanceHandleForClassControlCommandModel = theObject;
        ObjectInstanceInfo objectInstanceInfo;
        objectInstanceInfo.federateName = federateName;
        objectInstanceInfo.type = TypeObjectHandle::ControlCommand;
        _ObjectsInstanceInfo[theObject] = objectInstanceInfo;
		#ifdef DEBUG_HLA_XPLANE_FED
		std::wcout << L"New Command Object Instance has been discovered" << std::endl;
		#endif
    }

    std::wcout << L"\n\n\n Discover Object :" << std::endl;
    for(auto it=_ObjectsInstanceInfo.begin(); it != _ObjectsInstanceInfo.end(); ++it)
    {
        std::wcout << it->first.toString() << L" : " << it->second.federateName << L" - type : " << static_cast<int>(it->second.type) << std::endl;
    }
    std::wcout << L"\n\n\n" << std::endl;

#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> discoverObjectInstance(): End" << std::endl;
#endif
}

/**
 * @brief XplaneFederateHla1516e::removeObjectInstance
 *          These callback is call when a federate delete an object instance
 * @param theObject
 */
void XplaneFederateHla1516e::removeObjectInstance(ObjectInstanceHandle theObject
                                                  , const VariableLengthData &/*theUserSuppliedTag*/
                                                  , OrderType /*sentOrder*/
                                                  , SupplementalRemoveInfo /*theRemoveInfo*/)
                                                    throw (rti1516e::FederateInternalError)
{
#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"HlaXplaneFederate1516e.cc -> removeObjectInstance(): Start" << std::endl;
#endif

    auto itObjectInstance = _ObjectsInstanceInfo.find(theObject);
    if(itObjectInstance != _ObjectsInstanceInfo.end())
    {
        auto itAircraftToDraw = _RemoteAircraftsToDraw.find(itObjectInstance->second.federateName);
        if(itAircraftToDraw != _RemoteAircraftsToDraw.end())
            _RemoteAircraftsToDraw.erase(itAircraftToDraw);
        _ObjectsInstanceInfo.erase(itObjectInstance);
    }

#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"HlaXplaneFederate1516e.cc -> removeObjectInstance(): End" << std::endl;
#endif
}

// ----------------------------------------------------------------------------
// getLocalHLAFlightDynamicsModelData 
/**
 * @brief XplaneFederateHla1516e::getfromXplaneLocalHLAFlightDynamicsModelData
 *          Update _LocalHLAFlightDynamicsModelData member with value inside Xplane.
 *
 *          SPECIFIC XPLANE
 */
void XplaneFederateHla1516e::getfromXplaneLocalHLAFlightDynamicsModelData()
{
//#ifdef DEBUG_HLA_XPLANE_REFLECT
//    std::wcout << L"XplaneFederateHla1516e.cc -> getfromXplaneLocalHLAFlightDynamicsModelData(): Start" << std::endl;
//#endif
#ifdef XPLANE_PLUGIN
	HLAfloat64LE latitutdeDeg(XPLMGetDatad(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLatitudeDeg)));
	HLAfloat64LE longitudeDeg(XPLMGetDatad(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLongitudeDeg)));
	HLAfloat64LE altitudeMSLMeters(XPLMGetDatad(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrAltitudeMSLMeters)));
	HLAfloat32LE altitudeAGLMeters(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrAltitudeAGLMeters)));
	HLAfloat32LE phiDeg(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrPhiDeg)));
	HLAfloat32LE thetaDeg(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrThetaDeg)));
	HLAfloat32LE psiDeg(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrPsiDeg)));
	HLAfloat32LE alphaDeg(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrAlphaDeg)));
	HLAfloat32LE betaDeg(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrBetaDeg)));
	HLAfloat32LE hpathDeg(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrHpathDeg)));
	HLAfloat32LE vpathDeg(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrVpathDeg)));
	HLAfloat32LE pDegps(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrPDegps)));
	HLAfloat32LE qDegps(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrQDegps)));
	HLAfloat32LE rDegps(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrRDegps)));
	HLAfloat32LE trueAirspeedMetersps(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrTrueAirspeedMetersps)));
	HLAfloat32LE indicatedAirspeedKias(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrindicatedAirspeedKias)));
	HLAfloat32LE groundspeedMetersps(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrGroundspeedMetersps)));
	HLAfloat32LE verticalspeedMetersps(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrVerticalspeedMetersps)));
	HLAfloat32LE vMach(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrVmach)));
	HLAfloat32LE rho(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrRho)));

	HLAfloat64LE localX(XPLMGetDatad(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalX)));
	HLAfloat64LE localY(XPLMGetDatad(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalY)));
	HLAfloat64LE localZ(XPLMGetDatad(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalZ)));

	HLAfloat32LE localVxMetersps(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalVxMetersps)));
	HLAfloat32LE localVyMetersps(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalVyMetersps)));
	HLAfloat32LE localVzMetersps(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalVzMetersps)));
	HLAfloat32LE localAxMetersps2(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalAxMetersps2)));
	HLAfloat32LE localAyMetersps2(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalAyMetersps2)));
	HLAfloat32LE localAzMetersps2(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalAzMetersps2)));

	_LocalHLAFlightDynamicsModelData.set(0, latitutdeDeg);
	_LocalHLAFlightDynamicsModelData.set(1, longitudeDeg);
	_LocalHLAFlightDynamicsModelData.set(2, altitudeMSLMeters);
	_LocalHLAFlightDynamicsModelData.set(3, altitudeAGLMeters);
	_LocalHLAFlightDynamicsModelData.set(4, phiDeg);
	_LocalHLAFlightDynamicsModelData.set(5, thetaDeg);
	_LocalHLAFlightDynamicsModelData.set(6, psiDeg);
	_LocalHLAFlightDynamicsModelData.set(7, alphaDeg);
	_LocalHLAFlightDynamicsModelData.set(8, betaDeg);
	_LocalHLAFlightDynamicsModelData.set(9, hpathDeg);
	_LocalHLAFlightDynamicsModelData.set(10, vpathDeg);
	_LocalHLAFlightDynamicsModelData.set(11, pDegps);
	_LocalHLAFlightDynamicsModelData.set(12, qDegps);
	_LocalHLAFlightDynamicsModelData.set(13, rDegps);
	_LocalHLAFlightDynamicsModelData.set(14, trueAirspeedMetersps);
	_LocalHLAFlightDynamicsModelData.set(15, indicatedAirspeedKias);
	_LocalHLAFlightDynamicsModelData.set(16, groundspeedMetersps);
	_LocalHLAFlightDynamicsModelData.set(17, verticalspeedMetersps);
	_LocalHLAFlightDynamicsModelData.set(18, vMach);
	_LocalHLAFlightDynamicsModelData.set(19, rho);

	_LocalHLAFlightDynamicsModelData.set(20, localX);
	_LocalHLAFlightDynamicsModelData.set(21, localY);
	_LocalHLAFlightDynamicsModelData.set(22, localZ);

	_LocalHLAFlightDynamicsModelData.set(23, localVxMetersps);
	_LocalHLAFlightDynamicsModelData.set(24, localVyMetersps);
	_LocalHLAFlightDynamicsModelData.set(25, localVzMetersps);
	_LocalHLAFlightDynamicsModelData.set(26, localAxMetersps2);
	_LocalHLAFlightDynamicsModelData.set(27, localAyMetersps2);
    _LocalHLAFlightDynamicsModelData.set(28, localAzMetersps2);
#endif

//#ifdef DEBUG_HLA_XPLANE_REFLECT
//    std::wcout << L"XplaneFederateHla1516e.cc -> getfromXplaneLocalHLAFlightDynamicsModelData(): End" << std::endl;
//#endif
}
// SPECIFIC AEROFLY
void XplaneFederateHla1516e::getfromAeroflyLocalHLAFlightDynamicsModelData(std::map<std::wstring, double> mapLocalDynamicModelDatas)
{
#ifdef AEROFLY_PLUGIN
    HLAfloat64LE latitutdeDeg(mapLocalDynamicModelDatas[L"Latitude"]);
    HLAfloat64LE longitudeDeg(mapLocalDynamicModelDatas[L"Longitude"]);
    HLAfloat64LE altitudeMSLMeters(mapLocalDynamicModelDatas[L"Altitude"]);
    HLAfloat32LE altitudeAGLMeters(mapLocalDynamicModelDatas[L"Altitude"]);
    HLAfloat32LE phiDeg(mapLocalDynamicModelDatas[L"phi"]);
    HLAfloat32LE thetaDeg(mapLocalDynamicModelDatas[L"Pitch"]);
    HLAfloat32LE psiDeg(mapLocalDynamicModelDatas[L"psi"]);
    HLAfloat32LE alphaDeg(10.0);
    HLAfloat32LE betaDeg(10.0);
    HLAfloat32LE hpathDeg(10.0);
    HLAfloat32LE vpathDeg(10.0);
    HLAfloat32LE pDegps(10.0);
    HLAfloat32LE qDegps(10.0);
    HLAfloat32LE rDegps(10.0);
    HLAfloat32LE trueAirspeedMetersps(10.0);
    HLAfloat32LE indicatedAirspeedKias(10.0);
    HLAfloat32LE groundspeedMetersps(10.0);
    HLAfloat32LE verticalspeedMetersps(10.0);
    HLAfloat32LE vMach(mapLocalDynamicModelDatas[L"MachNumber"]);
    HLAfloat32LE rho(10.0);

    HLAfloat64LE localX(mapLocalDynamicModelDatas[L"PositionX"]);
    HLAfloat64LE localY(mapLocalDynamicModelDatas[L"PositionY"]);
    HLAfloat64LE localZ(mapLocalDynamicModelDatas[L"PositionZ"]);

    HLAfloat32LE localVxMetersps(mapLocalDynamicModelDatas[L"VelocityX"]);
    HLAfloat32LE localVyMetersps(mapLocalDynamicModelDatas[L"VelocityY"]);
    HLAfloat32LE localVzMetersps(mapLocalDynamicModelDatas[L"VelocityZ"]);

    HLAfloat32LE localAxMetersps2(0.0);
    HLAfloat32LE localAyMetersps2(0.0);
    HLAfloat32LE localAzMetersps2(0.0);
    if (mapLocalDynamicModelDatas.find(L"AccelerationX") != mapLocalDynamicModelDatas.end()) {
        localAxMetersps2 = mapLocalDynamicModelDatas[L"AccelerationX"];
        localAyMetersps2 = mapLocalDynamicModelDatas[L"AccelerationY"];
        localAzMetersps2 = mapLocalDynamicModelDatas[L"AccelerationZ"];
    }

    _LocalHLAFlightDynamicsModelData.set(0, latitutdeDeg);
    _LocalHLAFlightDynamicsModelData.set(1, longitudeDeg);
    _LocalHLAFlightDynamicsModelData.set(2, altitudeMSLMeters);
    _LocalHLAFlightDynamicsModelData.set(3, altitudeAGLMeters);
    _LocalHLAFlightDynamicsModelData.set(4, phiDeg);
    _LocalHLAFlightDynamicsModelData.set(5, thetaDeg);
    _LocalHLAFlightDynamicsModelData.set(6, psiDeg);
    _LocalHLAFlightDynamicsModelData.set(7, alphaDeg);
    _LocalHLAFlightDynamicsModelData.set(8, betaDeg);
    _LocalHLAFlightDynamicsModelData.set(9, hpathDeg);
    _LocalHLAFlightDynamicsModelData.set(10, vpathDeg);
    _LocalHLAFlightDynamicsModelData.set(11, pDegps);
    _LocalHLAFlightDynamicsModelData.set(12, qDegps);
    _LocalHLAFlightDynamicsModelData.set(13, rDegps);
    _LocalHLAFlightDynamicsModelData.set(14, trueAirspeedMetersps);
    _LocalHLAFlightDynamicsModelData.set(15, indicatedAirspeedKias);
    _LocalHLAFlightDynamicsModelData.set(16, groundspeedMetersps);
    _LocalHLAFlightDynamicsModelData.set(17, verticalspeedMetersps);
    _LocalHLAFlightDynamicsModelData.set(18, vMach);
    _LocalHLAFlightDynamicsModelData.set(19, rho);

    _LocalHLAFlightDynamicsModelData.set(20, localX);
    _LocalHLAFlightDynamicsModelData.set(21, localY);
    _LocalHLAFlightDynamicsModelData.set(22, localZ);

    _LocalHLAFlightDynamicsModelData.set(23, localVxMetersps);
    _LocalHLAFlightDynamicsModelData.set(24, localVyMetersps);
    _LocalHLAFlightDynamicsModelData.set(25, localVzMetersps);
    _LocalHLAFlightDynamicsModelData.set(26, localAxMetersps2);
    _LocalHLAFlightDynamicsModelData.set(27, localAyMetersps2);
    _LocalHLAFlightDynamicsModelData.set(28, localAzMetersps2);
#endif
}

// ----------------------------------------------------------------------------
// getfromXplaneLocalHLAHydraulicActuatorsModelData 
/**
 * @brief XplaneFederateHla1516e::getfromXplaneLocalHLAHydraulicActuatorsModelData
 *          Update _LocalHLAHydraulicActuatorsModelData member with value inside Xplane.
 *
 *          SPECIFIC XPLANE
 */
void XplaneFederateHla1516e::getfromXplaneLocalHLAHydraulicActuatorsModelData()
{
//#ifdef DEBUG_HLA_XPLANE_REFLECT
//    std::wcout << L"XplaneFederateHla1516e.cc -> getfromXplaneLocalHLAHydraulicActuatorsModelData(): Start" << std::endl;
//#endif
#ifdef XPLANE_PLUGIN
	HLAfloat32LE LeftAileronEffectiveDeflectionDegrees(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLeftAileronEffectiveDeflectionDegrees)));
	HLAfloat32LE RightAileronEffectiveDeflectionDegrees(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrRightAileronEffectiveDeflectionDegrees)));
	HLAfloat32LE LeftElevatorEffectiveDeflectionDegrees(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLeftElevatorEffectiveDeflectionDegrees)));
	HLAfloat32LE RightElevatorEffectiveDeflectionDegrees(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrRightElevatorEffectiveDeflectionDegrees)));
	HLAfloat32LE HorizontalStabilizerEffectiveDeflectionDegrees(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrHorizontalStabilizerEffectiveDeflectionDegrees)));
	HLAfloat32LE FlapsEffectiveDeployment(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrFlapsEffectiveDeflectionDegrees)));
	HLAfloat32LE RudderEffectiveDeflectionDegrees(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrRudderEffectiveDeflectionDegrees)));
	HLAfloat32LE LeftSpeedbrakesEffectiveDeflectionDegrees(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLeftSpeedbrakesEffectiveDeflectionDegrees)));
	HLAfloat32LE RightSpeedbrakesEffectiveDeflectionDegrees(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrRightSpeedbrakesEffectiveDeflectionDegrees)));
	HLAfloat32LE LeftSpoilersEffectiveDeflectionDegrees(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLeftSpoilersEffectiveDeflectionDegrees)));
	HLAfloat32LE RightSpoilersEffectiveDeflectionDegrees(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrRightSpoilersEffectiveDeflectionDegrees)));

	_LocalHLAHydraulicActuatorsModelData.set(0, LeftAileronEffectiveDeflectionDegrees);
	_LocalHLAHydraulicActuatorsModelData.set(1, RightAileronEffectiveDeflectionDegrees);
	_LocalHLAHydraulicActuatorsModelData.set(2, LeftElevatorEffectiveDeflectionDegrees);
	_LocalHLAHydraulicActuatorsModelData.set(3, RightElevatorEffectiveDeflectionDegrees);
	_LocalHLAHydraulicActuatorsModelData.set(4, HorizontalStabilizerEffectiveDeflectionDegrees);
	_LocalHLAHydraulicActuatorsModelData.set(5, FlapsEffectiveDeployment);
	_LocalHLAHydraulicActuatorsModelData.set(6, RudderEffectiveDeflectionDegrees);
	_LocalHLAHydraulicActuatorsModelData.set(7, LeftSpeedbrakesEffectiveDeflectionDegrees);
	_LocalHLAHydraulicActuatorsModelData.set(8, RightSpeedbrakesEffectiveDeflectionDegrees);
	_LocalHLAHydraulicActuatorsModelData.set(9, LeftSpoilersEffectiveDeflectionDegrees);
	_LocalHLAHydraulicActuatorsModelData.set(10, RightSpoilersEffectiveDeflectionDegrees);
	
	// FF
	float gears[10];
	XPLMGetDatavf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrGearsEffectiveDeploymentRatio), &gears[0], 0, 8);
	HLAfloat32LE prototype(0.0f);
	HLAfixedArray arrayGEARS(prototype, 10);
	for(unsigned int i=0; i < 8; i++)
	{
		arrayGEARS.set(i, HLAfloat32LE(gears[i]));
	}
	_LocalHLAHydraulicActuatorsModelData.set(11, arrayGEARS);
#endif
//#ifdef DEBUG_HLA_XPLANE_REFLECT
//    std::wcout << L"XplaneFederateHla1516e.cc -> getfromXplaneLocalHLAHydraulicActuatorsModelData(): End" << std::endl;
//#endif
}

// ----------------------------------------------------------------------------
// getfromXplaneLocalHLAHydraulicActuatorsModelData 
/**
 * @brief XplaneFederateHla1516e::getfromXplaneLocalHLATimeInformationModelData
 *          Update _LocalHLATimeInformationModelData member with value inside Xplane.
 *
 *          SPECIFIC XPLANE
 */
void XplaneFederateHla1516e::getfromXplaneLocalHLATimeInformationModelData()
{
//#ifdef DEBUG_HLA_XPLANE_REFLECT
//    std::wcout << L"XplaneFederateHla1516e.cc -> getfromXplaneLocalHLATimeInformationModelData(): Start" << std::endl;
//#endif
#ifdef XPLANE_PLUGIN
	HLAinteger32LE dateDays(XPLMGetDatai(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrDateDays)));
	HLAfloat32LE localTimeSec(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalTimeSec)));

    _LocalHLATimeInformationModelData.set(0, dateDays);
    _LocalHLATimeInformationModelData.set(1, localTimeSec);
#endif
//#ifdef DEBUG_HLA_XPLANE_REFLECT
//    std::wcout << L"XplaneFederateHla1516e.cc -> getfromXplaneLocalHLATimeInformationModelData(): End" << std::endl;
    //#endif
}

// ----------------------------------------------------------------------------
// getfromXplaneLocalHLAEngineActuatorsModelData
/**
 * @brief XplaneFederateHla1516e::getfromXplaneLocalHLAEngineActuatorsModelData
 *          Update _LocalHLAEngineActuatorsModelData member with value inside Xplane.
 *
 *          SPECIFIC XPLANE
 */
void XplaneFederateHla1516e::getfromXplaneLocalHLAEngineActuatorsModelData()
{
//#ifdef DEBUG_HLA_XPLANE_REFLECT
//    std::wcout << L"XplaneFederateHla1516e.cc -> getfromXplaneLocalHLAEngineActuatorsModelData(): Start" << std::endl;
//#endif
#ifdef XPLANE_PLUGIN
    // THrottle
    float throttle[8];
    XPLMGetDatavf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrEnginesThrottleUsedRatio), &throttle[0], 0, 8);
    HLAfloat32LE prototype(0.0f);
    HLAfixedArray arrayThrottle(prototype, 8);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayThrottle.set(i, HLAfloat32LE(throttle[i]));
    }
    _LocalHLAEngineActuatorsModelData.set(0, arrayThrottle);
    // Thrust
    float thurst[8];
    XPLMGetDatavf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrEnginesThrust), &thurst[0], 0, 8);
    HLAfloat32LE prototype2(0.0f);
    HLAfixedArray arrayThrust(prototype2, 8);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayThrust.set(i, HLAfloat32LE(thurst[i]));
    }
    _LocalHLAEngineActuatorsModelData.set(1, arrayThrust);
    // N1
    float n1[8];
    XPLMGetDatavf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrEnginesN1Percent), &n1[0], 0, 8);
    HLAfloat32LE prototype3(0.0f);
    HLAfixedArray arrayN1(prototype3, 8);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayN1.set(i, HLAfloat32LE(n1[i]));
    }
    _LocalHLAEngineActuatorsModelData.set(2, arrayN1);
    // N2
    float n2[8];
    XPLMGetDatavf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrEnginesN2Percent), &n2[0], 0, 8);
    HLAfloat32LE prototype4(0.0f);
    HLAfixedArray arrayN2(prototype4, 8);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayN2.set(i, HLAfloat32LE(n2[i]));
    }
    _LocalHLAEngineActuatorsModelData.set(3, arrayN2);
    // EGT
    float egt[8];
    XPLMGetDatavf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrEnginesEGTDegreesCelsius), &egt[0], 0, 8);
    HLAfloat32LE prototype5(0.0f);
    HLAfixedArray arrayEGT(prototype5, 8);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayEGT.set(i, HLAfloat32LE(egt[i]));
    }
    _LocalHLAEngineActuatorsModelData.set(4, arrayEGT);
    // EPR
    float epr[8];
    XPLMGetDatavf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrEnginesEPRRatio), &epr[0], 0, 8);
    HLAfloat32LE prototype6(0.0f);
    HLAfixedArray arrayEPR(prototype6, 8);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayEPR.set(i, HLAfloat32LE(epr[i]));
    }
    _LocalHLAEngineActuatorsModelData.set(5, arrayEPR);
    // FF
    float ff[8];
    XPLMGetDatavf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrEnginesFuelFlowKgSec), &ff[0], 0, 8);
    HLAfloat32LE prototype7(0.0f);
    HLAfixedArray arrayFF(prototype7, 8);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayFF.set(i, HLAfloat32LE(ff[i]));
    }
    _LocalHLAEngineActuatorsModelData.set(6, arrayFF);
#endif
//#ifdef DEBUG_HLA_XPLANE_REFLECT
//    std::wcout << L"XplaneFederateHla1516e.cc -> getfromXplaneLocalHLAEngineActuatorsModelData(): End" << std::endl;
//#endif
}

/**
 * @brief XplaneFederateHla1516e::getfromXplaneLocalHLAControlCommandModelData
 *          Update _LocalHLAControlCommandModelData member with value inside Xplane.
 *
 *          SPECIFIC XPLANE
 */
void XplaneFederateHla1516e::getfromXplaneLocalHLAControlCommandModelData()
{
#ifdef XPLANE_PLUGIN
    HLAfloat32LE ElevatorDeflectionNormCommand(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrElevatorDeflectionNormCommand)));
    HLAfloat32LE AileronDeflectionNormCommand(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrAileronDeflectionNormCommand)));
    HLAfloat32LE RudderDeflectionNormCommand(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrRudderDeflectionNormCommand)));

    float throttle[8];
    XPLMGetDatavf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrEnginesThrottleUsedRatio), &throttle[0], 0, 8);
    HLAfloat32LE prototype(0.0f);
    HLAfixedArray arrayThrottle(prototype, 8);
    for(unsigned int i=0; i < 8; i++)
    {
        arrayThrottle.set(i, HLAfloat32LE(throttle[i]));
    }

    HLAfloat32LE StabilizerDeflectionDegreesCommand(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrStabilizerDeflectionDegreesCommand)));
    HLAboolean GearHandleStatusSwitchCommand(XPLMGetDatai(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrGearHandleStatusSwitchCommand)));
    HLAfloat32LE SpeedBrakesRatioCommand(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrSpeedBrakesRatioCommand)));
    HLAfloat32LE FlapsDeploymentRatioCommand(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrFlapsDeploymentRatioCommand)));
    HLAfloat32LE ParkingBrakesRatioCommand(XPLMGetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrParkingBrakesRatioCommand)));

    _LocalHLAControlCommandModelData.set(0, ElevatorDeflectionNormCommand);
    _LocalHLAControlCommandModelData.set(1, AileronDeflectionNormCommand);
    _LocalHLAControlCommandModelData.set(2, RudderDeflectionNormCommand);
    _LocalHLAControlCommandModelData.set(3, arrayThrottle);
    _LocalHLAControlCommandModelData.set(4, StabilizerDeflectionDegreesCommand);
    _LocalHLAControlCommandModelData.set(5, GearHandleStatusSwitchCommand);
    _LocalHLAControlCommandModelData.set(6, SpeedBrakesRatioCommand);
    _LocalHLAControlCommandModelData.set(7, FlapsDeploymentRatioCommand);
    _LocalHLAControlCommandModelData.set(8, ParkingBrakesRatioCommand);
#endif
}

/**
 * @brief XplaneFederateHla1516e::setToXplaneLocalHLAControlCommandModelData
 *          Update Xplane values from _LocalHLAControlCommandModelData member.
 *
 *          SPECIFIC XPLANE
 */
void XplaneFederateHla1516e::setToXplaneLocalHLAControlCommandModelData()
{
#ifdef XPLANE_PLUGIN
    XPLMSetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrElevatorDeflectionNormCommand), static_cast<const HLAfloat32LE&>(_LocalHLAControlCommandModelData.get(0)));
    XPLMSetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrAileronDeflectionNormCommand), static_cast<const HLAfloat32LE&>(_LocalHLAControlCommandModelData.get(1)));
    XPLMSetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrRudderDeflectionNormCommand), static_cast<const HLAfloat32LE&>(_LocalHLAControlCommandModelData.get(2)));
    HLAfixedArray arrayThrottle = static_cast<const HLAfixedArray&>(_LocalHLAControlCommandModelData.get(3));
    if(arrayThrottle.size() != kNbThrottle)
        throw rti1516e::EncoderException(L"The arrayThrottle is : " + std::to_wstring(arrayThrottle.size())
                                         + L" instead of " + std::to_wstring(kNbThrottle));
    float throttle[kNbThrottle];
    for(unsigned int i=0; i < arrayThrottle.size(); i++)
    {
         throttle[i] = static_cast<const HLAfloat32LE&>(arrayThrottle.get(i));
    }
    XPLMSetDatavf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrThrottleCommand), throttle, 0, kNbThrottle);
    XPLMSetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrStabilizerDeflectionDegreesCommand), static_cast<const HLAfloat32LE&>(_LocalHLAControlCommandModelData.get(4)));
    XPLMSetDatai(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrGearHandleStatusSwitchCommand), static_cast<const HLAboolean&>(_LocalHLAControlCommandModelData.get(5)));
    XPLMSetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrSpeedBrakesRatioCommand), static_cast<const HLAfloat32LE&>(_LocalHLAControlCommandModelData.get(6)));
    XPLMSetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrFlapsDeploymentRatioCommand), static_cast<const HLAfloat32LE&>(_LocalHLAControlCommandModelData.get(7)));
    XPLMSetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrParkingBrakesRatioCommand), static_cast<const HLAfloat32LE&>(_LocalHLAControlCommandModelData.get(8)));
#endif
}


/**
 * @brief XplaneFederateHla1516e::setToXplaneLocalPosition
 *          Update Xplane values from parameters
 * @param latitudeDeg Latitude in degrees
 * @param longitudeDeg Longitude in degrees
 * @param altitudeMSLMeters Altitude from the sea in meters
 *
 *          SPECIFIC XPLANE
 */
void XplaneFederateHla1516e::setToXplaneLocalPosition(double latitudeDeg, double longitudeDeg, double altitudeMSLMeters)
{
    double x = .0;
    double y = .0;
    double z = .0;
#ifdef XPLANE_PLUGIN
    XPLMWorldToLocal(latitudeDeg, longitudeDeg, altitudeMSLMeters, &x, &y, &z);

    XPLMSetDatad(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalX), x);
    XPLMSetDatad(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalY), y);
    XPLMSetDatad(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalZ), z);
#endif
}

/**
 * @brief XplaneFederateHla1516e::setToXplaneVolFormation
 *          Update Xplane values from parameters
 * @param numPlane Unique id of the plane necessary to choose position in formation flight
 * @param latitude Latitude of the master plane in degrees
 * @param longitude Longitude of the master plane in degrees
 * @param altitude Alitude of the master plane in metrs from the sea
 * @param distance Distance between own plane and the master plane
 * @param velocityX The X velocity in meter/sec
 * @param velocityY The Y velocity in meter/sec
 * @param velocityZ The Z velocity in meter/sec
 * @param cap ??
 * @param formation The type of the formation (Column, triangle, ...)
 */
void XplaneFederateHla1516e::setToXplaneVolFormation(const int &numPlane,
                                                     const double &latitude,
                                                     const double &longitude,
                                                     const double &altitude,
                                                     const double &distance,
                                                     const double &velocityX,
                                                     const double &velocityY,
                                                     const double &velocityZ,
                                                     const double &cap,
                                                     const Formation &formation)
{
    //En fonction du numéro du pilote et de la formation
    double longd=0;
    double latid=0;
//    const double convdst = 0.00001116;
    const double convdst = 1;
    //En Colonne
    if (formation == Formation::_column)
    {
         longd = longitude + ((numPlane)*convdst*distance*sin(cap*M_PI/180));
         latid = latitude + ((numPlane)*convdst*distance*cos(cap*M_PI/180));
    }
    //En Ligne
    else if(formation == Formation::_line)
    {
         longd = longitude + ((numPlane)*convdst*distance*cos(cap*M_PI/180));
         latid = latitude + ((numPlane)*convdst*distance*sin(cap*M_PI/180));
    }
    //En Echelon à droite
    else if (formation  == Formation::_rightEchelon)
    {
         longd = longitude + ((numPlane)*convdst*distance*sin((M_PI/4)+(cap*M_PI/180)));
         latid = latitude + ((numPlane)*convdst*distance*cos((M_PI/4)+(cap*M_PI/180)));
    }
    //En Echelon à gauche
    else if (formation  == Formation::_leftEchelon)
    {
         longd = longitude - ((numPlane)*convdst*distance*sin((M_PI/4)+(cap*M_PI/180)));
         latid = latitude + ((numPlane)*convdst*distance*cos((M_PI/4)+(cap*M_PI/180)));
    }
    //En Triangle
    else if (formation  == Formation::_triangular)
    {
         longd = longitude + (pow((-1),numPlane)*floor(numPlane/2)*convdst*distance*sin((M_PI/4)+(cap*M_PI/180)));
         latid = latitude - (floor(numPlane/2)*convdst*distance*cos((M_PI/4)+(cap*M_PI/180)));
    }

//    setToXplaneLocalPosition(latid, longd, altitude);

    double x = .0;
    double y = .0;
    double z = .0;
    #ifdef XPLANE_PLUGIN
        XPLMWorldToLocal(latid, longd, altitude, &x, &y, &z);

        XPLMSetDatad(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalX), x);
        XPLMSetDatad(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalY), y);
        XPLMSetDatad(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalZ), z);
        XPLMSetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalVxMetersps), velocityX);
        XPLMSetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalVyMetersps), velocityY);
        XPLMSetDataf(_XplaneDatarefs.at(XPLANE_DATAREFS_PATH::_DrLocalVzMetersps), velocityZ);

    #endif
}


// ----------------------------------------------------------------------------
// Print

/**
 * @brief XplaneFederateHla1516e::printMyHLAFlightDynamicsModelData
 *          Print on the terminal FlightDynamics Data
 */
void XplaneFederateHla1516e::printMyHLAFlightDynamicsModelData()
{
    HLAfloat64LE latitutdeDeg = static_cast<const HLAfloat64LE&>(_LocalHLAFlightDynamicsModelData.get(0));
    HLAfloat64LE longitudeDeg = static_cast<const HLAfloat64LE&>(_LocalHLAFlightDynamicsModelData.get(1));
    HLAfloat64LE altitudeMSLMeters = static_cast<const HLAfloat64LE&>(_LocalHLAFlightDynamicsModelData.get(2));
    HLAfloat32LE altitudeAGLMeters = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(3));
    HLAfloat32LE phiDeg = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(4));
    HLAfloat32LE thetaDeg = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(5));
    HLAfloat32LE psiDeg = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(6));
    HLAfloat32LE alphaDeg = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(7));
    HLAfloat32LE betaDeg = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(8));
    HLAfloat32LE hpathDeg = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(9));
    HLAfloat32LE vpathDeg = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(10));
    HLAfloat32LE pDegps = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(11));
    HLAfloat32LE qDegps = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(12));
    HLAfloat32LE rDegps = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(13));
    HLAfloat32LE trueAirspeedMetersps = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(14));
    HLAfloat32LE indicatedAirspeedKias = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(15));
    HLAfloat32LE groundspeedMetersps = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(16));
    HLAfloat32LE verticalspeedMetersps = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(17));
    HLAfloat32LE vMach = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(18));
    HLAfloat32LE rho = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(19));

    HLAfloat64LE localX = static_cast<const HLAfloat64LE&>(_LocalHLAFlightDynamicsModelData.get(20));
    HLAfloat64LE localY = static_cast<const HLAfloat64LE&>(_LocalHLAFlightDynamicsModelData.get(21));
    HLAfloat64LE localZ = static_cast<const HLAfloat64LE&>(_LocalHLAFlightDynamicsModelData.get(22));

    HLAfloat32LE localVxMetersps = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(23));
    HLAfloat32LE localVyMetersps = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(24));
    HLAfloat32LE localVzMetersps = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(25));
    HLAfloat32LE localAxMetersps2 = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(26));
    HLAfloat32LE localAyMetersps2 = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(27));
    HLAfloat32LE localAzMetersps2 = static_cast<const HLAfloat32LE&>(_LocalHLAFlightDynamicsModelData.get(28));

    std::cout << "latitutdeDeg : " << latitutdeDeg << "\t"
              << "longitudeDeg : " << longitudeDeg << "\n"
              << "altitudeMSLMeters : " << altitudeMSLMeters << "\t"
              << "altitudeAGLMeters : " << altitudeAGLMeters << "\n"
              << "phiDeg : " << phiDeg << "\t"
              << "thetaDeg : " << thetaDeg << "\n"
              << "psiDeg : " << psiDeg << "\t"
              << "alphaDeg : " << alphaDeg << "\n"
              << "betaDeg : " << betaDeg << "\t"
              << "hpathDeg : " << hpathDeg << "\n"
              << "vpathDeg : " << vpathDeg << "\t"
              << "pDegps : " << pDegps << "\n"
              << "qDegps : " << qDegps << "\t"
              << "rDegps : " << rDegps << "\n"
              << "trueAirspeedMetersps : " << trueAirspeedMetersps << "\t"
              << "indicatedAirspeedKias : " << indicatedAirspeedKias << "\n"
              << "groundspeedMetersps : " << groundspeedMetersps << "\t"
              << "verticalspeedMetersps : " << verticalspeedMetersps << "\n"
              << "vMach : " << vMach << "\t"
              << "rho : " << rho << "\n"
              << "localX : " << localX << "\t"
              << "localY : " << localY << "\t"
              << "localZ : " << localZ << "\n"
              << "localVxMetersps : " << localVxMetersps << "\t"
              << "localVyMetersps : " << localVyMetersps << "\n"
              << "localVzMetersps : " << localVzMetersps << "\t"
              << "localAxMetersps2 : " << localAxMetersps2 << "\n"
              << "localAyMetersps2 : " << localAyMetersps2 << "\t"
              << "localAzMetersps2 : " << localAzMetersps2 << std::endl;
}

/**
 * @brief XplaneFederateHla1516e::printMyHLAEngineActuatorsModelData
 *          print on the terminal Engine actuators data
 */
void XplaneFederateHla1516e::printMyHLAEngineActuatorsModelData()
{
    HLAfixedArray arrayThrottle = static_cast<const HLAfixedArray&>(_LocalHLAEngineActuatorsModelData.get(0));
    std::wcout << "Throttle : ";
    for(unsigned int i=0; i < arrayThrottle.size(); i++)
    {
         std::wcout << static_cast<const HLAfloat32LE&>(arrayThrottle.get(i)) << " ";
    }
    std::wcout << std::endl;

    HLAfixedArray arrayThurst = static_cast<const HLAfixedArray&>(_LocalHLAEngineActuatorsModelData.get(1));
    std::wcout << "Thurst : ";
    for(unsigned int i=0; i < arrayThurst.size(); i++)
    {
         std::wcout << static_cast<const HLAfloat32LE&>(arrayThurst.get(i)) << " ";
    }
    std::wcout << std::endl;

    HLAfixedArray arrayN1 = static_cast<const HLAfixedArray&>(_LocalHLAEngineActuatorsModelData.get(2));
    std::wcout << "N1 : ";
    for(unsigned int i=0; i < arrayN1.size(); i++)
    {
         std::wcout << static_cast<const HLAfloat32LE&>(arrayN1.get(i)) << " ";
    }
    std::wcout << std::endl;

    HLAfixedArray arrayN2 = static_cast<const HLAfixedArray&>(_LocalHLAEngineActuatorsModelData.get(3));
    std::wcout << "N2 : ";
    for(unsigned int i=0; i < arrayN2.size(); i++)
    {
         std::wcout << static_cast<const HLAfloat32LE&>(arrayN2.get(i)) << " ";
    }
    std::wcout << std::endl;

    HLAfixedArray arrayEGT = static_cast<const HLAfixedArray&>(_LocalHLAEngineActuatorsModelData.get(4));
    std::wcout << "EGT : ";
    for(unsigned int i=0; i < arrayEGT.size(); i++)
    {
         std::wcout << static_cast<const HLAfloat32LE&>(arrayEGT.get(i)) << " ";
    }
    std::wcout << std::endl;

    HLAfixedArray arrayEPR = static_cast<const HLAfixedArray&>(_LocalHLAEngineActuatorsModelData.get(5));
    std::wcout << "EPR : ";
    for(unsigned int i=0; i < arrayEPR.size(); i++)
    {
         std::wcout << static_cast<const HLAfloat32LE&>(arrayEPR.get(i)) << " ";
    }
    std::wcout << std::endl;

    HLAfixedArray arrayFF = static_cast<const HLAfixedArray&>(_LocalHLAEngineActuatorsModelData.get(6));
    std::wcout << "FF : ";
    for(unsigned int i=0; i < arrayFF.size(); i++)
    {
         std::wcout << static_cast<const HLAfloat32LE&>(arrayFF.get(i)) << " ";
    }
    std::wcout << std::endl;
}

/**
 * @brief XplaneFederateHla1516e::printMyHLAControlCommandModelData
 *              Print on the terminal control command data
 */
void XplaneFederateHla1516e::printMyHLAControlCommandModelData()
{

    std::cout << "_DrElevatorDeflectionNormCommand : " << static_cast<const HLAfloat32LE&>(_LocalHLAControlCommandModelData.get(0)) << std::endl;
    std::cout << "_DrAileronDeflectionNormCommand : " << static_cast<const HLAfloat32LE&>(_LocalHLAControlCommandModelData.get(1)) << std::endl;
    std::cout << "_DrRudderDeflectionNormCommand : " << static_cast<const HLAfloat32LE&>(_LocalHLAControlCommandModelData.get(2)) << std::endl;
    HLAfixedArray arrayThrottle = static_cast<const HLAfixedArray&>(_LocalHLAControlCommandModelData.get(3));

    std::cout << "Throttle : ";
    for(unsigned int i=0; i < arrayThrottle.size(); i++)
    {
         std::cout << "           " << static_cast<const HLAfloat32LE&>(arrayThrottle.get(i)) << std::endl;
    }

    std::cout << "_DrStabilizerDeflectionDegreesCommand : " << static_cast<const HLAfloat32LE&>(_LocalHLAControlCommandModelData.get(4)) << std::endl;
    std::cout << "_DrGearHandleStatusSwitchCommand : " << static_cast<const HLAboolean&>(_LocalHLAControlCommandModelData.get(5)) << std::endl;
    std::cout << "_DrSpeedBrakesRatioCommand : " << static_cast<const HLAfloat32LE&>(_LocalHLAControlCommandModelData.get(6)) << std::endl;
    std::cout << "_DrFlapsDeploymentRatioCommand : " << static_cast<const HLAfloat32LE&>(_LocalHLAControlCommandModelData.get(7)) << std::endl;
    std::cout << "_DrParkingBrakesRatioCommand : " << static_cast<const HLAfloat32LE&>(_LocalHLAControlCommandModelData.get(8)) << std::endl;

}

/**
 * @brief XplaneFederateHla1516e::printOtherPlanes
 *          Print other planes data (these data is used to draw other plane inside the simulator)
 */
void XplaneFederateHla1516e::printOtherPlanes()
{
    for(auto it=_RemoteAircraftsToDraw.begin(); it != _RemoteAircraftsToDraw.end(); ++it)
    {
        it->second.print();
        if(it != _RemoteAircraftsToDraw.end())
            std::cout << "--------------------------" << std::endl;
    }
}

/**
 * @brief XplaneFederateHla1516e::getOthersAircraftToDraw
 * @return a reference to the map _RemoteAircraftsToDraw which contain all other aircraft data.
 */
const std::map<std::wstring, RemoteAircraftToDraw> &XplaneFederateHla1516e::getOthersAircraftToDraw()
{
    return _RemoteAircraftsToDraw;
}

/**
 * @brief XplaneFederateHla1516e::getNbOtherAircraftToDraw
 * @return the number of other aircraft to draw
 */
unsigned int XplaneFederateHla1516e::getNbOtherAircraftToDraw()
{
    return  _RemoteAircraftsToDraw.size();
}

/**
 * @brief XplaneFederateHla1516e::connect
 *          Connect to the rtig. These method have to be called before all initilization step of the federate.
 * @param FederationName The name of the federation
 * @param FederateName The name of the federate
 * @param FomFileName The name of the fom
 */
void XplaneFederateHla1516e::connect(const std::wstring &FederationName, const std::wstring &FederateName, const std::wstring &FomFileName)
{
    _FederationName = FederationName;
    _FederateName = FederateName;
    _FomFileName = FomFileName;

    bool test = true;
    try
    {
        std::unique_ptr < rti1516e::RTIambassadorFactory > rtiAmbFact (new rti1516e::RTIambassadorFactory ());
        std::unique_ptr < rti1516e::RTIambassador > rtiAmbP (rtiAmbFact->createRTIambassador ());
        _RtiAmb = rtiAmbP.release ();
        std::wcout << L"* Ambassador created" << std::endl;
    }
    catch (rti1516e::Exception & e)
    {
        test = false;
        std:: wcout << L"\t->createAmbassador" << std::endl;
        std::wcout << L"* Error creating ambassador" << e.what() << std::endl;
    }

    if (test) {
    /* HLA Evolved requires to 'connect' to the RTI before using RTIAmb */
        try
        {
            _RtiAmb->connect((* this), rti1516e::HLA_EVOKED);
            std::wcout << L"* Ambassador connected" << std::endl;
        }
        catch (rti1516e::Exception& e)
        {
            std::wcout << L"RTIambassador connect caught Error " << e.what() <<std::endl;
        }
    }
}

// ----------------------------------------------------------------------------
// Send 
/**
 * @brief XplaneFederateHla1516e::sendUpdateAttributes
 *          Update attribute for members :
 *          - _LocalHLAFlightDynamicsModelData
 *          - _LocalHLAHydraulicActuatorsModelData
 *          - _LocalHLAEngineActuatorsModelData
 *          - _LocalHLATimeInformationModelData
 *          - _LocalHLAControlCommandModelData
 */
void XplaneFederateHla1516e::sendUpdateAttributes()
{
//#ifdef DEBUG_HLA_XPLANE_REFLECT
//    std::wcout << L"XplaneFederateHla1516e.cc -> sendUpdateAttributes(): Start" << std::endl;
//#endif
	//create AttributeHandleValueMap
	rti1516e::AttributeHandleValueMap ahvpsForAttrFlightDynamicsModelData;
	rti1516e::AttributeHandleValueMap ahvpsForAttrHydraulicActuatorsModelData;
	rti1516e::AttributeHandleValueMap ahvpsForAttrEngineActuatorsModelData;
	rti1516e::AttributeHandleValueMap ahvpsForAttrTimeInformationModelData;
    rti1516e::AttributeHandleValueMap ahvpsForAttrClassControlCommandModelData;
	
    //Encode Data
	ahvpsForAttrFlightDynamicsModelData[_AttributeHandleForAttrFlightDynamicsModelData] = _LocalHLAFlightDynamicsModelData.encode();
    ahvpsForAttrHydraulicActuatorsModelData[_AttributeHandleForAttrHydraulicActuatorsModelData] = _LocalHLAHydraulicActuatorsModelData.encode();
    ahvpsForAttrEngineActuatorsModelData[_AttributeHandleForAttrEngineActuatorsModelData] = _LocalHLAEngineActuatorsModelData.encode();
    ahvpsForAttrTimeInformationModelData[_AttributeHandleForAttrTimeInformationModelData] = _LocalHLATimeInformationModelData.encode();
    ahvpsForAttrClassControlCommandModelData[_AttributeHandleForAttrControlCommandModelData] = _LocalHLAControlCommandModelData.encode();

    //Tags
    std::wstring tag{L""}; // useless ... or maybe ...
    HLAunicodeString prototype(L"");
    HLAvariableArray nameFederates(prototype);
    int i = 0;
    for(auto it = _RemoteAircraftsToDraw.begin(); it != _RemoteAircraftsToDraw.end(); ++it)
    {
        nameFederates.addElement(HLAunicodeString(it->first));
        i++;
    }
    try
    {
		// If federate is regulator (HLA time management) it has to send data with timestamp
        if ( !_IsTimeReg )
        {
            _RtiAmb->updateAttributeValues(_ObjectInstanceHandleForClassFlightDynamicsModel, ahvpsForAttrFlightDynamicsModelData, {tag.c_str(), tag.size()});
            _RtiAmb->updateAttributeValues(_ObjectInstanceHandleForClassHydraulicActuatorsModel, ahvpsForAttrHydraulicActuatorsModelData, {tag.c_str(), tag.size()});
            _RtiAmb->updateAttributeValues(_ObjectInstanceHandleForClassEngineActuatorsModel, ahvpsForAttrEngineActuatorsModelData, {tag.c_str(), tag.size()});
            _RtiAmb->updateAttributeValues(_ObjectInstanceHandleForClassTimeInformationModel, ahvpsForAttrTimeInformationModelData, {tag.c_str(), tag.size()});
		}
		else
        {
            RTI1516fedTime updateTime = _LocalTime.getFedTime() + _Lookahead.getFedTime();
            _RtiAmb->updateAttributeValues(_ObjectInstanceHandleForClassFlightDynamicsModel, ahvpsForAttrFlightDynamicsModelData, {tag.c_str(), tag.size()}, updateTime);
            _RtiAmb->updateAttributeValues(_ObjectInstanceHandleForClassHydraulicActuatorsModel, ahvpsForAttrHydraulicActuatorsModelData, {tag.c_str(), tag.size()}, updateTime);
            _RtiAmb->updateAttributeValues(_ObjectInstanceHandleForClassEngineActuatorsModel, ahvpsForAttrEngineActuatorsModelData, {tag.c_str(), tag.size()}, updateTime);
            _RtiAmb->updateAttributeValues(_ObjectInstanceHandleForClassTimeInformationModel, ahvpsForAttrTimeInformationModelData, {tag.c_str(), tag.size()}, updateTime);
//            printMyHLAControlCommandModelData();
//            if(_IsCreator)
//                _RtiAmb->updateAttributeValues(_ObjectInstanceHandleForClassControlCommandModel, ahvpsForAttrClassControlCommandModelData, nameFederates.encode(), updateTime);
        }
    }
    catch ( rti1516e::Exception &e ) 
	{
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
	} 
	catch ( ... ) 
	{
        std::wcout  << "Error: unknown non-RTI exception." << std::endl;
    }
//#ifdef DEBUG_HLA_XPLANE_REFLECT
//    std::wcout << L"XplaneFederateHla1516e.cc -> sendUpdateAttributes(): End" << std::endl;
//#endif
}

/**
 * @brief XplaneFederateHla1516e::sendInteractionForcePosition
 *          To force position of another plane with the position of your own plane
 * @param planeName The name of the plane which position have to be changed
 */
void XplaneFederateHla1516e::sendInteractionForcePosition(const std::wstring &planeName)
{
    HLAunicodeString hlaPlaneName(planeName);
    HLAfloat64LE hlaLatitudeDeg = static_cast<const HLAfloat64LE&>(_LocalHLAFlightDynamicsModelData.get(0));
    HLAfloat64LE hlaLongitudeDeg = static_cast<const HLAfloat64LE&>(_LocalHLAFlightDynamicsModelData.get(1));
    HLAfloat64LE hlaAltitudeMSLMeters = static_cast<const HLAfloat64LE&>(_LocalHLAFlightDynamicsModelData.get(2));

    _InteractionForcePositionData.set(0, hlaLatitudeDeg);
    _InteractionForcePositionData.set(1, hlaLongitudeDeg);
    _InteractionForcePositionData.set(2, hlaAltitudeMSLMeters);

    ParameterHandleValueMap parameters;
    parameters[_ParameterHandleForInteractionPlaneNamePosition] = hlaPlaneName.encode();
    parameters[_ParameterHandleForInteractionForcePosition] = _InteractionForcePositionData.encode();

    _RtiAmb->sendInteraction(_InteractionHandleForForcePosition, parameters, hlaPlaneName.encode());
}

/**
 * @brief XplaneFederateHla1516e::sendInteractionForcePosition
 *          To force position of another plane with the position in parameters
 * @param latitude Latitude in degrees
 * @param longitude Longitude in degrees
 * @param altitude Altitude from the sea in meters
 * @param planeName The name of the plane which position have to be changed
 */
void XplaneFederateHla1516e::sendInteractionForcePosition(const double &latitude, const double &longitude, const double &altitude, const std::wstring &planeName)
{
    HLAunicodeString hlaPlaneName(planeName);
    HLAfloat64LE hlaLatitudeDeg(latitude);
    HLAfloat64LE hlaLongitudeDeg(longitude);
    HLAfloat64LE hlaAltitudeMSLMeters(altitude);

    _InteractionForcePositionData.set(0, hlaLatitudeDeg);
    _InteractionForcePositionData.set(1, hlaLongitudeDeg);
    _InteractionForcePositionData.set(2, hlaAltitudeMSLMeters);

    ParameterHandleValueMap parameters;
    parameters[_ParameterHandleForInteractionPlaneNamePosition] = hlaPlaneName.encode();
    parameters[_ParameterHandleForInteractionForcePosition] = _InteractionForcePositionData.encode();
    _RtiAmb->sendInteraction(_InteractionHandleForForcePosition, parameters, hlaPlaneName.encode());
}

/**
 * @brief XplaneFederateHla1516e::sendInteractionsVolFormation
 *          To force position of another plane with the position in parameters to follow a flight formation
 * @param latitude Latitude in degrees
 * @param longitude Longitude in degrees
 * @param altitude Altitude in meters from the sea
 * @param distance Distance with your own plane
 * @param velocityX The X velocity in meter/sec
 * @param velocityY  The Y velocity in meter/sec
 * @param velocityZ  The Z velocity in meter/sec
 * @param cap ??
 * @param formation The formation of the flight (Column, triangle, ...)
 */
void XplaneFederateHla1516e::sendInteractionsVolFormation(const double &latitude,
                                                          const double &longitude,
                                                          const double &altitude,
                                                          const double &distance,
                                                          const double &velocityX,
                                                          const double &velocityY,
                                                          const double &velocityZ,
                                                          const double &cap,
                                                          const Formation &formation)
{
    // Start other federate with 1 because 0 is for our position
    int numFederate = 1;
    for(auto it=_RemoteAircraftsToDraw.begin(); it!=_RemoteAircraftsToDraw.end(); ++it)
    {
        HLAunicodeString hlaPlaneName(it->first);

//        std::wcout << L"Plane name : " << hlaPlaneName.get() << L"Num federate : " << hlaNumFederate.get() << std::endl;

        _InteractionVolFormation.set(0, HLAfloat64LE(latitude));
        _InteractionVolFormation.set(1, HLAfloat64LE(longitude));
        _InteractionVolFormation.set(2, HLAfloat64LE(altitude));
        _InteractionVolFormation.set(3, HLAfloat64LE(distance));
        _InteractionVolFormation.set(4, HLAfloat32LE(velocityX));
        _InteractionVolFormation.set(5, HLAfloat32LE(velocityY));
        _InteractionVolFormation.set(6, HLAfloat32LE(velocityZ));
        _InteractionVolFormation.set(7, HLAfloat64LE(cap));
        _InteractionVolFormation.set(8, HLAinteger32LE(static_cast<int>(formation)));
        _InteractionVolFormation.set(9, HLAinteger32LE(numFederate));

        ParameterHandleValueMap parameters;
        parameters[_ParameterHandleForInteractionPlaneNameVolFormation] = hlaPlaneName.encode();
        parameters[_ParameterHandleForInteractionVolFormation] = _InteractionVolFormation.encode();
        _RtiAmb->sendInteraction(_InteractionHandleForVolFormation, parameters, hlaPlaneName.encode());
        numFederate++;
    }
}
/**
 * @brief XplaneFederateHla1516e::sendInteractionsVolFormation2
 *          To force position of another plane with the position in parameters to follow a flight formation with other plane reference
 * @param latitude Latitude in degrees
 * @param longitude Longitude in degrees
 * @param altitude Altitude in meters from the sea
 * @param distance Distance with your own plane
 * @param velocityX The X velocity in meter/sec
 * @param velocityY  The Y velocity in meter/sec
 * @param velocityZ  The Z velocity in meter/sec
 * @param cap ??
 * @param formation The formation of the flight (Column, triangle, ...)
 */
void XplaneFederateHla1516e::sendInteractionsVolFormation2(const double &latitude,
                                                          const double &longitude,
                                                          const double &altitude,
                                                          const double &distance,
                                                          const double &velocityX,
                                                          const double &velocityY,
                                                          const double &velocityZ,
                                                          const double &cap,
                                                          const Formation &formation)
{
    // Start other federate with 1 because 0 is for our position
    int numFederate = 1;
    for(auto it=_RemoteAircraftsToDraw.begin(); it!=_RemoteAircraftsToDraw.end(); ++it)
    {
        // set current plane to position 1 in formation
        if(numFederate == 1){
            setToXplaneVolFormation(numFederate, latitude, longitude, altitude, distance, velocityX, velocityY, velocityZ, cap, formation);
            numFederate++;
            continue;
        }
        HLAunicodeString hlaPlaneName(it->first);

//        std::wcout << L"Plane name : " << hlaPlaneName.get() << L"Num federate : " << hlaNumFederate.get() << std::endl;

        _InteractionVolFormation.set(0, HLAfloat64LE(latitude));
        _InteractionVolFormation.set(1, HLAfloat64LE(longitude));
        _InteractionVolFormation.set(2, HLAfloat64LE(altitude));
        _InteractionVolFormation.set(3, HLAfloat64LE(distance));
        _InteractionVolFormation.set(4, HLAfloat32LE(velocityX));
        _InteractionVolFormation.set(5, HLAfloat32LE(velocityY));
        _InteractionVolFormation.set(6, HLAfloat32LE(velocityZ));
        _InteractionVolFormation.set(7, HLAfloat64LE(cap));
        _InteractionVolFormation.set(8, HLAinteger32LE(static_cast<int>(formation)));
        _InteractionVolFormation.set(9, HLAinteger32LE(numFederate));

        ParameterHandleValueMap parameters;
        parameters[_ParameterHandleForInteractionPlaneNameVolFormation] = hlaPlaneName.encode();
        parameters[_ParameterHandleForInteractionVolFormation] = _InteractionVolFormation.encode();
        _RtiAmb->sendInteraction(_InteractionHandleForVolFormation, parameters, hlaPlaneName.encode());
        numFederate++;
    }
}

/**
 * @brief XplaneFederateHla1516e::runOneStep
 *          Use to advance in time. Realise a TimeAdvanceRequest and evoqueCallback while timeAdvanceGrant is not done
 */
void XplaneFederateHla1516e::runOneStep()
{
    //_RtiAmb->queryLogicalTime(_LocalTime);
//	std::wcout << L"XplaneFederateHla1516e.cc -> runOneStep(): Start" << std::endl;
    _IsTimeAdvanceGrant=false;

    RTI1516fedTime nextStep{_LocalTime.getFedTime() + _TimeStep.getFedTime()};
    _RtiAmb->timeAdvanceRequest(nextStep);
#ifdef DEBUG_HLA_XPLANE_TIME_STEP
    int nbCycle = 0;
    auto initTime = std::chrono::system_clock::now();
#endif
    while (!_IsTimeAdvanceGrant) {
        _RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
#ifdef DEBUG_HLA_XPLANE_TIME_STEP
        nbCycle++;
#endif
    }
#ifdef DEBUG_HLA_XPLANE_TIME_STEP
    auto now = std::chrono::system_clock::now();
    auto stepTime = duration_cast<microseconds>(now.time_since_epoch()).count() - duration_cast<microseconds>(initTime.time_since_epoch()).count();
    std::wcout << "nbCycle : " << nbCycle << "\tstepTime : " << stepTime << std::endl;
#endif
}

void XplaneFederateHla1516e::pause()
{
#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> pause(): Start" << std::endl;
#endif
    std::wstring InitString (L"Init");

    if (_IsCreator)
    {
        std::wcout << L">> CREATOR: PRESS START WHEN ALL FEDERATES ARE READY " << std::endl;
//        std::getchar();
        std::wcout << L"Pause requested per Creator " << std::endl;
        try
        {
            _RtiAmb->registerFederationSynchronizationPoint(InitString, _MySyncTag);
        }
        catch ( rti1516e::Exception &e )
        {
            std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
        }
        catch ( ... )
        {
            std::wcout  << "Error: unknown non-RTI exception." << std::endl;
        }

        while (_SyncRegSuccess && !_SyncRegFailed)
        {
            try
            {
                _RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
            }
            catch ( rti1516e::Exception &e )
            {
                std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
            }
            catch ( ... )
            {
                std::wcout  << "Error: unknown non-RTI exception." << std::endl;
            }
            std::wcout << L">> Waiting for success or failure of synchronisation point init. " << std::endl;
        }
        if(_SyncRegFailed)
        {
            std::wcout << L">> Error on initial Synchronization" << std::endl;
        }
    }

    while (!_IsSyncAnnonced)
    {
        std::wcout << L">> Waiting for synchronisation point Init announcement." << std::endl;
        try
        {
            _RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
        }
        catch ( rti1516e::Exception &e )
        {
            std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
        }
        catch ( ... )
        {
            std::wcout  << "Error: unknown non-RTI exception." << std::endl;
        }
    }
    try
    {
        _RtiAmb->synchronizationPointAchieved(InitString);
    }
    catch ( rti1516e::Exception &e )
    {
        std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
    }
    catch ( ... )
    {
        std::wcout  << "Error: unknown non-RTI exception." << std::endl;
    }
    std::wcout << L">> Init Synchronisation point satisfied." << std::endl;

    while (_InPause)
    {
        std::wcout << L">> Waiting for initialization phase." << std::endl ;
        try
        {
            _RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
        }
        catch ( rti1516e::Exception &e )
        {
            std::wcout << L"RTI Exception caught Error " << e.what() <<std::endl;
        }
        catch ( ... )
        {
            std::wcout  << "Error: unknown non-RTI exception." << std::endl;
        }
    }
#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> pause(): End" << std::endl;
#endif
}


// ----------------------------------------------------------------------------
// Callback : reflect attribute values without time
/**
 * @brief XplaneFederateHla1516e::reflectAttributeValues
 *          These callback is call when another federate use updateAttributeValue of the ambassador.
 * @param theObject The handle of the object concerned by the update of attributes values
 * @param theAttributes A map of attibute handle of the object
 * @param theUserSuppliedTag a tag
 * @param sentOrdering Not used
 * @param theTransport Not used
 * @param theReflectInfo Not used
 */
void XplaneFederateHla1516e::reflectAttributeValues( rti1516e::ObjectInstanceHandle theObject,
														rti1516e::AttributeHandleValueMap const &theAttributes,
														rti1516e::VariableLengthData const &theUserSuppliedTag,
                                                        rti1516e::OrderType /*sentOrdering*/,
                                                        rti1516e::TransportationType /*theTransport*/,
                                                        rti1516e::SupplementalReflectInfo /*theReflectInfo*/
														)
                                                        throw ( rti1516e::FederateInternalError)
{
#ifdef DEBUG_HLA_XPLANE_REFLECT
    std::wcout << L"XplaneFederateHla1516e.cc -> reflectAttributeValues(): Start" << std::endl;
#endif

    auto itObjectInstanceInfo = _ObjectsInstanceInfo.find(theObject);
    if( itObjectInstanceInfo != _ObjectsInstanceInfo.end() && itObjectInstanceInfo->second.type == TypeObjectHandle::FlightDynamicsModel)
    {
        for (auto it = theAttributes.begin (); it != theAttributes.end (); ++it)
        {
            _RemoteHLAFlightDynamicsModelData.decode(it->second);
            _RemoteAircraftsToDraw.at(itObjectInstanceInfo->second.federateName).RefreshFromHLAFlightDynamicsModelData(_RemoteHLAFlightDynamicsModelData);
        }
    }
    else if( itObjectInstanceInfo != _ObjectsInstanceInfo.end() && itObjectInstanceInfo->second.type == TypeObjectHandle::EngineActuators)
    {
        for (auto it = theAttributes.begin (); it != theAttributes.end (); ++it)
        {
            _RemoteHLAEngineActuatorsModelData.decode(it->second);
            _RemoteAircraftsToDraw.at(itObjectInstanceInfo->second.federateName).RefreshFromHLAEngineActuatorsModelData(_RemoteHLAEngineActuatorsModelData);
        }
    }
    else if( itObjectInstanceInfo != _ObjectsInstanceInfo.end() && itObjectInstanceInfo->second.type == TypeObjectHandle::ControlCommand)
    {
        HLAunicodeString prototype(L"");
        HLAvariableArray planeNames(prototype);
        planeNames.decode(theUserSuppliedTag);
        bool nameFinded{false};
        for(unsigned int i=0; i < planeNames.size() && !nameFinded; i++)
        {
            if(_FederateName == static_cast<const HLAunicodeString&>(planeNames.get(i)).get())
            {
                for (auto it = theAttributes.begin (); it != theAttributes.end (); ++it)
                {
                    _LocalHLAControlCommandModelData.decode(it->second);
                    setToXplaneLocalHLAControlCommandModelData();
                    nameFinded = true;
                }
            }
        }
    }

#ifdef DEBUG_HLA_XPLANE_REFLECT
    std::wcout << L"XplaneFederateHla1516e.cc -> reflectAttributeValues(): End" << std::endl;
#endif
}

void XplaneFederateHla1516e::reflectAttributeValues(ObjectInstanceHandle theObject
                                                    , const AttributeHandleValueMap &theAttributes
                                                    , const VariableLengthData &theUserSuppliedTag
                                                    , OrderType sentOrdering
                                                    , TransportationType theTransport
                                                    , const LogicalTime &/*theTime*/
                                                    , OrderType /*receivedOrdering*/
                                                    , MessageRetractionHandle /*theHandle*/
                                                    , SupplementalReflectInfo theReflectInfo)
throw ( rti1516e::FederateInternalError)
{
    reflectAttributeValues(theObject, theAttributes, theUserSuppliedTag, sentOrdering, theTransport, theReflectInfo);
}

void XplaneFederateHla1516e::reflectAttributeValues(ObjectInstanceHandle theObject,
                                                    const AttributeHandleValueMap &theAttributeValues,
                                                    const VariableLengthData &theUserSuppliedTag,
                                                    OrderType sentOrder,
                                                    TransportationType theType,
                                                    const LogicalTime &/*theTime*/,
                                                    OrderType /*receivedOrder*/,
                                                    SupplementalReflectInfo theReflectInfo)
throw ( rti1516e::FederateInternalError)
{
    reflectAttributeValues(theObject, theAttributeValues, theUserSuppliedTag, sentOrder, theType, theReflectInfo);
}

/**
 * @brief XplaneFederateHla1516e::receiveInteraction
 *          These callback is call when another federate call sendInteraction method of the ambassador
 * @param theInteraction The handle of the interaction
 * @param theParameterValues A map which contain all parameter handles whaich have to be change
 * @param theUserSuppliedTag Not use
 * @param sentOrder Not use
 * @param theType Not use
 * @param theReceiveInfo Not use
 */
void XplaneFederateHla1516e::receiveInteraction(InteractionClassHandle theInteraction,
                                                const ParameterHandleValueMap &theParameterValues,
                                                const VariableLengthData &/*theUserSuppliedTag*/,
                                                OrderType /*sentOrder*/,
                                                TransportationType /*theType*/,
                                                SupplementalReceiveInfo /*theReceiveInfo*/)
throw ( rti1516e::FederateInternalError)
{
#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> receiveInteraction(): Start" << std::endl;
#endif
    std::wcout << L"theInteraction : " << theInteraction << " - _InteractionHandleForForcePosition : " << _InteractionHandleForForcePosition << std::endl;
    if(theInteraction == _InteractionHandleForForcePosition)
    {
        std::wcout << L"Interaction : _InteractionHandleForForcePosition" << std::endl;
        auto planeNameParam = theParameterValues.at(_ParameterHandleForInteractionPlaneNamePosition);
        HLAunicodeString planeName;
        planeName.decode(planeNameParam);
        std::wcout << L"Plane name : " << planeName.get() << std::endl;

        _InteractionForcePositionData.decode(theParameterValues.at(_ParameterHandleForInteractionForcePosition));
        HLAfloat64LE hlaLatitude = static_cast<const HLAfloat64LE&>(_InteractionForcePositionData.get(0));
        std::wcout << L"hlaLatitude : " << hlaLatitude.get() << std::endl;
        HLAfloat64LE hlaLongitude = static_cast<const HLAfloat64LE&>(_InteractionForcePositionData.get(1));
        std::wcout << L"hlaLongitude : " << hlaLongitude.get() << std::endl;
        HLAfloat64LE hlaAltitude = static_cast<const HLAfloat64LE&>(_InteractionForcePositionData.get(2));
        std::wcout << L"hlaAltitude : " << hlaAltitude.get() << std::endl;

        if(planeName.get() == L"*" || planeName.get() == _FederateName) {
            setToXplaneLocalPosition(hlaLatitude, hlaLongitude, hlaAltitude);
        }
    }
    else if(theInteraction == _InteractionHandleForVolFormation)
    {
        std::wcout << L"Interaction : _InteractionHandleForVolFormation" << std::endl;
        auto planeIdentityParam = theParameterValues.at(_ParameterHandleForInteractionPlaneNameVolFormation);

        HLAunicodeString hlaPlaneName(L"");
        std::wcout << L"BEFORE DECODE" << std::endl;
        hlaPlaneName.decode(planeIdentityParam);
        std::wcout << L"AFTER DECODE" << std::endl;

        std::wcout << L"Plane name : " << hlaPlaneName.get() << std::endl;

        _InteractionVolFormation.decode(theParameterValues.at(_ParameterHandleForInteractionVolFormation));
        HLAfloat64LE hlaLatitude = static_cast<const HLAfloat64LE&>(_InteractionVolFormation.get(0));
        std::wcout << L"hlaLatitude : " << hlaLatitude.get() << std::endl;
        HLAfloat64LE hlaLongitude = static_cast<const HLAfloat64LE&>(_InteractionVolFormation.get(1));
        std::wcout << L"hlaLongitude : " << hlaLongitude.get() << std::endl;
        HLAfloat64LE hlaAltitude = static_cast<const HLAfloat64LE&>(_InteractionVolFormation.get(2));
        std::wcout << L"hlaAltitude : " << hlaAltitude.get() << std::endl;
        HLAfloat64LE hlaDistance = static_cast<const HLAfloat64LE&>(_InteractionVolFormation.get(3));
        std::wcout << L"hlaDistance : " << hlaDistance.get() << std::endl;
        HLAfloat32LE hlaVelocityX = static_cast<const HLAfloat32LE&>(_InteractionVolFormation.get(4));
        std::wcout << L"hlaVelocity : " << hlaVelocityX.get() << std::endl;
        HLAfloat32LE hlaVelocityY = static_cast<const HLAfloat32LE&>(_InteractionVolFormation.get(5));
        std::wcout << L"hlaVelocity : " << hlaVelocityY.get() << std::endl;
        HLAfloat32LE hlaVelocityZ = static_cast<const HLAfloat32LE&>(_InteractionVolFormation.get(6));
        std::wcout << L"hlaVelocity : " << hlaVelocityZ.get() << std::endl;
        HLAfloat64LE hlaCap = static_cast<const HLAfloat64LE&>(_InteractionVolFormation.get(7));
        std::wcout << L"hlaCap : " << hlaCap.get() << std::endl;
        HLAinteger32LE hlaFormation = static_cast<const HLAinteger32LE&>(_InteractionVolFormation.get(8));
        std::wcout << L"hlaFormation : " << hlaFormation.get() << std::endl;
        HLAinteger32LE hlaNumPlane = static_cast<const HLAinteger32LE&>(_InteractionVolFormation.get(9));
        std::wcout << L"hlaNumPlane : " << hlaNumPlane.get() << std::endl;
//        int hlaNumPlane = 1;
//        std::wstring hlaFormation(L"test");
        if(hlaPlaneName.get() == _FederateName) {
            setToXplaneVolFormation(hlaNumPlane, hlaLatitude, hlaLongitude, hlaAltitude, hlaDistance, hlaVelocityX, hlaVelocityY, hlaVelocityZ, hlaCap, static_cast<Formation>(hlaFormation.get()));
        }

    }
#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> receiveInteraction(): End" << std::endl;
#endif
}


void XplaneFederateHla1516e::receiveInteraction(InteractionClassHandle theInteraction,
                                                const ParameterHandleValueMap &theParameterValues,
                                                const VariableLengthData &theUserSuppliedTag,
                                                OrderType sentOrder,
                                                TransportationType theType,
                                                const LogicalTime &/*theTime*/,
                                                OrderType /*receivedOrder*/,
                                                MessageRetractionHandle /*theHandle*/,
                                                SupplementalReceiveInfo theReceiveInfo)
throw ( rti1516e::FederateInternalError)
{
    receiveInteraction(theInteraction, theParameterValues, theUserSuppliedTag, sentOrder, theType, theReceiveInfo);
}

void XplaneFederateHla1516e::receiveInteraction(InteractionClassHandle theInteraction,
                                                const ParameterHandleValueMap &theParameterValues,
                                                const VariableLengthData &theUserSuppliedTag,
                                                OrderType sentOrder,
                                                TransportationType theType,
                                                const LogicalTime &/*theTime*/,
                                                OrderType /*receivedOrder*/,
                                                SupplementalReceiveInfo theReceiveInfo)
                                                throw ( rti1516e::FederateInternalError)
{
    receiveInteraction(theInteraction, theParameterValues, theUserSuppliedTag, sentOrder, theType, theReceiveInfo);
}

/**
 * @brief XplaneFederateHla1516e::timeRegulationEnabled
 *      These callback is called when time reugulation is enabled. Necessary to do a synchronisation point
 */
void XplaneFederateHla1516e::timeRegulationEnabled(const LogicalTime &/*theTime*/)
throw (rti1516e::FederateInternalError)
{
    _IsTimeReg = true;
}

/**
 * @brief XplaneFederateHla1516e::timeConstrainedEnabled
 *          These callback is called when time constrained is enabled. Necessary to do a synchronisation point
 * @param theTime Not used
 */
void XplaneFederateHla1516e::timeConstrainedEnabled(const LogicalTime &/*theTime*/)
throw (rti1516e::FederateInternalError)
{
    _IsTimeConst = true;
}

/**
 * @brief XplaneFederateHla1516e::timeAdvanceGrant
 *          These callback is called when time advance is granted
 * @param theTime The time of the federate
 */
void XplaneFederateHla1516e::timeAdvanceGrant(const LogicalTime &theTime)
throw (rti1516e::FederateInternalError)
{
    _LocalTime = theTime;
    _IsTimeAdvanceGrant = true;
}

/**
 * @brief XplaneFederateHla1516e::enableTimeRegulation
 *          Enable the time regulation. is necessary to do a synchronisation point
 */
void XplaneFederateHla1516e::enableTimeRegulation()
{
#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> enableTimeRegulation(): Start" << std::endl;
#endif
    RTI1516fedTimeInterval lookahead(_Lookahead.getFedTime());
    _RtiAmb->enableTimeRegulation(lookahead);
    while (!_IsTimeReg)
    {
            _RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
    }
#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> enableTimeRegulation(): End" << std::endl;
#endif
}

/**
 * @brief XplaneFederateHla1516e::enableTimeConstrained
 *          Enable the time constrained. is necessary to do a synchronisation point
 */
void XplaneFederateHla1516e::enableTimeConstrained()
{
#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> enableTimeConstrained(): Start" << std::endl;
#endif
    _RtiAmb->enableTimeConstrained();

    while (!_IsTimeConst)
    {
        _RtiAmb->evokeCallback(std::numeric_limits<double>::infinity());
    }
#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> enableTimeConstrained(): End" << std::endl;
#endif
}

/**
 * @brief XplaneFederateHla1516e::enableAsynchronousDelivery
 *          Enable the asynchronous delivery. To advance in time without to consider all federate time after a synchronisation point.
 */
void XplaneFederateHla1516e::enableAsynchronousDelivery()
{
#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> enableAsynchronousDelivery(): Start" << std::endl;
#endif
    _RtiAmb->enableAsynchronousDelivery();
#ifdef DEBUG_HLA_XPLANE_FED
    std::wcout << L"XplaneFederateHla1516e.cc -> enableAsynchronousDelivery(): End" << std::endl;
#endif
}

/**
 * @brief XplaneFederateHla1516e::disableTimeRegulation
 */
void XplaneFederateHla1516e::disableTimeRegulation()
{
    _RtiAmb->disableTimeRegulation();
    _IsTimeReg = false;
}

/**
 * @brief XplaneFederateHla1516e::disableTimeConstrained
 */
void XplaneFederateHla1516e::disableTimeConstrained()
{
    _RtiAmb->disableTimeConstrained();
    _IsTimeConst = false;
}

/**
 * @brief XplaneFederateHla1516e::disableAsynchronousDelivery
 */
void XplaneFederateHla1516e::disableAsynchronousDelivery()
{
    _RtiAmb->disableAsynchronousDelivery();
}

/**
 * @brief XplaneFederateHla1516e::synchronizationPointRegistrationSucceeded
 *          These callback is called when synchronisation point is successfull
 * @param label
 */
void XplaneFederateHla1516e::synchronizationPointRegistrationSucceeded(const std::wstring &label)
throw (rti1516e::FederateInternalError)
{
    _SyncRegSuccess = true;
    std::wcout << L"Successfully registered sync point: " << label << std::endl;
}

/**
 * @brief XplaneFederateHla1516e::synchronizationPointRegistrationFailed
 *          These callback is called when synchronisation point is fail
 * @param label
 * @param reason
 */
void XplaneFederateHla1516e::synchronizationPointRegistrationFailed(const std::wstring &label, SynchronizationPointFailureReason reason)
throw (rti1516e::FederateInternalError)
{
    _SyncRegFailed = true;
    std::wcout << L"Failed to register sync point: " << label << L" -> " << reason << std::endl;
}


/**
 * @brief XplaneFederateHla1516e::announceSynchronizationPoint
 * @param label
 * @param theUserSuppliedTag Not used
 */
void XplaneFederateHla1516e::announceSynchronizationPoint(const std::wstring &label, const VariableLengthData &/*theUserSuppliedTag*/)
throw (rti1516e::FederateInternalError)
{
    _InPause = true ;
    _IsSyncAnnonced = true;
     std::wcout << L"Successfully announceSynchronizationPoint: " << label << std::endl;
}

/**
 * @brief XplaneFederateHla1516e::federationSynchronized
 * @param label
 * @param failedToSyncSet Not used
 */
void XplaneFederateHla1516e::federationSynchronized(const std::wstring &label, const FederateHandleSet &/*failedToSyncSet*/)
throw (rti1516e::FederateInternalError)
{
    _InPause = false ;
    std::wcout << L"Successfully federationSynchronized: " << label << std::endl;
}



