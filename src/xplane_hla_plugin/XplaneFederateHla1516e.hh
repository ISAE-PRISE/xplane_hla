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


#ifndef HLA_XPLANE_FEDERATE_HH_DEF
#define HLA_XPLANE_FEDERATE_HH_DEF

// System includes
#include <string>
#include <sstream>
#include <iostream>
#include <stdint.h>
#include <memory>
#include <vector>
#include <iterator>
#include <assert.h>
#include <time.h>
#include <limits>
#include <assert.h>

// RTI includes
#include <RTI/RTI1516.h>
#include <RTI/Enums.h>
#include <RTI/NullFederateAmbassador.h>
#include <RTI/RTI1516fedTime.h>
#include <RTI/LogicalTime.h>
#include <RTI/encoding/HLAfixedRecord.h>
#include <RTI/encoding/HLAfixedArray.h>
#include <RTI/encoding/BasicDataElements.h>

#ifdef XPLANE_PLUGIN
// Xplane SDK Include
#include "XPLMDataAccess.h"
#include "XPLMGraphics.h"
#endif
#ifdef AEROFLY_PLUGIN
#include <tm_external_message.h>
#endif

// FOr remote aircrafts
#include <RemoteAircraftToDraw.hh>

using namespace rti1516e;

enum class Formation {
    _column,
    _line,
    _rightEchelon,
    _leftEchelon,
    _triangular
};

namespace XPLANE_DATAREFS_PATH {
	
	// HLAFlightDynamicsModelData
    const std::string _DrLongitudeDeg = "sim/flightmodel/position/longitude";
    const std::string _DrLatitudeDeg = "sim/flightmodel/position/latitude";
    const std::string _DrAltitudeMSLMeters = "sim/flightmodel/position/elevation";
    const std::string _DrAltitudeAGLMeters = "sim/flightmodel/position/y_agl";

    const std::string _DrPhiDeg = "sim/flightmodel/position/phi";
    const std::string _DrThetaDeg = "sim/flightmodel/position/theta";
    const std::string _DrPsiDeg = "sim/flightmodel/position/psi";

    const std::string _DrAlphaDeg = "sim/flightmodel/position/alpha";
    const std::string _DrBetaDeg = "sim/flightmodel/position/beta";
    const std::string _DrHpathDeg = "sim/flightmodel/position/h";
    const std::string _DrVpathDeg = "sim/flightmodel/position/v";

    const std::string _DrPDegps = "sim/flightmodel/position/P";
    const std::string _DrQDegps = "sim/flightmodel/position/Q";
    const std::string _DrRDegps = "sim/flightmodel/position/R";

    const std::string _DrTrueAirspeedMetersps = "sim/flightmodel/position/true_airspeed";
    const std::string _DrindicatedAirspeedKias = "sim/flightmodel/position/indicated_airspeed";
    const std::string _DrGroundspeedMetersps = "sim/flightmodel/position/groundspeed";
    const std::string _DrVerticalspeedMetersps = "sim/flightmodel/position/vh_ind";
    const std::string _DrVmach = "sim/flightmodel/misc/machno";

    const std::string _DrRho = "sim/weather/rho";

    const std::string _DrLocalX = "sim/flightmodel/position/local_x";
    const std::string _DrLocalY = "sim/flightmodel/position/local_y";
    const std::string _DrLocalZ = "sim/flightmodel/position/local_z";

    const std::string _DrLocalVxMetersps = "sim/flightmodel/position/local_vx";
    const std::string _DrLocalVyMetersps = "sim/flightmodel/position/local_vy";
    const std::string _DrLocalVzMetersps = "sim/flightmodel/position/local_vz";

    const std::string _DrLocalAxMetersps2 = "sim/flightmodel/position/local_ax";
    const std::string _DrLocalAyMetersps2 = "sim/flightmodel/position/local_ay";
    const std::string _DrLocalAzMetersps2 = "sim/flightmodel/position/local_az";
    
    // HlaControlCommandModelData
//    const std::string _DrElevatorDeflectionNormCommand = "sim/joystick/FC_ptch";
//    const std::string _DrAileronDeflectionNormCommand = "sim/joystick/FC_roll";
//    const std::string _DrRudderDeflectionNormCommand = "sim/joystick/FC_hdng";
    const std::string _DrElevatorDeflectionNormCommand = "sim/joystick/yoke_pitch_ratio";
    const std::string _DrAileronDeflectionNormCommand = "sim/joystick/yoke_roll_ratio";
    const std::string _DrRudderDeflectionNormCommand = "sim/joystick/yoke_heading_ratio";
    // const std::string _DrThrottleCommand = "sim/flightmodel/engine/ENGN_thro"; // Throttle (per engine) as set by user, 0 = idle, 1 = max
    const std::string _DrThrottleCommand = "sim/flightmodel/engine/ENGN_thro_use"; // float[8], Throttle (per engine) when overridden by you, plus with thrust vectors - use override_throttles to change.
    const std::string _DrStabilizerDeflectionDegreesCommand = "sim/flightmodel2/controls/stabilizer_deflection_degrees"; // float, This is the actual stabilizer deflection with trim for all-moving horizontal stabilizers. This is the deflection you can see visually on airliners. This is in degrees, positive for leading-edge nose up.
	const std::string _DrGearHandleStatusSwitchCommand = "sim/cockpit/switches/gear_handle_status"; // boolean, Gear handle is up or down?
	const std::string _DrSpeedBrakesRatioCommand = "sim/flightmodel2/controls/speedbrake_ratio"; // FLoat, This is how much the speedbrakes surfaces are extended, in ratio, where 0.0 is fully retracted, and 1.0 is fully extended.
    const std::string _DrFlapsDeploymentRatioCommand = "sim/flightmodel2/controls/flap_handle_deploy_ratio"; // float	y	ratio	This is the ACTUAL FLAP deployment for overall flap system, in ratio, where 0.0 is flaps fully retracted, and 1.0 is flaps fully extended. You should probably use the deployment for flap set 1 or flap set 2 to deflect the surfaces though. This takes into account that flaps deploy slowly, not instantaneously as the handle is dragged.
    const std::string _DrParkingBrakesRatioCommand = "sim/flightmodel/controls/parkbrake"; // float	y	[0..1]	Parking Brake, 1 = max
    
    // Hydraulics Actuators Datarefs - Sounds tricky
    // https://developer.x-plane.com/2013/05/using-the-right-wing-datarefs/
    // http://www.xsquawkbox.net/xpsdk/mediawiki/Sim/flightmodel2/wing/#Wing_to_Array_Index_Table
    const std::string _DrLeftAileronEffectiveDeflectionDegrees = "sim/flightmodel2/wing/aileron1_deg[0]";
    const std::string _DrRightAileronEffectiveDeflectionDegrees = "sim/flightmodel2/wing/aileron1_deg[1]";
    const std::string _DrLeftElevatorEffectiveDeflectionDegrees = "sim/flightmodel2/wing/elevator1_deg[8]";
    const std::string _DrRightElevatorEffectiveDeflectionDegrees = "sim/flightmodel2/wing/elevator1_deg[9]";
    const std::string _DrHorizontalStabilizerEffectiveDeflectionDegrees = "sim/flightmodel2/controls/stabilizer_deflection_degrees"; // incorrectcorrect?
	const std::string _DrFlapsEffectiveDeflectionDegrees = "sim/flightmodel2/wing/flap1_deg[0]"; // Has to be double checked
	const std::string _DrRudderEffectiveDeflectionDegrees = "sim/flightmodel2/wing/rudder1_deg[10]"; // sim/flightmodel2/wing/rudder2_deg ??
	const std::string _DrLeftSpeedbrakesEffectiveDeflectionDegrees = "sim/flightmodel2/wing/speedbrake1_deg[0]";
	const std::string _DrRightSpeedbrakesEffectiveDeflectionDegrees = "sim/flightmodel2/wing/speedbrake1_deg[1]";
	const std::string _DrLeftSpoilersEffectiveDeflectionDegrees = "sim/flightmodel2/wing/spoiler1_deg[0]";
	const std::string _DrRightSpoilersEffectiveDeflectionDegrees = "sim/flightmodel2/wing/spoiler1_deg[1]";
	const std::string _DrGearsEffectiveDeploymentRatio = "sim/flightmodel2/gear/deploy_ratio";
    
    // Engine Actuators Datarefs
//    const std::string _DrEnginesThrottleUsedRatio = "sim/flightmodel2/engine/throttle_used_ratio";
    const std::string _DrEnginesThrottleUsedRatio = "sim/flightmodel/engine/ENGN_thro";
    const std::string _DrEnginesThrust = "sim/flightmodel/engine/POINT_thrust"; // 	float[8], Engines thrust vector can be either sim/flightmodel2/controls/thrust_vector_ratio
    const std::string _DrEnginesN1Percent = "sim/cockpit2/engine/indicators/N1_percent";
    const std::string _DrEnginesN2Percent = "sim/cockpit2/engine/indicators/N2_percent";
    const std::string _DrEnginesEGTDegreesCelsius = "sim/cockpit2/engine/indicators/EGT_deg_C";
	const std::string _DrEnginesEPRRatio = "sim/cockpit2/engine/indicators/EPR_ratio";
	const std::string _DrEnginesFuelFlowKgSec = "sim/cockpit2/engine/indicators/fuel_flow_kg_sec";
	

    const std::string _DrDateDays = "sim/time/local_date_days"; // Date in days since January 1st
    const std::string _DrLocalTimeSec = "sim/time/local_time_sec"; // Local time (seconds since midnight??)
    //const std::string _DrFrameRateFps = "sim/time/framerate_period"; // Xplane Frame Rate Not writable

    // For overriding Xplane built-in Models
    const std::string _DrOverRideArtStab = "sim/operation/override/override_artstab"; // Override control of the artificial stability system
    const std::string _DrOverRideFlightControl = "sim/operation/override/override_flightcontrol"; // Override all parts of the flight system at once
    const std::string _DrOverRidePlanePath = "sim/operation/override/override_planepath"; // Override position updates of this plane
    const std::string _DrOverRideThrottle = "sim/operation/override/override_throttles"; // Override the throttles. Use ENGN_thro_use to control them
    const std::string _DrOverRideGearBrake = "sim/operation/override/override_gearbrake"; // Override gear and brake status
    const std::string _DrOverRideAutopilot = "sim/operation/override/override_autopilot"; // Override the autopilot's brains
    const std::string _DrOverRideJoystick = "sim/operation/override/override_joystick"; // Override control of the joystick deflections (overrides stick, yoke, pedals, keys, mouse, and auto-coordination) 
}

enum class TypeObjectHandle
{
    FlightDynamicsModel,
    HydraulicActuator,
    EngineActuators,
    TimeInformation,
    ControlCommand,
    Unknown
};

struct ObjectInstanceInfo
{
    std::wstring federateName = L"";
    TypeObjectHandle type = TypeObjectHandle::Unknown;
};

class XplaneFederateHla1516e : public rti1516e::NullFederateAmbassador 
{
	// Private  elements
	private:
	
		// Internal model
#ifdef XPLANE_PLUGIN
		std::map<std::string, XPLMDataRef> _XplaneDatarefs;
#endif
        std::map<ObjectInstanceHandle, ObjectInstanceInfo> _ObjectsInstanceInfo;
        std::map<std::wstring, RemoteAircraftToDraw> _RemoteAircraftsToDraw;
	
		rti1516e::RTIambassador*    _RtiAmb;
		rti1516e::FederateHandle _FedHandle;

        std::wstring          _FederateName;
        std::wstring          _FederationName;
        std::wstring          _FomFileName;	    
	    std::wstring          _TrimSyncPointName;
        std::wstring          _SimSyncPointName;
	    
	    rti1516e::FederateHandle _FederateHandle ;
	
		rti1516e::VariableLengthData _MyTag;
		rti1516e::VariableLengthData _MySyncTag;
		
		bool _IsTimeReg;
		bool _IsTimeConst;
		bool _IsTimeAdvanceGrant;
		bool _IsOutMesTimespamped;

		RTI1516fedTime _TimeStep;   
		RTI1516fedTime _Lookahead;   
		RTI1516fedTime _RestOfTimeStep; 			
		RTI1516fedTime _LocalTime;
		RTI1516fedTime _SimulationEndTime;

        bool _IsInitialized{false};
		bool _SyncRegSuccess;
		bool _SyncRegFailed;
		bool _InPause;
		bool _IsCreator;
        bool _IsSyncAnnonced;
		
		// DataTypes to transmit the data
		HLAfixedRecord _LocalHLAFlightDynamicsModelData;
		HLAfixedRecord _LocalHLAHydraulicActuatorsModelData;
		HLAfixedRecord _LocalHLAEngineActuatorsModelData;
		HLAfixedRecord _LocalHLATimeInformationModelData;
		// DataTypes to receive data from others
        HLAfixedRecord _RemoteHLAFlightDynamicsModelData;
        HLAfixedRecord _RemoteHLAHydraulicActuatorsModelData;
        HLAfixedRecord _RemoteHLAEngineActuatorsModelData;
        HLAfixedRecord _RemoteHLATimeInformationModelData;

		HLAfixedRecord _LocalHLAControlCommandModelData;

        // DataTypes to send force position oh others planes
        HLAfixedRecord _InteractionForcePositionData;
        HLAfixedRecord _InteractionVolFormation;
		
        // Handles and Flags for ControlCommand datas
		rti1516e::ObjectClassHandle _ObjectClassHandleForClassControlCommandModel;
		rti1516e::ObjectInstanceHandle _ObjectInstanceHandleForClassControlCommandModel; // here we might have only one
		rti1516e::AttributeHandle _AttributeHandleForAttrControlCommandModelData;  
		rti1516e::AttributeHandleSet _AttributeHandleSetForAttrControlCommandModelData;
        rti1516e::AttributeHandleSet _AttributeHandleSetForAttrControlCommandModelDataSubscribed;
		bool _DiscovObjectInstanceHandleForClassControlCommandModel;
		bool _NewAttributeReceivedForAttrControlCommandModelData;

		// Handles and Flags for published datas
		// Flight dynamic data
		rti1516e::ObjectClassHandle _ObjectClassHandleForClassFlightDynamicsModel; 
		rti1516e::ObjectInstanceHandle _ObjectInstanceHandleForClassFlightDynamicsModel;
		rti1516e::AttributeHandle _AttributeHandleForAttrFlightDynamicsModelData; 
		rti1516e::AttributeHandleSet _AttributeHandleSetForAttrFlightDynamicsModelData; 
		rti1516e::AttributeHandleSet _AttributeHandleSetForAttrFlightDynamicsModelDataSubscribed; // FOr received data from other xpane instances
        // Hydraulic actuators data
		rti1516e::ObjectClassHandle _ObjectClassHandleForClassHydraulicActuatorsModel; 
		rti1516e::ObjectInstanceHandle _ObjectInstanceHandleForClassHydraulicActuatorsModel;
		rti1516e::AttributeHandle _AttributeHandleForAttrHydraulicActuatorsModelData; 
		rti1516e::AttributeHandleSet _AttributeHandleSetForAttrHydraulicActuatorsModelData; 
		// Engines actuators data
		rti1516e::ObjectClassHandle _ObjectClassHandleForClassEngineActuatorsModel; 
		rti1516e::ObjectInstanceHandle _ObjectInstanceHandleForClassEngineActuatorsModel;
		rti1516e::AttributeHandle _AttributeHandleForAttrEngineActuatorsModelData; 
		rti1516e::AttributeHandleSet _AttributeHandleSetForAttrEngineActuatorsModelData; 
		// Time information data
		rti1516e::ObjectClassHandle _ObjectClassHandleForClassTimeInformationModel; 
		rti1516e::ObjectInstanceHandle _ObjectInstanceHandleForClassTimeInformationModel;
		rti1516e::AttributeHandle _AttributeHandleForAttrTimeInformationModelData; 
		rti1516e::AttributeHandleSet _AttributeHandleSetForAttrTimeInformationModelData;
        // Force position
        rti1516e::InteractionClassHandle _InteractionHandleForForcePosition;
        rti1516e::ParameterHandle _ParameterHandleForInteractionPlaneNamePosition;
        rti1516e::ParameterHandle _ParameterHandleForInteractionForcePosition;
        // Vol Formation
        rti1516e::InteractionClassHandle _InteractionHandleForVolFormation;
        rti1516e::ParameterHandle _ParameterHandleForInteractionPlaneNameVolFormation;
        rti1516e::ParameterHandle _ParameterHandleForInteractionVolFormation;
		
	public:
        XplaneFederateHla1516e();
		virtual ~XplaneFederateHla1516e();
    
		// wstring handling
		static const std::string getString(const std::wstring& wstr) {
			return std::string(wstr.begin(),wstr.end());
		};

		static const std::wstring getWString(const char* cstr) {
			std::wstringstream ss;
			ss << cstr;
			return std::wstring(ss.str());
		};

        void disableFlightControl();
        void enableFlightControl();
        void disableJoystick();
        void enableJoystick();

        // get values for data type to sent
		void getfromXplaneLocalHLAFlightDynamicsModelData();
        void getfromAeroflyLocalHLAFlightDynamicsModelData(std::map<std::wstring, double> mapLocalDynamicModelDatas);
		void getfromXplaneLocalHLAHydraulicActuatorsModelData();
		void getfromXplaneLocalHLAEngineActuatorsModelData();
		void getfromXplaneLocalHLATimeInformationModelData();
        void getfromXplaneLocalHLAControlCommandModelData();
		// set values for data type to receive
		void setToXplaneLocalHLAControlCommandModelData();
        void setToXplaneLocalPosition(double latitude, double longitude, double altitude);
        void setToXplaneVolFormation(const int &position,
                                     const double &latitude,
                                     const double &longitude,
                                     const double &altitude,
                                     const double &distance,
                                     const double &vitesseX,
                                     const double &vitesseY,
                                     const double &vitesseZ,
                                     const double &cap,
                                     const Formation &formation
                                     );

        void printMyHLAFlightDynamicsModelData();
        void printMyHLAEngineActuatorsModelData();
        void printMyHLAControlCommandModelData();

        void printOtherPlanes();
        const std::map<std::wstring, RemoteAircraftToDraw> &getOthersAircraftToDraw();
        unsigned int getNbOtherAircraftToDraw();

        void connect(const std::wstring &FederationName, const std::wstring &FederateName, const std::wstring &FomFileName);
		void findUsedXplaneDataRefs();
		void createFederationExecution();
		void destroyFederationExecution();
		void joinFederationExecution();
		void resignFederationExecution();

		rti1516e::FederateHandle getFederateHandle() const ;
		void getAllHandles();
		void publishAndSubscribe();
		void registerObjectInstances();
        void deleteObjectInstances();
		void unpublishAndUnsubscribe();
		void waitForAllObjectDiscovered();
		void waitForAllAttributesReceived();
		void setHlaTimeManagementSettings( RTI1516fedTime TimeStep
                                         , RTI1516fedTime Lookahead
                                         , RTI1516fedTime LocalTime
                                         , RTI1516fedTime SimulationEndTime
                                         );
        void initializationFederate();
        void initializationRecords();
        bool isInitialized();
        void setInitialized(bool isInitialized);
        bool isCreator();
        void close();
		void calculateState();
        void calculateOutput();
        void sendUpdateAttributes();
        void sendInitialAllAttributes();
        void sendInteractionForcePosition(const std::wstring &planeName=L"*");
        void sendInteractionForcePosition(const double &latitude,
                                          const double &longitude,
                                          const double &altitude,
                                          const std::wstring &planeName=L"*");

        void sendInteractionsVolFormation(const double &latitude,
                                          const double &longitude,
                                          const double &altitude,
                                          const double &distance,
                                          const double &vitesseX,
                                          const double &vitesseY,
                                          const double &vitesseZ,
                                          const double &cap,
                                          const Formation &formation);
        void sendInteractionsVolFormation2(const double &latitude,
                                          const double &longitude,
                                          const double &altitude,
                                          const double &distance,
                                          const double &vitesseX,
                                          const double &vitesseY,
                                          const double &vitesseZ,
                                          const double &cap,
                                          const Formation &formation);
		void run();  
        void runOneStep();
        void pause();
		void pauseInitTrim();
		void pauseInitSim();
		bool getEndOfSimulation();

		// Callback : discover object instance
		void discoverObjectInstance (rti1516e::ObjectInstanceHandle theObject,
		                             rti1516e::ObjectClassHandle theObjectClass,
		                             std::wstring const &theObjectInstanceName)
                                     throw (rti1516e::FederateInternalError);

        void removeObjectInstance(ObjectInstanceHandle theObject,
                                  const VariableLengthData &theUserSuppliedTag,
                                  OrderType sentOrder,
                                  SupplementalRemoveInfo theRemoveInfo)
                                  throw (rti1516e::FederateInternalError);

		// Callback : reflect attribute values without time
		void reflectAttributeValues ( rti1516e::ObjectInstanceHandle theObject,
									rti1516e::AttributeHandleValueMap const &theAttributes,
									rti1516e::VariableLengthData const &theUserSuppliedTag,
									rti1516e::OrderType sentOrdering,
									rti1516e::TransportationType theTransport,
									rti1516e::SupplementalReflectInfo theReflectInfo)
							  throw (rti1516e::FederateInternalError) ;

		// Callback : reflect attribute values with time
		void reflectAttributeValues ( rti1516e::ObjectInstanceHandle theObject,
									rti1516e::AttributeHandleValueMap const &theAttributes,
									rti1516e::VariableLengthData const &theUserSuppliedTag,
									rti1516e::OrderType sentOrdering,
									rti1516e::TransportationType theTransport,
									rti1516e::LogicalTime const &theTime,
									rti1516e::OrderType receivedOrdering,
									rti1516e::MessageRetractionHandle theHandle,
									rti1516e::SupplementalReflectInfo theReflectInfo
									)
							  throw ( rti1516e::FederateInternalError) ;

        void reflectAttributeValues(ObjectInstanceHandle theObject,
                                    const AttributeHandleValueMap &theAttributeValues,
                                    const VariableLengthData &theUserSuppliedTag,
                                    OrderType sentOrder,
                                    TransportationType theType,
                                    const LogicalTime &theTime,
                                    OrderType receivedOrder,
                                    SupplementalReflectInfo theReflectInfo)
                                    throw ( rti1516e::FederateInternalError) ;

        void receiveInteraction(InteractionClassHandle theInteraction,
                                const ParameterHandleValueMap &theParameterValues,
                                const VariableLengthData &theUserSuppliedTag,
                                OrderType sentOrder,
                                TransportationType theType,
                                SupplementalReceiveInfo theReceiveInfo)
                                throw ( rti1516e::FederateInternalError) ;

        void receiveInteraction(InteractionClassHandle theInteraction,
                                const ParameterHandleValueMap &theParameterValues,
                                const VariableLengthData &theUserSuppliedTag,
                                OrderType sentOrder,
                                TransportationType theType,
                                const LogicalTime &theTime,
                                OrderType receivedOrder,
                                MessageRetractionHandle theHandle,
                                SupplementalReceiveInfo theReceiveInfo)
                                throw ( rti1516e::FederateInternalError) ;

        void receiveInteraction(InteractionClassHandle theInteraction,
                                const ParameterHandleValueMap &theParameterValues,
                                const VariableLengthData &theUserSuppliedTag,
                                OrderType sentOrder,
                                TransportationType theType,
                                const LogicalTime &theTime,
                                OrderType receivedOrder,
                                SupplementalReceiveInfo theReceiveInfo)
                                throw ( rti1516e::FederateInternalError) ;

		// HLA specific methods : TIME MANAGEMENT 
		// Callback : timeRegulationEnabled
		/*virtual void timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
		throw ( rti1516e::InvalidLogicalTime,
			rti1516e::NoRequestToEnableTimeRegulationWasPending,
			rti1516e::FederateInternalError) ;*/
        void timeRegulationEnabled(const rti1516e::LogicalTime& theTime)
		throw (rti1516e::FederateInternalError) ;

		// Callback : timeConstrainedEnabled
		/*virtual void timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
		throw ( rti1516e::InvalidLogicalTime,
			rti1516e::NoRequestToEnableTimeConstrainedWasPending,
			rti1516e::FederateInternalError) ;*/
        void timeConstrainedEnabled(const rti1516e::LogicalTime& theTime)
		throw (rti1516e::FederateInternalError) ;
		
		// Callback : timeAdvanceGrant
		/*virtual void timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
		throw ( rti1516e::InvalidLogicalTime,
			rti1516e::JoinedFederateIsNotInTimeAdvancingState,
			rti1516e::FederateInternalError) ;*/
		void timeAdvanceGrant(const rti1516e::LogicalTime& theTime)
		throw (rti1516e::FederateInternalError) ;
			
		void enableTimeRegulation();
		void enableTimeConstrained();
		void enableAsynchronousDelivery();
		void disableTimeRegulation();
		void disableTimeConstrained();
		void disableAsynchronousDelivery();
		void timeAdvanceRequest(RTI1516fedTime NextLogicalTime);
		void nextEventRequest(RTI1516fedTime NextLogicalTime);
		void timeAdvanceRequestAvailable(RTI1516fedTime NextLogicalTime);
		void nextEventAvailable(RTI1516fedTime NextLogicalTime);

		// HLA specific methods : SYNCHRONISATION 
		// Callback : synchronizationPointRegistrationSucceeded
		void synchronizationPointRegistrationSucceeded(std::wstring const &label )
		throw (rti1516e::FederateInternalError) ;

		// Callback : synchronizationPointRegistrationFailed
		void synchronizationPointRegistrationFailed(std::wstring const &label,  rti1516e::SynchronizationPointFailureReason reason)
		throw (rti1516e::FederateInternalError) ;

		// Callback : announceSynchronizationPoint
		void announceSynchronizationPoint(std::wstring const &label, rti1516e::VariableLengthData const &theUserSuppliedTag )
		throw (rti1516e::FederateInternalError) ;

		// Callback : federationSynchronized
		virtual void federationSynchronized( std::wstring const &label, rti1516e::FederateHandleSet const& failedToSyncSet)
			throw( rti1516e::FederateInternalError );

        const HLAfixedRecord& getLocalHLAFlightDynamicsModelData() const;

        const HLAfixedRecord& getLocalHLAHydraulicActuatorsModelData() const;

        const HLAfixedRecord& getLocalHLAEngineActuatorsModelData() const;

        const HLAfixedRecord& getLocalHLATimeInformationModelData() const;

private:
        // init all HlaRecord
        void initHLAFlightDynamicsModelData(bool remote);
        void initHLAHydraulicActuatorsModelData(bool remote);
        void initHLAEngineActuatorsModelData(bool remote);
        void initHLATimeInformationModelData(bool remote);
        void initHLAControlCommandModelData();
        void initHLAForcePosition();
        void initHLAVolFormation();

};

#endif // VISUALIZATION_HH_DEF
