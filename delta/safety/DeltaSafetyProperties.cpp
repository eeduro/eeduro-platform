#include "DeltaSafetyProperties.hpp"
#include "../control/ControlSystem.hpp"
#include <eeros/hal/HAL.hpp>
#include <eeros/safety/InputAction.hpp>
#include <eeros/safety/inputActions.hpp>
#include <eeros/safety/OutputAction.hpp>

#include <iostream>

using namespace eeduro::delta;
using namespace eeros;
using namespace eeros::hal;
using namespace eeros::safety;

DeltaSafetyProperties::DeltaSafetyProperties(ControlSystem* cs) : controlSys(cs) {

	if(controlSys == nullptr) {
		throw -325; // TODO
	}
	
	HAL& hal = HAL::instance();
	
	// ############ Define criticcal outputs ############
	enable0 = hal.getLogicPeripheralOutput("enable0");
	enable1 = hal.getLogicPeripheralOutput("enable1");
	enable2 = hal.getLogicPeripheralOutput("enable2");
	enable3 = hal.getLogicPeripheralOutput("enable3");
	
	criticalOutputs = { power, enable0, enable1, enable2, enable3 };
	
	// ############ Define criticcal inputs ############
	emergencyStop = hal.getLogicPeripheralInput("emergency");
	approval = hal.getLogicPeripheralInput("approval");
	q0 = hal.getRealPeripheralInput("q0");
	q1 = hal.getRealPeripheralInput("q1");
	q2 = hal.getRealPeripheralInput("q2");
	q3 = hal.getRealPeripheralInput("q3");
	
	criticalInputs = { emergencyStop, approval, q0, q1, q2, q3 };
	
	// ############ Define Levels ############
	
	levels = {
		{ off,                "System off",                                            },
		{ swInitializing,     "Initializing software",                                 },
		{ swInitialized,      "Software initialized",                                  },
		{ emergency,          "EMERGENCY state",                                       },
		{ resetingEmergency,  "Reseting emergency state",                              },
//		{ waitingForApproval, "Waiting for approval",                                  },
		{ controlStopping,    "Stopping control system",                               },
		{ controlStarting,    "Starting control system",                               },
		{ systemOn,           "System is on, drives disabled, waiting for approval",   },
		{ poweringDown,       "Powering down: disabeling drives",                      },
		{ poweringUp,         "Powering up: enabeling drives",                         },
		{ powerOn,            "Power on, 0V for all axis",                             },
		{ homeing,            "Automatic homeing for all axis",                        },
		{ axisHomed,          "Axis homed, moving TCP to ready position",              },
		{ parking,            "Parking: moving TCP to park position",                  },
		{ parked,             "Robot parked: TCP in park position, axis disabled",     },
		{ systemReady,        "System ready: not moving, axis enabled and controlled", },
		{ moving,             "Robot is moving",                                       }
	};
	
	// ############ Add events to the levels ############
	level(off               ).addEvent(doSwInit,             swInitializing,     kPublicEvent);
	level(swInitializing    ).addEvent(swInitDone,           swInitialized,      kPrivateEvent);
	level(swInitialized     ).addEvent(doControlStart,       controlStarting,    kPublicEvent);
	level(emergency         ).addEvent(doEmergencyReset,     resetingEmergency,  kPublicEvent);
	level(resetingEmergency ).addEvent(emergencyResetDone,   systemOn,           kPrivateEvent);
	level(resetingEmergency ).addEvent(doEmergency,          emergency,          kPrivateEvent);
//	level(waitingForApproval).addEvent(doControlStart,       controlStarting,    kPublicEvent);
	level(controlStopping   ).addEvent(controlStoppingDone,  off,                kPublicEvent);
	level(controlStarting   ).addEvent(controlStartingDone,  systemOn,           kPrivateEvent);
	level(systemOn          ).addEvent(doControlStop,        controlStopping,    kPublicEvent);
	level(systemOn          ).addEvent(doPoweringUp,         poweringUp,         kPublicEvent);
	level(poweringDown      ).addEvent(poweringDownDone,     systemOn,           kPrivateEvent);
	level(poweringUp        ).addEvent(poweringUpDone,       powerOn,            kPrivateEvent);
	level(powerOn           ).addEvent(doPoweringDown,       poweringDown,       kPublicEvent);
	level(powerOn           ).addEvent(doHoming,             homeing,            kPublicEvent);
	level(homeing           ).addEvent(homeingDone,          axisHomed,          kPrivateEvent);
	level(axisHomed         ).addEvent(doSystemReady,        systemReady,        kPrivateEvent);
	level(parking           ).addEvent(parkingDone,          parked,             kPrivateEvent);
	level(parked            ).addEvent(doSystemReady,        systemReady,        kPublicEvent);
	level(systemReady       ).addEvent(doMoving,             moving,             kPublicEvent);
	level(systemReady       ).addEvent(doParking,            parking,            kPublicEvent);
	level(moving            ).addEvent(stopMoving,           systemReady,        kPublicEvent);
	
	addEventToLevelAndAbove(systemOn, doEmergency, emergency, kPublicEvent);
	
	// ############ Define input states and events for all levels ############
// 	level(off               ).setInputActions({ ignore(emergencyStop),                   ignore(approval),                         ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(swInitializing    ).setInputActions({ ignore(emergencyStop),                   ignore(approval),                         ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(swInitialized     ).setInputActions({ ignore(emergencyStop),                   ignore(approval),                         ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(emergency         ).setInputActions({ ignore(emergencyStop),                   check(approval, true, resetingEmergency), ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(resetingEmergency ).setInputActions({ check(emergencyStop, true, doEmergency), ignore(approval),                         ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
//	level(waitingForApproval).setInputActions({ ignore(emergencyStop),                   check(approval, true, doControlStart),    ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(controlStopping   ).setInputActions({ ignore(emergencyStop),                   ignore(approval),                         ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(controlStarting   ).setInputActions({ ignore(emergencyStop),                   ignore(approval),                         ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(systemOn          ).setInputActions({ ignore(emergencyStop),                   check(approval, true, doPoweringUp),      ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(poweringDown      ).setInputActions({ check(emergencyStop, true, doEmergency), ignore(approval),                         ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(poweringUp        ).setInputActions({ check(emergencyStop, true, doEmergency), ignore(approval),                         ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(powerOn           ).setInputActions({ check(emergencyStop, true, doEmergency), ignore(approval),                         ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(homeing           ).setInputActions({ check(emergencyStop, true, doEmergency), ignore(approval),                         ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(axisHomed         ).setInputActions({ check(emergencyStop, true, doEmergency), ignore(approval),                         ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(parking           ).setInputActions({ check(emergencyStop, true, doEmergency), ignore(approval),                         range(q0, q012SafeMin, q012SafeMax, doEmergency), range(q0, q012SafeMin, q012SafeMax, doEmergency), range(q0, q012SafeMin, q012SafeMax, doEmergency), range(q0, q012SafeMin, q012SafeMax, doEmergency) });
	level(parked            ).setInputActions({ check(emergencyStop, true, doEmergency), ignore(approval),                         ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(systemReady       ).setInputActions({ check(emergencyStop, true, doEmergency), ignore(approval),                         range(q0, q012SafeMin, q012SafeMax, doEmergency), range(q0, q012SafeMin, q012SafeMax, doEmergency), range(q0, q012SafeMin, q012SafeMax, doEmergency), range(q0, q012SafeMin, q012SafeMax, doEmergency) });
	level(moving            ).setInputActions({ check(emergencyStop, true, doEmergency), ignore(approval),                         range(q0, q012SafeMin, q012SafeMax, doEmergency), range(q0, q012SafeMin, q012SafeMax, doEmergency), range(q0, q012SafeMin, q012SafeMax, doEmergency), range(q0, q012SafeMin, q012SafeMax, doEmergency) });
	
	// ############ Define output states and events for all levels ############
	level(off               ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false) });
	level(swInitializing    ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false) });
	level(swInitialized     ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false) });
	level(emergency         ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false) });
	level(resetingEmergency ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false) });
//	level(waitingForApproval).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false) });
	level(controlStopping   ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false) });
	level(controlStarting   ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false) });
	level(systemOn          ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false) });
	level(poweringDown      ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false) });
	level(poweringUp        ).setOutputActions({ set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ) });
	level(powerOn           ).setOutputActions({ set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ) });
	level(homeing           ).setOutputActions({ set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ) });
	level(axisHomed         ).setOutputActions({ set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ) });
	level(parking           ).setOutputActions({ set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ) });
	level(parked            ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false) });
	level(systemReady       ).setOutputActions({ set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ) });
	level(moving            ).setOutputActions({ set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ) });

	// Define and add level functions
	level(off).setLevelAction([&](SafetyContext* privateContext) {
		privateContext->triggerEvent(doSwInit);
	});
	
	level(swInitializing).setLevelAction([&](SafetyContext* privateContext) {
		privateContext->triggerEvent(swInitDone);
	});
	
	level(swInitialized).setLevelAction([&](SafetyContext* privateContext) {
		privateContext->triggerEvent(doControlStart);
	});
	
	level(controlStopping).setLevelAction([&](SafetyContext* privateContext) {
		controlSys->stop();
		privateContext->triggerEvent(controlStoppingDone);
	});
	
	level(controlStarting).setLevelAction([&](SafetyContext* privateContext) {
		controlSys->torqueLimitation.setLimit({-q012gearTorqueLimit, -q012gearTorqueLimit, -q012gearTorqueLimit, -q3gearTorqueLimit}, {q012gearTorqueLimit, q012gearTorqueLimit, q012gearTorqueLimit, q3gearTorqueLimit});
		controlSys->start();
		privateContext->triggerEvent(controlStartingDone);
	});
	
	level(poweringDown).setLevelAction([&](SafetyContext* privateContext) {
		controlSys->disableAxis();
		privateContext->triggerEvent(poweringDownDone);
	});
	
	level(poweringUp).setLevelAction([&](SafetyContext* privateContext) {
		controlSys->enableAxis();
		privateContext->triggerEvent(poweringUpDone);
	});
	
	level(powerOn).setLevelAction([&](SafetyContext* privateContext) {
		if(!controlSys->axisHomed()) privateContext->triggerEvent(doHoming);
		else privateContext->triggerEvent(doPoweringDown);
	});
	
	level(homeing).setLevelAction([&](SafetyContext* privateContext) {
		static unsigned int count = 0;
		if(count == 0) {
			controlSys->setVoltageForInitializing({q012InitVoltage, q012InitVoltage, q012InitVoltage, q3InitVoltage});
		}
		else if(count > static_cast<unsigned int>(1.0 / dt)) {
			if(controlSys->switchToPosControl()) {
				
				privateContext->triggerEvent(homeingDone);
			}
		}
		count++;
	});
	
	level(axisHomed).setLevelAction([&](SafetyContext* privateContext) {
		static bool first = true;
		if(first) {
			first = false;
			controlSys->goToPos(tcpReady_x, tcpReady_y, tcpReady_z, tcpReady_phi);
		}
		else {
			AxisVector tcp = controlSys->getTcpPos();
			if(tcp(2) <= tcpReady_z) {
// 				std::cout << "q = " << controlSys->getAxisPos() << std::endl; 
				privateContext->triggerEvent(doSystemReady);
				
			}
		}
	});
	
	// Define entry level
	entryLevel = off;
}

