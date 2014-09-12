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
	led = hal.getLogicPeripheralOutput("led1");
	
	criticalOutputs = { power, enable0, enable1, enable2, enable3, led };
	
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
		{ controlStopping,    "Stopping control system",                               },
		{ controlStarting,    "Starting control system",                               },
		{ systemOn,           "System is on, drives disabled, waiting for approval",   },
		{ poweringDown,       "Powering down: disabeling drives",                      },
		{ poweringUp,         "Powering up: enabeling drives",                         },
		{ powerOn,            "Power on, 0V for all axes",                             },
		{ homeing,            "Automatic homeing for all axes",                        },
		{ axesHomed,          "Axes homed, moving TCP to ready position",              },
		{ parking,            "Parking: moving TCP to park position",                  },
		{ parked,             "Robot parked: TCP in park position, axes disabled",     },
		{ systemReady,        "System ready: not moving, axes enabled and controlled", },
		{ autoMoving,         "Robot is moving, pathplanner active",                   },
		{ mouseTeaching,      "Robot is moving, mouse input",                          },
		{ joystickTeaching,   "Robot is moving, joystick input",                       }
	};
	
	// ############ Add events to the levels ############
	level(off               ).addEvent(doSwInit,             swInitializing,     kPublicEvent);
	level(swInitializing    ).addEvent(swInitDone,           swInitialized,      kPrivateEvent);
	level(swInitialized     ).addEvent(doControlStart,       controlStarting,    kPublicEvent);
	level(emergency         ).addEvent(doEmergencyReset,     resetingEmergency,  kPublicEvent);
	level(resetingEmergency ).addEvent(emergencyResetDone,   systemOn,           kPrivateEvent);
	level(resetingEmergency ).addEvent(doEmergency,          emergency,          kPrivateEvent);
	level(controlStopping   ).addEvent(controlStoppingDone,  off,                kPublicEvent);
	level(controlStarting   ).addEvent(controlStartingDone,  systemOn,           kPrivateEvent);
	level(systemOn          ).addEvent(doControlStop,        controlStopping,    kPublicEvent);
	level(systemOn          ).addEvent(doPoweringUp,         poweringUp,         kPublicEvent);
	level(poweringDown      ).addEvent(poweringDownDone,     systemOn,           kPrivateEvent);
	level(poweringUp        ).addEvent(poweringUpDone,       powerOn,            kPrivateEvent);
	level(powerOn           ).addEvent(doPoweringDown,       poweringDown,       kPublicEvent);
	level(powerOn           ).addEvent(doHoming,             homeing,            kPublicEvent);
	level(homeing           ).addEvent(homeingDone,          axesHomed,          kPrivateEvent);
	level(axesHomed         ).addEvent(doSystemReady,        systemReady,        kPrivateEvent);
	level(parking           ).addEvent(parkingDone,          parked,             kPrivateEvent);
	level(parked            ).addEvent(doSystemReady,        systemReady,        kPublicEvent);
	level(systemReady       ).addEvent(doAutoMoving,         autoMoving,         kPublicEvent);
	level(systemReady       ).addEvent(doMouseTeaching,      autoMoving,         kPublicEvent);
	level(systemReady       ).addEvent(doJoystickTeaching,   autoMoving,         kPublicEvent);
	level(systemReady       ).addEvent(doParking,            parking,            kPublicEvent);
	level(autoMoving        ).addEvent(stopMoving,           systemReady,        kPublicEvent);
	level(mouseTeaching     ).addEvent(stopMoving,           systemReady,        kPublicEvent);
	level(joystickTeaching  ).addEvent(stopMoving,           systemReady,        kPublicEvent);
	
	addEventToLevelAndAbove(systemOn, doEmergency, emergency, kPublicEvent);
	
	// ############ Define input states and events for all levels ############
	level(off               ).setInputActions({ ignore(emergencyStop),                    ignore(approval),                          ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(swInitializing    ).setInputActions({ ignore(emergencyStop),                    ignore(approval),                          ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(swInitialized     ).setInputActions({ ignore(emergencyStop),                    ignore(approval),                          ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(emergency         ).setInputActions({ ignore(emergencyStop),                    check(approval, false, resetingEmergency), ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(resetingEmergency ).setInputActions({ check(emergencyStop, false, doEmergency), ignore(approval),                          ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
//	level(waitingForApproval).setInputActions({ ignore(emergencyStop),                    check(approval, false, doControlStart),    ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(controlStopping   ).setInputActions({ ignore(emergencyStop),                    ignore(approval),                          ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(controlStarting   ).setInputActions({ ignore(emergencyStop),                    ignore(approval),                          ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(systemOn          ).setInputActions({ ignore(emergencyStop),                    check(approval, false, doPoweringUp),      ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(poweringDown      ).setInputActions({ check(emergencyStop, false, doEmergency), ignore(approval),                          ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(poweringUp        ).setInputActions({ check(emergencyStop, false, doEmergency), ignore(approval),                          ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(powerOn           ).setInputActions({ check(emergencyStop, false, doEmergency), ignore(approval),                          ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(homeing           ).setInputActions({ check(emergencyStop, false, doEmergency), ignore(approval),                          ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(axesHomed         ).setInputActions({ check(emergencyStop, false, doEmergency), ignore(approval),                          ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(parking           ).setInputActions({ check(emergencyStop, false, doEmergency), ignore(approval),                          range(q0, q012SafeMin, q012SafeMax, doEmergency), range(q1, q012SafeMin, q012SafeMax, doEmergency), range(q2, q012SafeMin, q012SafeMax, doEmergency), range(q3, q3SafeMin, q3SafeMax, doEmergency)     });
	level(parked            ).setInputActions({ check(emergencyStop, false, doEmergency), ignore(approval),                          ignore(q0),                                       ignore(q1),                                       ignore(q2),                                       ignore(q3)                                       });
	level(systemReady       ).setInputActions({ check(emergencyStop, false, doEmergency), ignore(approval),                          range(q0, q012SafeMin, q012SafeMax, doEmergency), range(q1, q012SafeMin, q012SafeMax, doEmergency), range(q2, q012SafeMin, q012SafeMax, doEmergency), range(q3, q3SafeMin, q3SafeMax, doEmergency)     });
	level(autoMoving        ).setInputActions({ check(emergencyStop, false, doEmergency), ignore(approval),                          range(q0, q012SafeMin, q012SafeMax, doEmergency), range(q1, q012SafeMin, q012SafeMax, doEmergency), range(q2, q012SafeMin, q012SafeMax, doEmergency), range(q3, q3SafeMin, q3SafeMax, doEmergency)     });
	level(mouseTeaching     ).setInputActions({ check(emergencyStop, false, doEmergency), ignore(approval),                          range(q0, q012SafeMin, q012SafeMax, doEmergency), range(q1, q012SafeMin, q012SafeMax, doEmergency), range(q2, q012SafeMin, q012SafeMax, doEmergency), range(q3, q3SafeMin, q3SafeMax, doEmergency)     });
	level(joystickTeaching  ).setInputActions({ check(emergencyStop, false, doEmergency), ignore(approval),                          range(q0, q012SafeMin, q012SafeMax, doEmergency), range(q1, q012SafeMin, q012SafeMax, doEmergency), range(q2, q012SafeMin, q012SafeMax, doEmergency), range(q3, q3SafeMin, q3SafeMax, doEmergency)     });
	
	// ############ Define output states and events for all levels ############
	level(off               ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(led, false) });
	level(swInitializing    ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(led, false) });
	level(swInitialized     ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(led, false) });
	level(emergency         ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(led, true ) });
	level(resetingEmergency ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(led, true ) });
//	level(waitingForApproval).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(led, false) });
	level(controlStopping   ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(led, false) });
	level(controlStarting   ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(led, false) });
	level(systemOn          ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(led, false) });
	level(poweringDown      ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(led, false) });
	level(poweringUp        ).setOutputActions({ set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(led, false) });
	level(powerOn           ).setOutputActions({ set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(led, false) });
	level(homeing           ).setOutputActions({ set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(led, false) });
	level(axesHomed         ).setOutputActions({ set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(led, false) });
	level(parking           ).setOutputActions({ set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(led, false) });
	level(parked            ).setOutputActions({ set(enable0, false), set(enable1, false), set(enable2, false), set(enable3, false), set(led, false) });
	level(systemReady       ).setOutputActions({ set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(led, false) });
	level(autoMoving        ).setOutputActions({ set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(led, false) });
	level(mouseTeaching     ).setOutputActions({ set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(led, false) });
	level(joystickTeaching  ).setOutputActions({ set(enable0, true ), set(enable1, true ), set(enable2, true ), set(enable3, true ), set(led, false) });

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
	
	level(axesHomed).setLevelAction([&](SafetyContext* privateContext) {
		static bool first = true;
		static int count = 0;
		if(first) {
			first = false;
			controlSys->setVoltageForInitializing({-2, -2, -2, -2});
		}
		else {
			count++;
			if(count > static_cast<unsigned int>(0.3 / dt)) {
				controlSys->setVoltageForInitializing({0, 0, 0, 0});
				controlSys->voltageSwitch.switchToInput(0);
				privateContext->triggerEvent(doSystemReady);
				
			}
		}
	});
	
	// Define entry level
	entryLevel = off;
}

