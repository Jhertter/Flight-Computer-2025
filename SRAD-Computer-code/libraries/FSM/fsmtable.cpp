#include <stdio.h>
#include "fsm.h"
#include "fsmtable.h"

extern STATE pre_launch[];
extern STATE launch[];
extern STATE ascent[];
extern STATE apogee[];
extern STATE descent[];
extern STATE landing[];
extern STATE recovery[];
extern STATE mission_complete[];
#if DEBUG_FSM
extern STATE reset_FSM[];
#endif

// Variables globales

extern XBee xbee;

// prototipos

static void do_nothing(void);
#if DEBUG_FSM
static void reset(void);
#endif
static void waitForStart(void);
static void calibrateRocket(void);
static void waitForLaunch(void);
static void telemetry(void);
static void ascentRoutine(void);
static void apogeeRoutine(void);
static void descentRoutine(void);
static void recoverySignal(void);
static void standByMode(void);

static fsm_actions_t event_action = STAY;

/**
 * @brief 	Waits for signal to start the mission.
 * 			SRAD in low energy mode.
 *
 */
STATE pre_launch[] =
{
	{STAY, "pre_launch", pre_launch, waitForStart},
	{MOVE, "pre_launch", launch, calibrateRocket}
#if DEBUG_FSM
	,
	{RESET_MISSION, "pre_launch", reset_FSM, do_nothing}
#endif
};

/**
 * @brief 	ACK to ground station.
 * 			Wakes up the SRAD and sends signal to the airbrake.
 *			Starts XBee transmition.
 *
 */
STATE launch[] =
{
	{STAY, "launch", launch, waitForLaunch},
	{MOVE, "launch", ascent, do_nothing}
#if DEBUG_FSM
	,
	{RESET_MISSION, "launch", reset_FSM, do_nothing}
#endif
};

/**
 * @brief 	Captures sensor data and sends it to the ground station.
 *
 */
STATE ascent[] =
{
	{STAY, "ascent", ascent, ascentRoutine},
	{MOVE, "ascent", apogee, do_nothing}
#if DEBUG_FSM
	,
	{RESET_MISSION, "ascent", reset_FSM, do_nothing}
#endif
};

/**
 * @brief 	Once apogee is reached, max height is stored.
 *
 */
STATE apogee[] =
{
	{STAY, "apogee", apogee, apogeeRoutine},
	{MOVE, "apogee", descent, do_nothing}
#if DEBUG_FSM
	,
	{RESET_MISSION, "apogee", reset_FSM, do_nothing}
#endif
};

/**
 * @brief 	Captures sensor data and sends it to the ground station.
 *
 */
STATE descent[] =
{
	{STAY, "descent", descent, descentRoutine},
	{MOVE, "descent", landing, do_nothing}
#if DEBUG_FSM
	,
	{RESET_MISSION, "descent", reset_FSM, do_nothing}
#endif
};

/**
 * @brief 	Once IMU senses no more movement, sensors shut down.
 *
 */
STATE landing[] =
{
	{MOVE, "landing", recovery, standByMode}
#if DEBUG_FSM
	,
	{RESET_MISSION, "landing", reset_FSM, do_nothing}
#endif
};

/**
 * @brief 	Sends absolute coordinates to the ground station.
 * 			Starts max altitude protocol.
 *
 */
STATE recovery[] =
{
	{STAY, "recovery", recovery, recoverySignal},
	{MOVE, "recovery", mission_complete, do_nothing}
#if DEBUG_FSM
	,
	{RESET_MISSION, "recovery", reset_FSM, do_nothing}
#endif
};

/**
 * @brief 	If necesary, an ACK from GS can be received.
 * 			Altitude protocol.
 *
 */
STATE mission_complete[] =
{
	{STAY, "mission_complete", mission_complete, do_nothing}
#if DEBUG_FSM
	,
	{RESET_MISSION, "mission_complete", reset_FSM, do_nothing}
#endif
};

#if DEBUG_FSM
STATE *reset_FSM[] =
{
	{STAY, "reset", reset_FSM, do_nothing},
	{MOVE, "reset", pre_launch, do_nothing}
};
#endif

//========interfaz=================

STATE *FSM_GetInitState(void)
{
	return (pre_launch);
}

///=========Rutinas de accion===============



static void do_nothing(void)
{
}

#if DEBUG_FSM
static void reset(void)
{
	printf("Reset\n");
}
#endif