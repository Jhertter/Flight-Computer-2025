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

// prototipos

static void do_nothing(void);
#if DEBUG_FSM
static void reset(void);
#endif
static void waitForStart(void);
static void startTransmission(void);
static void telemetry(void);
static void apogee_routine(void);
static void recovery_signal(void);
static void stand_by_mode(void);

/**
 * @brief 	Waits for signal to start the mission.
 * 			SRAD in low energy mode.
 *
 */
STATE pre_launch[] =
{
	{STAY, "pre_launch", pre_launch, waitForStart},
	{MOVE, "pre_launch", launch, startTransmission}
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
	// {STAY, launch, do_nothing}do_nothing,
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
	{STAY, "ascent", ascent, telemetry},
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
	{STAY, "apogee", apogee, apogee_routine},
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
	{STAY, "descent", descent, telemetry},
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
	{MOVE, "landing", recovery, stand_by_mode}
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
	{STAY, "recovery", recovery, recovery_signal},
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

/**
 * @brief Waits for START signal from ground station. Sends ACK.
 *
 */
static void waitForStart(void)
{
	return;
}

/**
 * @brief Starts transmission and activates payload.
 *
 */
static void startTransmission(void)
{
}

/**
 * @brief Transmits telemetry through xbee
 *
 */
static void telemetry(void)
{
	// xbee.sendPkt();
}

/**
 * @brief 	When 2000m are reached, starts airbrake control.
 * 			Stores max height.
 *
 */
static void apogee_routine(void)
{
}

/**
 * @brief 	Sends absolute coordinates and time to GS.
 * 			Starts max altitude protocol.
 *
 */
static void recovery_signal(void)
{
}

/**
 * @brief 	Shuts down sensors -> enters low energy mode.
 *
 */
static void stand_by_mode(void)
{
}

static void do_nothing(void)
{
}

#if DEBUG_FSM
static void reset(void)
{
	printf("Reset\n");
}
#endif