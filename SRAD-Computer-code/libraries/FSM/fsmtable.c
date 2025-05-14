#include <stdio.h>
#include "fsm.h"
#include "fsmtable.h"
#include "contador.h"

/*Foward Declarations*/

extern STATE pre_launch[];
extern STATE launch[];
extern STATE ascent[];
extern STATE apogee[];
extern STATE descent[];
extern STATE landing[];
extern STATE recovery[];
extern STATE mission_complete[];

// prototipos

static void do_nothing(void);
static void reset_FSM(void);
static void match(void);

/**
 * @brief 	Waits for signal to start the mission. 
 * 			SRAD in low energy mode.
 *
 */
STATE pre_launch[] =
	{
		{'D', launch, do_nothing},
		{FIN_TABLA, estado_0, reset_FSM}};

/**
 * @brief 	ACK to ground station.
 * 			Wakes up the SRAD and sends signal to the airbrake.
 *			Starts XBee transmition.
 * 
 */
STATE launch[] =
	{
		{'a', ascent, do_nothing},
		{FIN_TABLA, estado_0, reset_FSM}};

/**
 * @brief 	Captures sensor data and sends it to the ground station.
 * 
 */
STATE ascent[] =
	{
		{'n', apogee, do_nothing},
		{FIN_TABLA, estado_0, reset_FSM}};

/**
 * @brief 	Once apogee is reached, max height is stored.
 * 
 */
STATE apogee[] =
	{
		{'i', descent, do_nothing},
		{FIN_TABLA, estado_0, reset_FSM}};

/**
 * @brief 	Captures sensor data and sends it to the ground station.
 * 
 */
STATE descent[] =
	{
		{'e', landing, do_nothing},
		{FIN_TABLA, estado_0, reset_FSM}};

/**
 * @brief 	Once IMU senses no more movement, sensors shut down.
 * 
 */
STATE landing[] =
	{
		{'l', recovery match},
		{FIN_TABLA, estado_0, reset_FSM}};

/**
 * @brief 	Sends absolute coordinates to the ground station.
 * 			Starts max altitude protocol.
 * 
 */
STATE recovery[] =
	{
		{'l', mission_complete, match},
		{FIN_TABLA, estado_0, reset_FSM}};

/**
 * @brief 	If necesary, an ACK from GS can be received.
 * 			Altitude protocol.
 * 
 */
STATE mission_complete[] =
	{
		{'l', estado_0, match},
		{'n', estado_0, match},
		{FIN_TABLA, estado_0, reset_FSM}};

//========interfaz=================

STATE *FSM_GetInitState(void)
{
	return (estado_0);
}

///=========Rutinas de accion===============

/*Incrementa el contador de coincidencias */

static void match(void)
{

	inc_cont();
}

/*Dummy function*/
static void do_nothing(void)
{
}

/*Restart FSM*/
static void reset_FSM(void)
{
	printf("Reset\n");
}
