/*
 * fsm.h
 *
 *  Created on: 28/07/2014
 *      Author: Daniel Jacoby
 */

#ifndef FSM_H_
#define FSM_H_

#define FIN_TABLA 0xFF
#define DEBUG_FSM 0

typedef unsigned char BYTE;
typedef struct state_diagram_edge STATE;



struct state_diagram_edge
{
	fsm_actions_t evento;
	const char *estado_actual;
	STATE *proximo_estado;
	void (*p_rut_accion)(void);
};

// Interfaz
STATE *fsm(STATE *p_tabla_estado, fsm_actions_t evento_actual);

#endif /* FSM_H_ */
