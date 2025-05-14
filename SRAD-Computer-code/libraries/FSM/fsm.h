/*
 * fsm.h
 *
 *  Created on: 28/07/2014
 *      Author: Daniel Jacoby
 */

#ifndef FSM_H_
#define FSM_H_

#define FIN_TABLA 0xFF

typedef unsigned char BYTE;
typedef struct state_diagram_edge STATE;

struct state_diagram_edge
{
	BYTE evento;
	STATE *proximo_estado;
	void (*p_rut_accion)(void);
};

// Interfaz
STATE *fsm(STATE *p_tabla_estado, uint32_t evento_actual);

#endif /* FSM_H_ */
