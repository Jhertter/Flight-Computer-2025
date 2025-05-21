/*****************************
 * FSM.c
 * 
 * @ author Daniel Jacoby
 * @ date 28/07/2014
 *
 *****************************/

#include <stdio.h>
#include "fsm.h"
#include "pico/stdlib.h"
#include <stdint.h>
// #include "termlib.h"

STATE *fsm(STATE *p_tabla_estado, uint32_t evento_actual)
{
   printf("STATE: \n", evento_actual); // just for test (debug)

   while (p_tabla_estado->evento != evento_actual && p_tabla_estado->evento != FIN_TABLA)
      ++p_tabla_estado;

   (*p_tabla_estado->p_rut_accion)();               
   p_tabla_estado = p_tabla_estado->proximo_estado; 

   return (p_tabla_estado);
}