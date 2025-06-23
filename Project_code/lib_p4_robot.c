/*
 * lib_p4_robot.c
 *
 *  Created on: 6 may. 2024
 *      Author: vadri
 */
#include "lib_p4_robot.h"

void move_up(uint32_t velocidad){
    move_motor(MOTOR_RIGHT,1,velocidad);//motor derecho
    move_motor(MOTOR_LEFT,0,velocidad);//motor izquierdo
}

void move_up_custom(uint32_t velocidadDerecha, uint32_t velocidadIzquierda){
    move_motor(MOTOR_RIGHT,1,velocidadDerecha);//motor derecho
    move_motor(MOTOR_LEFT,0,velocidadIzquierda);//motor izquierdo

}

void move_down (uint32_t velocidad){
    move_motor(MOTOR_RIGHT,0,velocidad);//motor derecho
    move_motor(MOTOR_LEFT,1,velocidad);//motor izquierdo
}
void move_rotation(uint32_t velocidad){
    move_motor(MOTOR_RIGHT,1,velocidad);//motor derecho
    move_motor(MOTOR_LEFT,1,velocidad);//motor izquierdo
}

void move_rotation_custom(uint32_t velocidadDerecha, uint32_t velocidadIzquierda){
    move_motor(MOTOR_RIGHT,1,velocidadDerecha);//motor derecho
    move_motor(MOTOR_LEFT,1,velocidadIzquierda);//motor izquierdo
}

void move_rotation_inverse(uint32_t velocidad){
    move_motor(MOTOR_RIGHT,0,velocidad);//motor derecho
    move_motor(MOTOR_LEFT,0,velocidad);//motor izquierdo
}

void move_rotation_inverse_custom(uint32_t velocidadDerecha, uint32_t velocidadIzquierda){
    move_motor(MOTOR_RIGHT,0,velocidadDerecha);//motor derecho
    move_motor(MOTOR_LEFT,0,velocidadIzquierda);//motor izquierdo
}


void move_rotation_right(uint32_t velocidad){
    move_motor(MOTOR_LEFT,0,velocidad);//motor izquierdo
}

void move_rotation_left(uint32_t velocidad){
    move_motor(MOTOR_RIGHT,1,velocidad);//motor derecho
}

void stop(){
    move_motor(MOTOR_RIGHT,0,1);//motor derecho
    move_motor(MOTOR_LEFT,0,1);//motor izquierdo
}
