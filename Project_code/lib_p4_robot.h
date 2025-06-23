#ifndef lib_p4_robot_LCD_H_
#define lib_p4_robot_LCD_H_
#include <stdio.h>
#include <stdint.h>
typedef uint8_t byte;

void move_motor(byte id_motor, byte sentido,uint32_t velocidad);
void move_up(uint32_t velocidad);
void move_down (uint32_t velocidad);
void move_rotation(uint32_t velocidad);
void move_rotation_custom(uint32_t velocidadDerecha, uint32_t velocidadIzquierda);
void move_rotation_inverse(uint32_t velocidad);
void move_rotation_inverse_custom(uint32_t velocidadDerecha, uint32_t velocidadIzquierda);
void move_rotation_right(uint32_t velocidad);
void move_rotation_right_custom(uint32_t velocidadDerecha, uint32_t velocidadIzquierda);
void move_rotation_left(uint32_t velocidad);
void move_rotation_left_custom(uint32_t velocidadDerecha, uint32_t velocidadIzquierda);
void stop();

#define MOTOR_LEFT 4
#define MOTOR_RIGHT 2
#define SENSOR 100

#endif /* lib_p4_robot_LCD_H_ */
