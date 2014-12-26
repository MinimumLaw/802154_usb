/*
 * sysclock.h
 *
 * Created: 01.12.2014 13:30:09
 *  Author: alex
 */ 


#ifndef _INCLUDE_SYSCLOCK_H_
#define _INCLUDE_SYSCLOCK_H_

extern void disable_all_peripherial (void);
extern void sysclock_init(void);

extern uint8_t read_calibration_byte( uint8_t index );


#endif /* _INCLUDE_SYSCLOCK_H_ */
