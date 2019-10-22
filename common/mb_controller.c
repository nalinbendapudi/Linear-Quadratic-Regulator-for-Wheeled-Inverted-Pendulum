#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mb_controller.h"
#include "mb_defs.h"

/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_controller_init(){
    mb_controller_load_config();
    /* TODO initialize your controllers here*/


    return 0;
}

/*
    This function gets the position of the motors and returns phi of the wheels
*/
int mb_encoder_read_pos(){
    /*
    double motorL_pos = 0.0, motorR_pos = 0.0;
    motorL_pos = rc_encoder_eqep_read(LEFT_MOTOR);
    motorR_pos = rc_encoder_eqep_read(RIGHT_MOTOR);
    printf("motor 1: %lf and motor 2: %lf\n", motorL_pos, motorR_pos);
    */

    return 0;
}

/*
    This function gets the velocity of the motors 
*/
int mb_encoder_read_velocity(){
    /*
    double motorL_pos = 0.0, motorR_pos = 0.0;
    motorL_pos = rc_encoder_eqep_read(LEFT_MOTOR);
    motorR_pos = rc_encoder_eqep_read(RIGHT_MOTOR);
    printf("motor 1: %lf and motor 2: %lf\n", motorL_pos, motorR_pos);
    */

    return 0;
}

/*******************************************************************************
* int mb_controller_load_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_controller_load_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening %s\n", CFG_PATH );
    }
    /* TODO parse your config file here*/
    /* K gain from LQR*/
    double k1, k2, k3, k4;
    fscanf(file, "%lf %lf %lf %lf", &k1, &k2, &k3, &k4);
    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* 
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* this should only be called in the imu call back function, no mutex needed
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state){
    /*TODO: Write your controller here*/
    /* imu orientation dmp */
    /* duty = -k1 * theta - k2 * theta_dot - k3 * phi - k4 * phi_dot;*/
    return 0;
}


/*******************************************************************************
* int mb_controller_cleanup()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_cleanup(){
    return 0;
}