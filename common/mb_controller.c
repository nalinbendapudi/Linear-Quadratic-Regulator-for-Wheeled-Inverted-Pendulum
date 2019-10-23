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

// gains for LQR controller
float k1=0.0, k2=0.0, k3=0.0, k4=0.0;


int mb_controller_init(){
    mb_controller_load_config();
    /* TODO initialize your controllers here*/


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
    fscanf(file, "%f %f %f %f", &k1, &k2, &k3, &k4);
    fclose(file);
    printf("Obtained gains: K1 = %f, K2 = %f, K3 = %f, K4 = %f\n", k1,k2,k3,k4);
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

    // apply the controller to get duty
    // get the duty cycle for the PID controller
    // k1 = 5.0;
    // float duty = -k1 * mb_state->theta;
    // get the duty cycle for the LQR controller
    float duty = -k1 * mb_state->theta - k2 * mb_state->theta_d - k3 * mb_state->phi - k4 * mb_state->phi_d;

    mb_state->left_cmd = duty;
    mb_state->right_cmd = duty;
    
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