/*******************************************************************************
* measure_moments.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the moments of inertia of your Balancebot
* 
* TODO: capture the gyro data and timestamps to a file to determine the period.
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>	// for mkdir and chmod
#include <sys/types.h>	// for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/mpu.h>

FILE* f1;

int64_t utime_now(void){
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t) tv.tv_sec*1000000 + tv.tv_usec;
}


/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	
	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.

    rc_mpu_data_t data; //struct to hold new data
    
    rc_mpu_config_t conf = rc_mpu_default_config();
    
    if(rc_mpu_initialize(&data, conf)){
        fprintf(stderr,"rc_mpu_initialize_failed\n");
        return -1;
    }

    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	/*if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
        return -1;
    }
*/
	// initialize enocders
    if(rc_encoder_eqep_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if(rc_adc_init()==-1){
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();


    rc_set_state(RUNNING);
    while(rc_get_state()!=EXITING){
    	rc_nanosleep(1E9);
        if(rc_mpu_read_gyro(&data)<0){
            printf("read gyro data failed\n");
        }
        int64_t time = utime_now();
        printf("%lld, %6.1f, %6.1f, %6.1f\n", time,   data.gyro[0], data.gyro[1],data.gyro[2]);
        fflush(stdout);
        printf("%lld, %6.1f, %6.1f, %6.1f\n", time,   data.gyro[0], data.gyro[1],data.gyro[2]);
    }

	// exit cleanly
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}