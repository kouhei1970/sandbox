//This file is Quadrotor Helicopter Control Program
#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>            // ArduPilot Mega Barometer Library
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <Filter.h>
#include <SITL.h>
#include <AP_Buffer.h>
#include <AP_Notify.h>
#include <AP_Vehicle.h>
#include <DataFlash.h>
#include <AP_BattMonitor.h>
#include <AP_RangeFinder.h>
#include <AP_Terrain.h>
#include <LowPassFilter.h> 

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>

#include <PID.h>


// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

// INS and Baro declaration
AP_InertialSensor_MPU6000 ins;
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);
AP_Compass_HMC5843 compass;
AP_GPS gps;
AP_AHRS_DCM  ahrs(ins, baro, gps);
AP_BattMonitor battery_mon;

//Super Sonic Sensor
RangeFinder sonar;
bool sonar_enabled = true; // enable user switch for sonar
LowPassFilterFloat low_pass_filter;
DataFlash_APM2 DataFlash;

#define HIGH 1
#define LOW 0

// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_MIN   1109
#define RC_YAW_MIN   1097
#define RC_YAW_MAX   1934
#define RC_PIT_MIN   1176
#define RC_PIT_MAX   1854
#define RC_ROL_MIN   1182
#define RC_ROL_MAX   1854

// Motor numbers definitions
#define MOTOR_FL   2    // Front left    
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right

#define SONAR_RELIABLE_DISTANCE_PCT 0.60f
#define SONAR_ALT_HEALTH_MAX 3

// Arduino map function
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float constrain(float x, float min, float max)
{
	if(x < min) x = min;
	if(x > max) x = max;
	return x;
}

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

// PID array (6 pids, two for each axis)
PID pids[6];
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5

#define FLY_STATE_LAND 0
#define FLY_STATE_FLY 1

int Fly_state = FLY_STATE_LAND;

float Alt[10]={0};
int AltCounter = 0;
static uint32_t timer_u;
static uint32_t timer_m;

long Time=0;
static int16_t sonar_alt;
static uint8_t sonar_alt_health;   // true if we can trust the altitude from the sonar



// return sonar altitude in centimeters
static int16_t read_sonar(void)
{
	sonar.update();

	// exit immediately if sonar is disabled
	if (!sonar.healthy()) {
        	sonar_alt_health = 0;
        	return 0;
    	}

	int16_t temp_alt = sonar.distance_cm();

	if (temp_alt >= sonar.min_distance_cm() && 
        	temp_alt <= sonar.max_distance_cm() * SONAR_RELIABLE_DISTANCE_PCT) {
        	if ( sonar_alt_health < SONAR_ALT_HEALTH_MAX ) {
            		sonar_alt_health++;
        	}
    	}else{
        	sonar_alt_health = 0;
    	}

 #if 0
    	// correct alt for angle of the sonar
    	float temp = ahrs.cos_pitch() * ahrs.cos_roll();
    	temp = max(temp, 0.707f);
    	temp_alt = (float)temp_alt * temp;
 #endif

    	return temp_alt;

}

void setup() 
{
	// Turn on MPU6050 - quad must be kept still as gyros will calibrate
	ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);
	ins.init_accel();
	ahrs.init();

	if( compass.init() ) {
		hal.console->printf("Enabling compass\n");
		ahrs.set_compass(&compass);
	}
	else {
		hal.console->printf("No compass detected\n");
	}
	
	//Init gps
	gps.init(NULL);

	//Init Low pass filter for altitude 
	low_pass_filter.set_cutoff_frequency(0.01f, 2.0f);
	low_pass_filter.reset(0);

	// setup the main LEDs as outputs
	hal.gpio->pinMode(HAL_GPIO_A_LED_PIN, HAL_GPIO_OUTPUT);
	hal.gpio->pinMode(HAL_GPIO_B_LED_PIN, HAL_GPIO_OUTPUT);
	hal.gpio->pinMode(HAL_GPIO_C_LED_PIN, HAL_GPIO_OUTPUT);

	// turn all lights off
	hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
	hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
	hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);

	// initialise the battery monitor
    	battery_mon.init();
    	battery_mon.set_monitoring(AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);

	// initialise the battery monitor
    	battery_mon.init();
    	battery_mon.set_monitoring(AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);

	//A2(Pin 56)Degital Output
    	hal.gpio->pinMode(56, HAL_GPIO_OUTPUT);
    	hal.gpio->write(56, HIGH);	


	// Enable the motors and set at 400Hz update
	hal.rcout->set_freq(0xF, 400);
    	for (int i=0; i<4; i++) {
        	hal.rcout->enable_ch(i);
        	hal.rcout->write(i, 900);
    	}

	// PID Configuration
	pids[PID_PITCH_RATE].kP(0.7);
	pids[PID_PITCH_RATE].kI(1);
	pids[PID_PITCH_RATE].imax(50);

	pids[PID_ROLL_RATE].kP(0.7);
	pids[PID_ROLL_RATE].kI(1);
	pids[PID_ROLL_RATE].imax(50);

	pids[PID_YAW_RATE].kP(2.7);
	pids[PID_YAW_RATE].kI(1);
	pids[PID_YAW_RATE].imax(50);

	pids[PID_PITCH_STAB].kP(4.5);
	pids[PID_ROLL_STAB].kP(4.5);
	pids[PID_YAW_STAB].kP(10);

    // print welcome message
    hal.console->println("Range Finder library test");

    // initialise sensor
    //AP_Param::set_object_value(&sonar, sonar.var_info, "_TYPE", RangeFinder::RangeFinder_TYPE_ANALOG);
    //AP_Param::set_object_value(&sonar, sonar.var_info, "_PIN", 0);
    //AP_Param::set_object_value(&sonar, sonar.var_info, "_SCALING", 3.10);
    sonar.init();
    sonar.update();

    // display range finder device count
    //uint8_t device_count = sonar.get_count();
    //hal.console->printf_P(PSTR("Number of range finders:%d\n"), (int)device_count);
    
    // kick off one reading
    //sonar.start_reading();

    for (int i = 0; i < 1; i++) {
        // check health
        if (!sonar.healthy(i)) {
            hal.console->printf_P(PSTR("Initialisation failed for device %d\n"), i);
        }
    }

	
	timer_u = hal.scheduler->micros();
	timer_m = hal.scheduler->millis();  

  // We're ready to go! Now over to loop()
}

void loop() 
{
	//A2 High**********************
    	hal.gpio->write(56, HIGH);	

	timer_u = hal.scheduler->micros(); //制御周期確認用（100Hz）
	static float yaw_target = 0;  

	//Battery State Read	
	battery_mon.read();
	float Voltage = battery_mon.voltage();
	//Battery Status if Battery voltage Under 21.0V then All LED ON 
	if(Voltage<21.0){
		hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
		hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
		hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
	}else{
		hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
		hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
		hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
	}
	
	//Altitude
	float alt = (float)read_sonar();
	float alt_f = low_pass_filter.apply(alt);

	// Read RC transmitter and map to sensible values  
	uint16_t channels[8];
	long rcthr, rcyaw, rcpit, rcroll;  // Variables to store radio in
	hal.rcin->read(channels, 8);
	rcthr = channels[2];
	rcyaw = map(channels[3], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
	rcpit = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, -45, 45);
	rcroll = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, -45, 45);

	//update gyro
	ahrs.update();

	//angle
	float roll,pitch,yaw;  
	roll =	ToDeg(ahrs.roll) ;
	pitch = ToDeg(ahrs.pitch) ;
	yaw = 	ToDeg(ahrs.yaw) ;

	//rate
	// Ask MPU6050 for gyro data
	Vector3f gyro = ins.get_gyro();
	float gyroPitch =	ToDeg(gyro.y);
	float gyroRoll =	ToDeg(gyro.x);
	float gyroYaw =		ToDeg(gyro.z);

	// Do the magic
	if(rcthr > RC_THR_MIN + 100) {  // Throttle raised, turn on stablisation.
		// Stablise PIDS
		float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250); 
		float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
		float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);

		// is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
		if(abs(rcyaw ) > 5) {
		  yaw_stab_output = rcyaw;
		  yaw_target = yaw;   // remember this yaw for when pilot stops
    	}
    
		// rate PIDS
		long pitch_output =  (long) constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), - 500, 500);  
		long roll_output =  (long) constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);  
		long yaw_output =  (long) constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);  

		// mix pid outputs and send to the motors.
		long mfl = rcthr + roll_output + pitch_output - yaw_output;
		long mbl = rcthr + roll_output - pitch_output + yaw_output;
		long mfr = rcthr - roll_output + pitch_output + yaw_output;
		long mbr = rcthr - roll_output - pitch_output - yaw_output;

		hal.rcout->write(MOTOR_FL, mfl);
		hal.rcout->write(MOTOR_BL, mbl);
		hal.rcout->write(MOTOR_FR, mfr);
		hal.rcout->write(MOTOR_BR, mbr);




#if 0
		hal.console->printf("Phi %5.2f The %5.2f Psi %5.2f p %5.2f q %5.2f r %5.2f\n",
			roll,
	      		pitch,
	      		yaw,
	      		gyroRoll,
	      		gyroPitch,
	      		gyroYaw
		);
#endif
#if 0
		hal.console->printf("Phi:%4.2f Theta:%4.2f Psi:%4.1f p:%5.1f q:%5.1f r:%5.1f in_ch3=%ld FL:%ld BL:%ld FR:%ld BR:%ld rcthr:%ld rcyaw:%ld rcpit:%ld rcroll:%ld alt:%5.3f\n",
			roll,
	      		pitch,
	      		yaw,
	      		gyroRoll,
	      		gyroPitch,
	      		gyroYaw,
	      		rcthr,      
	      		mfl,mbl,mfr,mbr,
			rcthr,(long)channels[3],(long)channels[1],(long)channels[0],
			alt
		);
#endif

#if 0
		hal.console->printf("rcthr:%ld rcyaw:%ld rcpit:%ld rcroll:%ld,Bat:%f\n",
			
			rcthr,(long)channels[3],(long)channels[1],(long)channels[0],batVoltage
		);
#endif

#if 1
		timer_m = hal.scheduler->millis();
		hal.console->printf_P(PSTR("%ld %f %f\n"),
			timer_m, alt, alt_f
		);
			
#endif


	}
 
	else {
		// motors off
		hal.rcout->write(MOTOR_FL, 1000);
		hal.rcout->write(MOTOR_BL, 1000);
		hal.rcout->write(MOTOR_FR, 1000);
		hal.rcout->write(MOTOR_BR, 1000);
		   
		// reset yaw target so we maintain this on takeoff
		yaw_target = yaw;

		// reset PID integrals whilst on the ground
		for(int i=0; i<6; i++)pids[i].reset_I();

		baro.update_calibration();

		Fly_state = FLY_STATE_LAND;


		//15ms
#if 0
		hal.console->printf("Phi %5.2f The %5.2f Psi %5.2f p %5.2f q %5.2f r %5.2f\n",
			roll,
	      		pitch,
	      		yaw,
	      		gyroRoll,
	      		gyroPitch,
	      		gyroYaw
		);
#endif
#if 0
		hal.console->printf("rcthr:%ld rcyaw:%ld rcpit:%ld rcroll:%ld,Bat:%f\n",
			rcthr,(long)channels[3],(long)channels[1],(long)channels[0],batVoltage
		);

#endif

#if 1
		
		timer_m = hal.scheduler->millis();
		hal.console->printf_P(PSTR("%ld %f %f th0\n"),
			timer_m, alt, alt_f
			
		);
#endif

	}
	//A2 Low***********************
    	hal.gpio->write(56, LOW);	
	//hal.scheduler->delay(5);

	while(hal.scheduler->micros()-timer_u<10000);
	
	//hal.scheduler->delay(5);	



}

AP_HAL_MAIN();
