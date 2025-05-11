/*  */
/**
  * @file           : actuator_xy.c
  * @brief          : Actuator startup, limit switch control, homing, etc...
  * @author			:r.muhammet.tulay@gmail.com
  */
/*  */

/*  How to use
Before superloop Starts define a time tick variable and call "ActuatorHomingIdleStart()" once
  uint32_t main_time_tick=0;
  uint32_t loop_cnt = 0;
  ActuatorHomingIdleStart(); //needs to be called once before starting ActuatorHomingProcess(), also stops the Actuator

inside the superloop, repeatedly call ActuatorHomingProcess() with incremented time tick,
check return value of the function and decide

		  while (SUPERLOOP)
		  {
		uint8_t HomingRes;
				HomingRes = ActuatorHomingProcess(&main_time_tick);

			  if (HomingRes == 0){				//homing in progress
				  HAL_Delay(1);
				  main_time_tick++;
				  loop_cnt++;
			  }
			  else if(HomingRes == 1) {			//homing done,
				  HAL_Delay(20);
				  //ActuatorHomingIdleStart();	//rerun homing
			  }

			  while(HomingRes > 1);				//Homing Error

		  }END SUPERLOOP
 */


/* Includes ------------------------------------------------------------------*/
#include "actuator_xy.h"


#define	LIMSWSTA_RESTED		0
#define	LIMSWSTA_ACTUATED	1
#define	LIMSWSTA_OMIT 		2

#define		ACT_DIR_XtoY	0
#define		ACT_DIR_YtoX	1



/**
  * @brief  Reads limit switch pins, if pin state persist TBD consecutive times this will be accepted as current pin state
  *
  * @note   reenterant, needs to be called at least TBD times to be able to acquire new pin states
  *
  * @param  XLimSwSta pointer for X Limit Switch status,
  *
  * @param  YLimSwSta pointer for Y Limit Switch status,
  *
  * @retval none
  */

uint8_t ReadLimitSwitches(uint8_t* XLimSwSta, uint8_t* YLimSwSta){
static uint8_t XCnt = 0;
static uint8_t YCnt = 0;

static GPIO_PinState XLastSta = GPIO_PIN_RESET;
static GPIO_PinState XPastSta = GPIO_PIN_RESET;

static GPIO_PinState YLastSta = GPIO_PIN_RESET;
static GPIO_PinState YPastSta = GPIO_PIN_RESET;

GPIO_PinState CurrSta;

	CurrSta = ReadLimitSwitchX_GPIO();
	if(XCnt < 0xFF){
	XCnt++;
	}
	if(XPastSta != CurrSta){
		XCnt = 0;
		XPastSta = CurrSta;
	}
	if(XCnt > LIMSW_SETTLE_CNT){
	XLastSta = CurrSta;
	}
	*XLimSwSta = (uint8_t)XLastSta;



	CurrSta = ReadLimitSwitchY_GPIO();
	if(YCnt < 0xFF){
	YCnt++;
	}
	if(YPastSta != CurrSta){
		YCnt = 0;
		YPastSta = CurrSta;
	}
	if(YCnt > LIMSW_SETTLE_CNT){
	YLastSta = CurrSta;
	}

	*YLimSwSta = (uint8_t)YLastSta;

	if((XCnt > LIMSW_SETTLE_CNT) & (YCnt > LIMSW_SETTLE_CNT)){
		return(1);
	}else{
		return(0);
	}
}

/**
  * @brief  Checks given Limit Switch's transition
  *
  * @note   reenterant, result depends CurrentTime and actual LimitSwitch state which is connected to the GPIO pins
  *
  * @param  X_DesiredSwitchSta  can be (limit switch is) ACTUATED or (limit switch is) RESTED
  *
  * @param  Y_DesiredSwitchSta  can be (limit switch is) ACTUATED or (limit switch is) RESTED
  *
  * @param  TravelDir can be ACT_DIR_XtoY (0) or ACT_DIR_XtoY (1) or DIR_STOP (other than 0 or 1)
  *
  * @param  TimeOutValMIN 				and		TimeOutValMAX can be
  * 		ACT_XtoY_TRAVELTIME_MIN, 			ACT_XtoY_TRAVELTIME_MAX,
  * 		ACT_YtoX_TRAVELTIME_MIN, 			ACT_YtoX_TRAVELTIME_MAX,
  * 		LIMSW_X_CLEARENCE_TIME_MIN,			LIMSW_X_CLEARENCE_TIME_MAX,
  * 		LIMSW_Y_CLEARENCE_TIME_MIN,			LIMSW_Y_CLEARENCE_TIME_MAX,
  *
  * @retval return(0); // transitioned successfully
  * 		return(1); // process is in progress
  * 		return(2); // MAX timeout error
  * 		return(3); // MIN timeout error
  */

uint8_t	CheckLimitSwitchTransition(uint8_t X_DesiredSwitchSta, uint8_t Y_DesiredSwitchSta, uint8_t TravelDir,uint32_t TimeOutValMax, uint32_t TimeOutValMin, uint32_t CurrentTime){

static uint8_t  XLimSwSta;
static uint8_t  YLimSwSta;


	if(TravelDir == ACT_DIR_XtoY){
		MoveActuatorXtoY();
	}
	if(TravelDir == ACT_DIR_YtoX){
		MoveActuatorYtoX();
	}
	if(TravelDir >  1){
		StopActuator();
	}

	ReadLimitSwitches(&XLimSwSta, &YLimSwSta);

	if(CurrentTime >= TimeOutValMax){
		return(2);
	}


	if( (XLimSwSta == X_DesiredSwitchSta) & (YLimSwSta == Y_DesiredSwitchSta) ){
		if(CurrentTime <= TimeOutValMin){
			return(3);
		}

		return(0);
	}



	return(1);
}

typedef enum
{
	ACTUATOR_ZONE_A		= 0x00U,
	ACTUATOR_ZONE_B		= 0x01U,
	ACTUATOR_ZONE_BC	= 0x02U,
	ACTUATOR_ZONE_C		= 0x03U,
	ACTUATOR_ZONE_CD	= 0x04U,
	ACTUATOR_ZONE_D		= 0x05U,
	ACTUATOR_ZONE_E		= 0x06U
}ActuatorZoneTypeDef;


static uint8_t ActuatorState = 203;	/*Be careful it is used throughout the ActuatorHomingProcess function.
									ActuatorHomingStart needs to be called before calling ActuatorHomingProcess() */
void ActuatorHomingIdleStart(){
	ActuatorState = 0;
	StopActuator();
}

uint8_t ActuatorHomingProcess(uint32_t *ActuatorTimeout){

static uint8_t ActErrSta = 0;
static uint32_t XtoYTravelTime = 0;
static uint32_t YtoXTravelTime = 0;
static ActuatorZoneTypeDef ActuatorZone = ACTUATOR_ZONE_A;

uint8_t ActTemp;

static uint8_t  XLimSwSta;
static uint8_t  YLimSwSta;

	switch(ActuatorState){


	case(0):
		StopActuator();

		XtoYTravelTime = 0;
		YtoXTravelTime = 0;
		ActuatorZone = ACTUATOR_ZONE_A;
		*ActuatorTimeout = 0;
		ActuatorState = 1;
	break;//0

	case(1):
		if(ReadLimitSwitches(&XLimSwSta, &YLimSwSta) ){
			ActuatorState = 2;
		}

		if(*ActuatorTimeout > LIMSW_DEBOUNCE_TIME_MAX){	//MAX timeout ERROR
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 202;		//Switch to error state
		}
	break;//1


	case(2):
		if( (XLimSwSta == LIMSWSTA_RESTED) & (YLimSwSta == LIMSWSTA_RESTED) ){
			*ActuatorTimeout = 0;
			ActuatorState = 20;
		}

		if( (XLimSwSta == LIMSWSTA_RESTED) & (YLimSwSta == LIMSWSTA_ACTUATED) ){
			*ActuatorTimeout = 0;
			ActuatorState = 30;
		}

		if( (XLimSwSta == LIMSWSTA_ACTUATED) & (YLimSwSta == LIMSWSTA_RESTED) ){
			*ActuatorTimeout = 0;
			ActuatorState = 40;
		}

		if( (XLimSwSta == LIMSWSTA_ACTUATED) & (YLimSwSta == LIMSWSTA_ACTUATED) ){
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 202;		//Switch to error state
		}
	break;//2


	case(20):
	MoveActuatorXtoY();

	if(*ActuatorTimeout < ACT_XtoY_TRAVELTIME_MAX){

		if(ReadLimitSwitches(&XLimSwSta, &YLimSwSta)){
			if( (XLimSwSta == LIMSWSTA_ACTUATED) & (YLimSwSta == LIMSWSTA_RESTED) ){
				StopActuator();
				*ActuatorTimeout = 0;
				ActuatorState = 40;
			}
			if( (XLimSwSta == LIMSWSTA_RESTED) & (YLimSwSta == LIMSWSTA_ACTUATED) ){
				StopActuator();
				*ActuatorTimeout = 0;
				ActuatorState = 30;
			}
		}
	}
	else{
		StopActuator();
		*ActuatorTimeout = 0;
		ActuatorState = 21;		//
	}
	break;//20

	case(21):
	MoveActuatorYtoX();

	if(*ActuatorTimeout < ACT_YtoX_TRAVELTIME_MAX){

		if(ReadLimitSwitches(&XLimSwSta, &YLimSwSta)){
			if( (XLimSwSta == LIMSWSTA_ACTUATED) & (YLimSwSta == LIMSWSTA_RESTED) ){
				StopActuator();
				*ActuatorTimeout = 0;
				ActuatorState = 40;
			}
			if( (XLimSwSta == LIMSWSTA_RESTED) & (YLimSwSta == LIMSWSTA_ACTUATED) ){
				StopActuator();
				*ActuatorTimeout = 0;
				ActuatorState = 30;
			}
		}
	}
	else{
		StopActuator();
		*ActuatorTimeout = 0;
		ActuatorState = 202;		//
	}
	break;//21


	case(30):

	MoveActuatorYtoX();

	if(*ActuatorTimeout < ACT_YtoX_TRAVELTIME_MAX){

		if(ReadLimitSwitches(&XLimSwSta, &YLimSwSta)){
			if( (XLimSwSta == LIMSWSTA_ACTUATED) & (YLimSwSta == LIMSWSTA_RESTED) ){
				StopActuator();
				*ActuatorTimeout = 0;
				ActuatorState = 40;
			}
		}
	}
	else{
		StopActuator();
		*ActuatorTimeout = 0;
		ActuatorState = 202;		//
	}
	break;//30


	case(40):
		ActTemp = CheckLimitSwitchTransition(LIMSWSTA_RESTED, LIMSWSTA_RESTED, ACT_DIR_XtoY, LIMSW_X_CLEARENCE_TIME_MAX, LIMSW_X_CLEARENCE_TIME_MIN, *ActuatorTimeout);

		if(ActTemp == 0){
			*ActuatorTimeout = 0;
			ActuatorState = 44;
		}

		if(ActTemp == 2){				//MAX timeout ERROR
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 202;		//Switch to error state
		}

		if(ActTemp == 3){				//MIN timeout error
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 203;		//Switch to error state
		}
	break;//40

	case(44):
		ActTemp = CheckLimitSwitchTransition(LIMSWSTA_RESTED, LIMSWSTA_ACTUATED, ACT_DIR_XtoY, ACT_XtoY_TRAVELTIME_MAX, ACT_XtoY_TRAVELTIME_MIN, *ActuatorTimeout);

		if(ActTemp == 0){
			XtoYTravelTime = *ActuatorTimeout;
			*ActuatorTimeout = 0;
			ActuatorState = 45;
		}

		if(ActTemp == 2){				//MAX timeout ERROR
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 202;		//Switch to error state
		}

		if(ActTemp == 3){				//MIN timeout error
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 203;		//Switch to error state
		}
	break;//44

	case(45):
		ActTemp = CheckLimitSwitchTransition(LIMSWSTA_RESTED, LIMSWSTA_RESTED, ACT_DIR_XtoY, LIMSW_Y_CLEARENCE_TIME_MAX, LIMSW_Y_CLEARENCE_TIME_MIN, *ActuatorTimeout);

		if(ActTemp == 0){
			StopActuator();
			*ActuatorTimeout = 0;
			ActuatorState = 46;
		}

		if(ActTemp == 2){				//MAX timeout ERROR
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 202;		//Switch to error state
		}

		if(ActTemp == 3){			//MIN timeout error
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 203;		//Switch to error state
		}
	break;//45


	case(46):
		ActTemp = CheckLimitSwitchTransition(LIMSWSTA_RESTED, LIMSWSTA_ACTUATED, ACT_DIR_YtoX, LIMSW_Y_CLEARENCE_TIME_MAX, LIMSW_Y_CLEARENCE_TIME_MIN, *ActuatorTimeout);

		if(ActTemp == 0){
			*ActuatorTimeout = 0;
			ActuatorState = 47;
		}

		if(ActTemp == 2){				//MAX timeout ERROR
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 202;		//Switch to error state
		}

		if(ActTemp == 3){			//MIN timeout error
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 203;		//Switch to error state
		}
	break;//46


	case(47):
		ActTemp = CheckLimitSwitchTransition(LIMSWSTA_RESTED, LIMSWSTA_RESTED, ACT_DIR_YtoX, LIMSW_Y_CLEARENCE_TIME_MAX, LIMSW_Y_CLEARENCE_TIME_MIN, *ActuatorTimeout);

		if(ActTemp == 0){
			*ActuatorTimeout = 0;
			ActuatorState = 48;
		}

		if(ActTemp == 2){				//MAX timeout ERROR
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 202;		//Switch to error state
		}

		if(ActTemp == 3){				//MIN timeout error
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 203;		//Switch to error state
		}
	break;//47


	case(48):
		ActTemp = CheckLimitSwitchTransition(LIMSWSTA_ACTUATED, LIMSWSTA_RESTED, ACT_DIR_YtoX, ACT_YtoX_TRAVELTIME_MAX, ACT_YtoX_TRAVELTIME_MIN, *ActuatorTimeout);

		if(ActTemp == 0){
			YtoXTravelTime = *ActuatorTimeout;
			*ActuatorTimeout = 0;
			ActuatorState = 49;
		}

		if(ActTemp == 2){				//MAX timeout ERROR
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 202;		//Switch to error state
		}

		if(ActTemp == 3){				//MIN timeout error
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 203;		//Switch to error state
		}
	break;//48


	case(49):
		ActTemp = CheckLimitSwitchTransition(LIMSWSTA_RESTED, LIMSWSTA_RESTED, ACT_DIR_YtoX, LIMSW_X_CLEARENCE_TIME_MAX, LIMSW_X_CLEARENCE_TIME_MIN, *ActuatorTimeout);

		if(ActTemp == 0){
			ActuatorZone = ACTUATOR_ZONE_B;
			StopActuator();
			*ActuatorTimeout = 0;
			ActuatorState = 50;
		}

		if(ActTemp == 2){				//MAX timeout ERROR
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 202;		//Switch to error state
		}

		if(ActTemp == 3){				//MIN timeout error
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 203;		//Switch to error state
		}
	break;//49

	case(50):
		ActTemp = CheckLimitSwitchTransition(LIMSWSTA_ACTUATED, LIMSWSTA_RESTED, ACT_DIR_XtoY, LIMSW_X_CLEARENCE_TIME_MAX, LIMSW_X_CLEARENCE_TIME_MIN, *ActuatorTimeout);

		if(ActTemp == 0){
			ActuatorZone = ACTUATOR_ZONE_BC;
			*ActuatorTimeout = 0;
			ActuatorState = 51;
		}

		if(ActTemp == 2){				//MAX timeout ERROR
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 202;		//Switch to error state
		}

		if(ActTemp == 3){				//MIN timeout error
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 203;		//Switch to error state
		}
	break;//50

	case(51):
		ActTemp = CheckLimitSwitchTransition(LIMSWSTA_RESTED, LIMSWSTA_RESTED, ACT_DIR_XtoY, LIMSW_X_CLEARENCE_TIME_MAX, LIMSW_X_CLEARENCE_TIME_MIN, *ActuatorTimeout);

		if(ActTemp == 0){
			//StopActuator();
			ActuatorZone = ACTUATOR_ZONE_C;
			*ActuatorTimeout = 0;
			ActuatorState = 52;
		}

		if(ActTemp == 2){				//MAX timeout ERROR
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 202;		//Switch to error state
		}

		if(ActTemp == 3){				//MIN timeout error
			ActErrSta = ActuatorState;	//Record the error source
			ActuatorState = 203;		//Switch to error state
		}
	break;//51

	case(52):
		ActTemp = CheckLimitSwitchTransition(LIMSWSTA_OMIT, LIMSWSTA_OMIT, ACT_DIR_XtoY, (uint32_t)((double)XtoYTravelTime*0.5), ACT_XtoY_TRAVELTIME_MIN, *ActuatorTimeout);

		if(ActTemp == 2){		//we selected XtoYTravelTime*0.5 as timeout value, so timeout means we reached the middle point aka HOME
			StopActuator();
			*ActuatorTimeout = 0;
			ActuatorState = 53;
		}
	break;//52


	case(53):
	StopActuator();
	break;//53


	case(202):		//MAX Timeout Error
		StopActuator();
	break;//202

	case(203):		//MIN Timeout Error
		StopActuator();
	break;//203

	default:
	break;

	}//switch end


	if(ActuatorState == 53){	//Homing Successful
		return(1);
	}

	if(ActuatorState < 53){		//Homing in progress
		return(0);
	}

	if(ActuatorState > 53){		//Homing Error
		return(ActErrSta);
	}

}







