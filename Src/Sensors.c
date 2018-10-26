#include "Sensors.h"
#include "global.h"
#include "communication.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

struct SensorsConstants SensCons;

uint8_t SensorsInit(struct Robot *robot, uint8_t sensors)
{
	// For all sensors
		uint8_t buf[22];
		uint8_t hr;
	
	// For different sensors
	if(sensors & SENS_PRESSURE) {
				//TODO
	}
	
	if(sensors & SENS_IN_PRESSURE) {
		receiveI2CPackageFromRegisters(DEV_I2C, IN_PRESSURE_Addr, IN_PRESSURE_ID_REG, buf, 1);
		if(buf[0] != IN_PRESSURE_ID) return ERR_SENS_NOT_DETECTED;
		
		// Load data from colib registers
		receiveI2CPackageFromRegisters(DEV_I2C, IN_PRESSURE_Addr, IN_PRESSURE_CALIB_REG, buf, 22);
		SensCons.InPressureCons.AC1 = (buf[0] << 8) + buf[1];
		SensCons.InPressureCons.AC2 = (buf[2] << 8) + buf[3];
		SensCons.InPressureCons.AC3 = (buf[4] << 8) + buf[5];
		SensCons.InPressureCons.AC4 = (buf[6] << 8) + buf[7];
		SensCons.InPressureCons.AC5 = (buf[8] << 8) + buf[9];
		SensCons.InPressureCons.AC6 = (buf[10] << 8) + buf[11];
		SensCons.InPressureCons.B1 = (buf[12] << 8) + buf[13];
		SensCons.InPressureCons.B2 = (buf[14] << 8) + buf[15];
		SensCons.InPressureCons.MB = (buf[16] << 8) + buf[17];
		SensCons.InPressureCons.MC = (buf[18] << 8) + buf[19];
		SensCons.InPressureCons.MD = (buf[20] << 8) + buf[21];
		
		SensCons.InPressureCons.oss = 1;// Standart sempling
		
		// Relise temp measurrment
		SendByteToRegisterByI2C(DEV_I2C, IN_PRESSURE_Addr, IN_PRESSURE_CONTR_REG, IN_PRESSURE_MEASURMENT_TEMP);
		HAL_Delay(MEASUREMENT_DELAY);
		hr = ReliseMeansurements(&Q100, SENS_IN_PRESSURE);
		if(hr != S_OK) return hr;
		SensCons.InPressureCons.UT = SensCons.InPressureCons.UP >> SensCons.InPressureCons.oss;
		
		// Culculate pressure const
		int32_t X1 = ((SensCons.InPressureCons.UT - SensCons.InPressureCons.AC6) * SensCons.InPressureCons.AC5) >> 15;
		int32_t X2 = (SensCons.InPressureCons.MC << 11) / (X1 + SensCons.InPressureCons.MD);
		int32_t B5 = X1 + X2;
		int32_t B6 = B5 - 4000;
		X1 = (SensCons.InPressureCons.B2 * (B6 * (B6 >> 12))) >> 11;
		X2 = SensCons.InPressureCons.AC2 * (B6 >> 11);
		int32_t X3 = X1 + X2;
		SensCons.InPressureCons.B3 = (((SensCons.InPressureCons.AC1 * 4 + X3) << SensCons.InPressureCons.oss) + 2) >> 2;
		X1 = SensCons.InPressureCons.AC3 * (B6 >> 13);
		X2 = (SensCons.InPressureCons.B1 * (B6 * (B6 >> 12))) >> 16;
		X3 = ((X1 + X2) + 2) >> 2;
		SensCons.InPressureCons.B4 = (SensCons.InPressureCons.AC4 * (uint32_t)(X3 + 32768)) >> 15;
			
		hr = StartMeansurements(&Q100, SENS_IN_PRESSURE);
		HAL_Delay(MEASUREMENT_DELAY);

		return S_OK;
	}
	
	if(sensors & SENS_LEAK_0) {
				//TODO
	}
	
	if(sensors & SENS_LEAK_1) {
				//TODO
	}
	return S_OK;
}


uint8_t StartMeansurements(struct Robot *robot, uint8_t sensors)
{
	if(sensors & SENS_PRESSURE) {
		//TODO
	}
	
	if(sensors & SENS_IN_PRESSURE) {
		//uint8_t oss = 1;// Standart discr.
		SendByteToRegisterByI2C(DEV_I2C, IN_PRESSURE_Addr, IN_PRESSURE_CONTR_REG, IN_PRESSURE_MEASURMENT_PRES + (SensCons.InPressureCons.oss << 6));
	}
	
	if(sensors & SENS_LEAK_0) {
		//TODO
	}
	
	if(sensors & SENS_LEAK_1) {
		//TODO
	}
	
	return S_OK;
}


uint8_t ReliseMeansurements(struct Robot *robot, uint8_t sensors)
{
	uint8_t buf[8];
	
	if(sensors & SENS_PRESSURE) {
				//TODO
	}
	
	if(sensors & SENS_IN_PRESSURE) {
		receiveI2CPackageFromRegisters(DEV_I2C, IN_PRESSURE_Addr, IN_PRESSURE_CONTR_REG, buf, 1);
		if(~buf[0] & IN_PRESSURE_MEASURMENT_COMPLITE)
		{
			receiveI2CPackageFromRegisters(DEV_I2C, IN_PRESSURE_Addr, IN_PRESSURE_DATA_REG, buf, 3);
			SensCons.InPressureCons.UP = ((buf[0] << 16) + (buf[1] << 8) + buf[2]) >> (8 - SensCons.InPressureCons.oss);
		} else {
			osDelay(MEASUREMENT_DELAY);
			receiveI2CPackageFromRegisters(DEV_I2C, IN_PRESSURE_Addr, IN_PRESSURE_CONTR_REG, buf, 1);
			if(~buf[0] & IN_PRESSURE_MEASURMENT_COMPLITE)
			{
				receiveI2CPackageFromRegisters(DEV_I2C, IN_PRESSURE_Addr, IN_PRESSURE_DATA_REG, buf, 3);
				SensCons.InPressureCons.UP = ((buf[0] << 16) + (buf[1] << 8) + buf[2]) >> (8 - SensCons.InPressureCons.oss);
			} else {
				return ERR_SENS_NOT_REQUEST;
			}
		}
		return S_OK;
	}
	
	if(sensors & SENS_LEAK_0) {
				//TODO
	}
	
	if(sensors & SENS_LEAK_1) {
				//TODO
	}
	
	return S_OK;
}


void CulculationResults(struct Robot *robot, uint8_t sensors)
{
	if(sensors & SENS_PRESSURE) {
		
	}
	
	if(sensors & SENS_IN_PRESSURE) {
		uint32_t B7 = ((uint32_t)SensCons.InPressureCons.UP - SensCons.InPressureCons.B3) * (50000 >> SensCons.InPressureCons.oss);
		
		int32_t p;
		if(B7 < 0x80000000) p = (B7 * 2) / SensCons.InPressureCons.B4;
		else p = (B7 / SensCons.InPressureCons.B4) * 2;
		
		int32_t X1 = (p >> 8) * (p >> 8);
		X1 = (X1 * 3038) >> 16;
		int32_t X2 = (7357 * p) >> 16;
		p = p + ((X1 + X2 + 3791) >> 4);
		
		robot->i_sensors.in_pressure = (uint32_t)p;
		robot->f_sensors.in_pressure = (float)p;
	}
	
	if(sensors & SENS_LEAK_0) {
		
	}
	
	if(sensors & SENS_LEAK_1) {
		
	}
}