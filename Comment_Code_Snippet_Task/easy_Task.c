/*   The functions below Init mpu6050, read the Accelerometer, Gyroscope and temperature values 
     seperately or all of them and assign values to DataStruct pointer to a MPU6050_t structure that contains
     the configuration information for the specified DataStruct.*/
#include <math.h>
#include "mpu6050.h"

/** @defgroup MPU6050 registers address definitions
  * @{
  */
#define WHOAMI 0x75
#define PWR_MGMT 0x6B
#define SMPLRT_DIV 0x19
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H 0x41
#define GYRO_CONFIG 0x1B
#define GYRO_XOUT_H 0x43
#define MPU6050_ADDR 0xD0 /* Default I2C address */

#define RAD2DEG 57.295779513082320876798154814105 //radian to degree value definition
const uint16_t I2C_TIMEOUT = 100; //Define I2C timeout value
const double Acc_Z_corrector = 14418.0; //Define Z corretcotr value
uint32_t timer;


/** @defgroup FilterX structure definition
  * @{
  */
Filter_t FilterX = {
    .Q_ANGLE = 0.001f,
    .Q_BIAS = 0.003f,
    .R_MEASURE = 0.03f};
/** @defgroup FilterY structure definition
  * @{
  */
Filter_t FilterY = {
    .Q_ANGLE = 0.001f,
    .Q_BIAS = 0.003f,
    .R_MEASURE = 0.03f,
};


/**
  * @brief  Initializes the MPU6050 according to the specified memory addresses
  * @param  I2C pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @retval Status
  *          - 0 : Init OK
  *          - 1 : Init ERROR
  */
uint8_t MPU_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check; //Variable to check the return of WHO_AM_I
    uint8_t Data; //Variable to send configuration of register below

    /*Read WHO_AM_I register default value*/ 
    /*if return 0x68 = 104, it matches*/
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHOAMI, 1, &check, 1, I2C_TIMEOUT);
    
    if (check == 104) 
    {
        Data = 0;
            /*Configure PWR_MGMT_1 register */ 
            /*It is specified via "Data" variable value above*/
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT, 1, &Data, 1, I2C_TIMEOUT);

        Data = 0x07;
            /*Configure SMPRT_DIV register */ 
            /*It is specified via "Data" variable value above*/
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1, I2C_TIMEOUT);

        Data = 0x00;
            /*Configure ACCEL_CONFIG register */ 
            /*It is specified via "Data" variable value above*/
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, I2C_TIMEOUT);

        Data = 0x00;
            /*Configure GYRO_CONFIG register */ 
            /*It is specified via "Data" variable value above*/
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, I2C_TIMEOUT);
        return 0;
    }
    return 1;
}


/**
  * @brief  Read X, Y, Z Accelerometer values.
  * @param  I2C pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param  DataStruct pointer to a MPU6050_t structure that contains
  *         the configuration information for the specified DataStruct.
  * @retval None
  */
void Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6]; //Array for read out splitted 8-bit X, Y, Z values

    /*Read out X, Y, Z values splitted into two 8-bit*/
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 6, I2C_TIMEOUT);

    /*Merge the splitted X, Y, Z values into 16-bit variable by shifting*/
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]); 
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]); 
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*Scale the raw X, Y values via AFS_SEL value and assign.*/
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0; 
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;

    DataStruct->Az = DataStruct->Accel_Z_RAW / Acc_Z_corrector; //Scale the raw Z value via Acc_Z_corrector value and assign.
}



/**
  * @brief  Read X, Y, Z Gyroscope values.
  * @param  I2C pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param  DataStruct pointer to a MPU6050_t structure that contains
  *         the configuration information for the specified DataStruct.
  * @retval None
  */
void Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6]; //Array for read out splitted 8-bit X, Y, Z values

    /*Read out X, Y, Z values splitted into two 8-bit*/
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6, I2C_TIMEOUT);

    /*Merge the splitted X, Y, Z values into 16-bit variable by shifting*/
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]); 
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]); 
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]); 

    /*Scale the raw X, Y, Z values via FS_SEL table and assign.*/
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0; 
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0; 
}


/**
  * @brief  Read temperature value.
  * @param  I2C pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param  DataStruct pointer to a MPU6050_t structure that contains
  *         the configuration information for the specified DataStruct.
  * @retval None
  */
void Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2]; //Array for reading out splitted 8-bit temperature value
    int16_t temp; //Raw temperature variable

    /*Read out the temperature value splitted into two 8-bit*/
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H, 1, Rec_Data, 2, I2C_TIMEOUT);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]); //Merge the splitted temperature value into 16-bit variable by shifting
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);//Scale and assign true temperature value
}


/**
  * @brief  Read accelerometer, temperature and gyroscope values.
  * @param  I2C pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param  DataStruct pointer to a MPU6050_t structure that contains
  *         the configuration information for the specified DataStruct.
  * @retval None
  */
void Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14]; //Array for reading out splitted 8-bit accelerometer, temperature and gyroscope values.
    int16_t temp; //Raw temperature variable

    /*Read out the accelerometer, temperature and gyroscope values splitted into 8-bit values*/
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 14, I2C_TIMEOUT);

    /*Merge the splitted X, Y, Z values into 16-bit variable by shifting*/
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);//Merge the splitted temperature value into 16-bit variable by shifting
    
     /*Merge the splitted X, Y, Z values into 16-bit variable by shifting*/
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    /*Scale the raw X, Y values via AFS_SEL value and assign.*/
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0; 
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;

    DataStruct->Az = DataStruct->Accel_Z_RAW / Acc_Z_corrector; //Scale the raw Z value via Acc_Z_corrector value and assign.

    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53); //Scale and assign true temperature value

    /*Scale the raw X, Y, Z values via FS_SEL table and assign.*/
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0; 

    /*Find the angle of Y for filtering*/
    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW); //finds the length of the accel vector.
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD2DEG;
    }
    else
    {
        roll = 0.0;
    }
    
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD2DEG;
    if ((pitch < -90 && DataStruct->FilterAngleY > 90) || (pitch > 90 && DataStruct->FilterAngleY < -90))
    {
        FilterY.angle = pitch;
        DataStruct->FilterAngleY = pitch;
    }
    else
    {
        DataStruct->FilterAngleY = Filter_getAngle(&FilterY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->FilterAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->FilterAngleX = Filter_getAngle(&FilterX, roll, DataStruct->Gx, dt);
}


/**
  * @brief  Filter the angels
  * @param  Filter pointer to a Filter_t structure that contains
  *         the configuration information for the specified Filter.
  * @param  newAngle 
  * @param  newRate 
  * @param  dt 
  * @retval Angel of the axis
  */
double Filter_getAngle(Filter_t *Filter, double newAngle, double newRate, double dt)
{
    double rate = newRate - Filter->bias;
    Filter->angle += dt * rate;

    Filter->P[0][0] += dt * (dt * Filter->P[1][1] - Filter->P[0][1] - Filter->P[1][0] + Filter->Q_ANGLE);
    Filter->P[0][1] -= dt * Filter->P[1][1];
    Filter->P[1][0] -= dt * Filter->P[1][1];
    Filter->P[1][1] += Filter->Q_BIAS * dt;

    double S = Filter->P[0][0] + Filter->R_MEASURE;
    double K[2];
    K[0] = Filter->P[0][0] / S;
    K[1] = Filter->P[1][0] / S;

    double y = newAngle - Filter->angle;
    Filter->angle += K[0] * y;
    Filter->bias += K[1] * y;

    double P00_temp = Filter->P[0][0];
    double P01_temp = Filter->P[0][1];

    Filter->P[0][0] -= K[0] * P00_temp;
    Filter->P[0][1] -= K[0] * P01_temp;
    Filter->P[1][0] -= K[1] * P00_temp;
    Filter->P[1][1] -= K[1] * P01_temp;

    return Filter->angle;
};