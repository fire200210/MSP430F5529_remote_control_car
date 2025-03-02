#include <msp430.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/*--- 參考作者 Artful Bytes: https://github.com/artfulbytes/vl6180x_vl53l0x_msp430 ---*/
#include "drivers/i2c.h"
#include "drivers/vl53l0x.h"
/*----------------------------------------------------------------------*/

#include "ADCelectricity/electricity.h"

/*--- 參考作者 art-wenyi: https://github.com/art-wenyi/MSP430_MPU9250 ---*/
#include "MPU9250_MPU6500/i2c.h"
#include "MPU9250_MPU6500/mpu9250_msp430.h"
#include "MPU9250_MPU6500/mpu9250_calibrate.h"
/*----------------------------------------------------------------------*/

//=====================================MPU6500變數============================================//
#define MPU6500_sensor

// sensor for all data from mpu9250
struct Sensor sensor = {
        //.gyro_reso = 1000.0/32768.0, // dafault use 1000dps   < °/s >
        //.accel_reso = 835.918,       // default use 4g/s      < m/s² >
        //.mag_reso = 4912.0/32760.0,   // default mag 16bit reso
        .convert = &Mpu_Convert_Data,   // register the function
        //.convert_asa = &Mpu_Convert_Asa
};

// calibrated sensor data
struct Sensor_Calibrate sensor_calibrate = {0};

char MPU6500_concated[40] = {0};
unsigned char MPU6500_CIPSEND_0[] = "AT+CIPSEND=0,39\r\n";  //MPU6500傳送字串
unsigned char MPU6500_CIPSEND_1[] = "AT+CIPSEND=1,39\r\n";
unsigned char MPU6500_CIPSEND_2[] = "AT+CIPSEND=2,39\r\n";
unsigned char MPU6500_CIPSEND_3[] = "AT+CIPSEND=3,39\r\n";
unsigned char MPU6500_CIPSEND_4[] = "AT+CIPSEND=4,39\r\n";
//=========================================================================================//

//=====================================電量偵測變數============================================//
//#define ADC_Battery_Detection

uint16_t Battery_Detection;
//=========================================================================================//

//=======================================雷射變數=============================================//
#define Laser_Management

uint16_t ranges[4] = {0};
uint16_t range1, range2, range3, range4;
float    range_1, range_2, range_3, range_4;

//char VL53L0X_concated[25] = {0};
//unsigned char VL53L0X_CIPSEND_0[] = "AT+CIPSEND=0,24\r\n";  //VL53L0X傳送字串
//unsigned char VL53L0X_CIPSEND_1[] = "AT+CIPSEND=1,24\r\n";
//unsigned char VL53L0X_CIPSEND_2[] = "AT+CIPSEND=2,24\r\n";
//unsigned char VL53L0X_CIPSEND_3[] = "AT+CIPSEND=3,24\r\n";
//unsigned char VL53L0X_CIPSEND_4[] = "AT+CIPSEND=4,24\r\n";
//=========================================================================================//

//=======================================舵機PWM計算==========================================//
#define MCU_CLOCK           1000000
#define PWM_FREQUENCY       48      // 以赫茲為單位，理想情況下為 50Hz。
#define SERVO_STEPS         180     // 以度為單位的最大步數（180 是常見的）
#define SERVO_MIN           1000    // 最小占空比
#define SERVO_MAX           2450    // 最大佔空比
unsigned int PWM_Period     = (MCU_CLOCK / PWM_FREQUENCY);  // PWM 週期
unsigned int PWM_Duty       = 0;                            // %

unsigned int servo_stepval, servo_stepnow;
unsigned int servo_lut[ SERVO_STEPS+1 ];
unsigned int i;

unsigned int Positive, negative;
//=========================================================================================//

//=======================================煞車舵機PWM計算=======================================//
#define Brakes_MCU_CLOCK           1000000
#define Brakes_PWM_FREQUENCY       48      // 以赫茲為單位，理想情況下為 50Hz。
#define Brakes_SERVO_STEPS         180     // 以度為單位的最大步數（180 是常見的）
#define Brakes_SERVO_MIN           1000    // 最小占空比
#define Brakes_SERVO_MAX           2450    // 最大佔空比
unsigned int Brakes_PWM_Period     = (Brakes_MCU_CLOCK / Brakes_PWM_FREQUENCY);  // PWM 週期
unsigned int Brakes_PWM_Duty       = 0;                            // %

unsigned int Brakes_servo_stepval, Brakes_servo_stepnow;
unsigned int Brakes_servo_lut[ Brakes_SERVO_STEPS+1 ];
unsigned int Brakes_i;

//struct CAR_Stop
//{
//    bool STOP_Forward = false, STOP_Back  = false,
//        STOP_Left    = false, STOP_Right = false,
//        STOP         = false, OPEN       = false;
//};

bool STOP_Forward      = false, STOP_Back      = false,
     STOP_Left         = false, STOP_Right     = false,
     STOP              = false, OPEN           = false,
     Auto_STOP_Forward = false, Auto_STOP_Back = false,
     Manual_STOP       = false;
//=========================================================================================//

//======================================馬達PWM計算===========================================//
#define SMCLK_FREQ 1045000 //頻率
#define PWM_DUTY_SEC 0.001 //1ms 1kHz
#define PWM_PERIOD ((SMCLK_FREQ*PWM_DUTY_SEC)-1)
#define PWM_PULSE_WIDE(x) ((x * 0.01) * PWM_PERIOD)
//=========================================================================================//

//========================================AT指令============================================//
unsigned char CWMODE[] = "AT+CWMODE=3\r\n"; //'\r'回車，'\n'換行
unsigned char CIPMUX[] = "AT+CIPMUX=1\r\n";
unsigned char CIPSERVER[] = "AT+CIPSERVER=1,8080\r\n";
unsigned char CIPSEND_0[] = "AT+CIPSEND=0,9\r\n";  //傳送字串
unsigned char CIPSEND_1[] = "AT+CIPSEND=1,9\r\n";
unsigned char CIPSEND_2[] = "AT+CIPSEND=2,9\r\n";
unsigned char CIPSEND_3[] = "AT+CIPSEND=3,9\r\n";
unsigned char CIPSEND_4[] = "AT+CIPSEND=4,9\r\n";
//unsigned char CWSAP[] = "AT+CWSAP=\"⠀⠀⠀⠀⠀⠀⠀⠀\",\"12345678\",1,0,4,0\r\n";  //更改名稱
char concated[10] = {0};
int TX;
int AT_string, AT_count = 0;
//=========================================================================================//

//====================================UART資料處理及變數========================================//
unsigned char rx_data[100];     //TODO 可能需要更改 //封包資料暫存區
unsigned char index = 0;        //暫存寫入索引(目前寫到哪裡)

//-------------------------------------------控制變數------------------------------------------
float x;
int   y;
uint16_t MaxSpeed, WarningDistance, BrakingDistance;
int      Speed, Angle, Forward, Reverse;

//----------------------------------------手機端確認頭尾碼---------------------------------------
int check_header_A(){
    return rx_data[12] == 0x25 && rx_data[13]|rx_data[14]|rx_data[15] == 0;
}
int check_footer_A(){
    return  rx_data[16] == 0x0F && rx_data[26] == 0x52;
}
int check_footer_B(){
    return  rx_data[16] == 0x14 && rx_data[31] == 0x52;
}
int check_footer_C(){
    return  rx_data[16] == 0x19 && rx_data[36] == 0x52;
}

int check_change_data_A(){
    return  rx_data[27] == 0x15 || rx_data[27] == 0x16 || rx_data[27] == 0x17;
}
//----------------------------------------電腦端確認頭尾碼----------------------------------------
int check_header_A_1(){
    return rx_data[12] == 0x4A && rx_data[13] == 0 && rx_data[39] == 0 && rx_data[40] == 0xA4;
}
//-------------------------------------------頭碼判定-------------------------------------------
int Header(){
    return  rx_data[12] == 0x25 || rx_data[12] == 0x4A;
}
//===========================================================================================//
void UART_Mobile_terminal(void);
void UART_Computer_terminal(void);
void Steering_Gear_Control(void);
void Car_Speed(void);

void Check_connect(void);

void Senser_Transport(void);
void Transport_String(void);
void MPU6500_Senser_Transport(void);
void MPU6500_Transport_String(void);
//void VL53L0X_Senser_Transport(void);
//void VL53L0X_Transport_String(void);

void TimerInit(void);
void UATR_Init(void);
void Servo_Init(void);
void Brakes_Servo_Init(void);
void Timer_Interrupt_Init(void);

void Stop_Event(void);
void Car_Warning_Distance(void);

void AT_command(void);
void Servo_PWM_control(void);
void Brakes_Servo_PWM_control(void);
//===========================================================================================//
int main(void)
{
    WDTCTL = WDTPW + WDTHOLD; // Stop WDT

    P6DIR |=  BIT3;
    P6OUT &=~ BIT3;

#ifdef Laser_Management
    i2c_init();
    bool success = vl53l0x_init();
#endif

#ifdef MPU6500_sensor
    Mpu_Init();
#endif

    Servo_Init();
    Servo_PWM_control(); //TODO 試試加到初始化執行

    Brakes_Servo_Init();
    Brakes_Servo_PWM_control();
    //TA2CCR2 = Brakes_servo_lut[60];

    Timer_Interrupt_Init();
    TimerInit();
    UATR_Init();

#ifdef ADC_Battery_Detection
    ADC_init(); // 初始化ADC
#endif
    __delay_cycles(5000000);//延遲發送AT指令，等待ESP32 reset 時間
    AT_command();

    __bis_SR_register(GIE);  // interrupts enabled
    __no_operation();        // For debugger

     //__enable_interrupt(); //可能需要移除

    while(1)
    {
//        if((rx_data[12] == 0x25) || (rx_data[12] == 0x4A))
//        {
//            Stop_Event();  //前後左右煞車警告事件
//        }

#ifdef Laser_Management

        if(success)
        {
            success = vl53l0x_read_range_single(VL53L0X_IDX_FIRST, &ranges[0]);

            range1 = ranges[0] - 50; //誤差值

            if((rx_data[12] == 0x25) || (rx_data[12] == 0x4A))
            {
                if((range1 <= 1000) && (range1 >= 1))
                {
                    range_1 = range1 * 0.1;
                    strcat(concated, "\x13\x01");
                    strcat(concated, "\x21");
                    memcpy(&concated[3], &range_1, 4);
                    strcat(concated, "\x01\x31");

                    if ((concated[0] == '\x13') && (concated[1] == '\x01') && (concated[7] == '\x01') && (concated[8] == '\x31'))
                        Senser_Transport();

                    for(TX = 0; TX <= 10; TX ++) concated[TX]=0;
                }
                else if(range1 == 8140)
                {
                    strcat(concated, "\x13\x01\x21\x4F\x56\x45\x52\x01\x31");
                    Senser_Transport();
                    for(TX = 0; TX <= 10; TX ++) concated[TX]=0;
                }
            }

    #ifdef VL53L0X_SECOND

            success &= vl53l0x_read_range_single(VL53L0X_IDX_SECOND, &ranges[1]);

            range2 = ranges[1] - 50;

            if((rx_data[12] == 0x25) || (rx_data[12] == 0x4A))
            {
                if((range2 <= 1000) && (range2 >= 1))
                {
                    range_2 = range2 * 0.1;
                    strcat(concated, "\x13\x01");
                    strcat(concated, "\x22");
                    memcpy(&concated[3], &range_2, 4);
                    strcat(concated, "\x01\x31");

                    if ((concated[0] == '\x13') && (concated[1] == '\x01') && (concated[7] == '\x01') && (concated[8] == '\x31'))
                        Senser_Transport();

                    for(TX = 0; TX <= 10; TX ++) concated[TX]=0;
                }
                else if(range2 == 8140)
                {
                    strcat(concated, "\x13\x01\x22\x4F\x56\x45\x52\x01\x31");
                    Senser_Transport();
                    for(TX = 0; TX <= 10; TX ++) concated[TX]=0;
                }
            }

    #endif
    #ifdef VL53L0X_THIRD

            success &= vl53l0x_read_range_single(VL53L0X_IDX_THIRD, &ranges[2]);

            range3 = ranges[2] - 50;

            if((rx_data[12] == 0x25) || (rx_data[12] == 0x4A))
            {
                if((range3 <= 1000) && (range3 >= 1))
                {
                    range_3 = range3 * 0.1;
                    strcat(concated, "\x13\x01");
                    strcat(concated, "\x23");
                    memcpy(&concated[3], &range_3, 4);
                    strcat(concated, "\x01\x31");

                    if ((concated[0] == '\x13') && (concated[1] == '\x01') && (concated[7] == '\x01') && (concated[8] == '\x31'))
                        Senser_Transport();

                    for(TX = 0; TX <= 10; TX ++) concated[TX]=0;
                }
                else if(range3 == 8140)
                {
                    strcat(concated, "\x13\x01\x23\x4F\x56\x45\x52\x01\x31");
                    Senser_Transport();
                    for(TX = 0; TX <= 10; TX ++) concated[TX]=0;
                }
            }

    #endif
    #ifdef VL53L0X_FOURTH

            success &= vl53l0x_read_range_single(VL53L0X_IDX_FOURTH, &ranges[3]);

            range4 = ranges[3] - 50;

            if((rx_data[12] == 0x25) || (rx_data[12] == 0x4A))
            {
                if((range4 <= 1000) && (range4 >= 1))
                {
                    range_4 = range4 * 0.1;
                    strcat(concated, "\x13\x01");
                    strcat(concated, "\x24");
                    memcpy(&concated[3], &range_4, 4);
                    strcat(concated, "\x01\x31");

                    if ((concated[0] == '\x13') && (concated[1] == '\x01') && (concated[7] == '\x01') && (concated[8] == '\x31'))
                        Senser_Transport();

                    for(TX = 0; TX <= 10; TX ++) concated[TX]=0;
                }
                else if(range4 == 8140)
                {
                    strcat(concated, "\x13\x01\x24\x4F\x56\x45\x52\x01\x31");
                    Senser_Transport();
                    for(TX = 0; TX <= 10; TX ++) concated[TX]=0;
                }
            }
    #endif
        }
#endif

#ifdef MPU6500_sensor
// TODO MPU9250試試
        if(Mpu_I2c_CheckDataReady())
        {
//            sensor_calibrate.t_elapse = TB0R;
//            TB0CTL |= TBCLR;        // clear ta1 count

            Mpu_I2c_ReadGyro(sensor.gyro_x, sensor.gyro_y, sensor.gyro_z, &sensor.int_status); //讀取陀螺儀數值
            Mpu_I2c_ReadAccel(sensor.accel_x, sensor.accel_y, sensor.accel_z, sensor.temp, &sensor.int_status); //讀取加速度與溫度數值
//            Mpu_I2c_ReadMag(sensor.mag_x, sensor.mag_y, sensor.mag_z, &sensor.mag_st1_status, &sensor.mag_st2_status);

            sensor.convert(&sensor); //數據處理
            sensor_calibrate.gxr = sensor.gyro_x_float;
            sensor_calibrate.gyr = sensor.gyro_y_float;
            sensor_calibrate.gzr = sensor.gyro_z_float;
            sensor_calibrate.axr = sensor.accel_x_float;
            sensor_calibrate.ayr = sensor.accel_y_float;
            sensor_calibrate.azr = sensor.accel_z_float;
            sensor_calibrate.Tempr = sensor.temp_float;
//            sensor_calibrate.mxr = sensor.mag_x_float;
//            sensor_calibrate.myr = sensor.mag_y_float;
//            sensor_calibrate.mzr = sensor.mag_z_float;
            PreProcess(&sensor_calibrate);//運算處理
//            UpdateGDFilter_MARG(&sensor_calibrate);

            if((rx_data[12] == 0x25) || (rx_data[12] == 0x4A))
            {
                strcat(MPU6500_concated, "\x13\x01");
                strcat(MPU6500_concated, "\x31"); memcpy(&MPU6500_concated[3], &sensor_calibrate.gx, 4);
                strcat(MPU6500_concated, "\x32"); memcpy(&MPU6500_concated[8], &sensor_calibrate.gy, 4);
                strcat(MPU6500_concated, "\x33"); memcpy(&MPU6500_concated[13], &sensor_calibrate.gz, 4);
                strcat(MPU6500_concated, "\x34"); memcpy(&MPU6500_concated[18], &sensor_calibrate.ax, 4);
                strcat(MPU6500_concated, "\x35"); memcpy(&MPU6500_concated[23], &sensor_calibrate.ay, 4);
                strcat(MPU6500_concated, "\x36"); memcpy(&MPU6500_concated[28], &sensor_calibrate.az, 4);
                strcat(MPU6500_concated, "\x37"); memcpy(&MPU6500_concated[33], &sensor_calibrate.Temp, 4);
                strcat(MPU6500_concated, "\x01\x31");

                if ((MPU6500_concated[0] == '\x13') && (MPU6500_concated[1] == '\x01') && (MPU6500_concated[37] == '\x01') && (MPU6500_concated[38] == '\x31'));
                {
                    __delay_cycles(10000);  //緩衝
                    MPU6500_Senser_Transport();
                }

                for(TX = 0; TX <= 40; TX ++) MPU6500_concated[TX]=0;
            }
//            roll = getRoll(sensor_calibrate.SEq1, sensor_calibrate.SEq2, sensor_calibrate.SEq3, sensor_calibrate.SEq4);
//            pitch = getPitch(sensor_calibrate.SEq1, sensor_calibrate.SEq2, sensor_calibrate.SEq3, sensor_calibrate.SEq4);
//            yaw = getYaw(sensor_calibrate.SEq1, sensor_calibrate.SEq2, sensor_calibrate.SEq3, sensor_calibrate.SEq4);
        }
#endif

#ifdef ADC_Battery_Detection
// TODO 記得要加電量偵測
        if(Battery_Detection == 50)//固定時間傳送電量
        {
            uint16_t result = ADC_sample(); // 進行一次ADC量測
            //printf("%u\n",result);
            float voltage = ((float)result * 3.3) / 4096; // 計算電壓
            float roundedNum = roundToOneDecimal(voltage); //小數四捨五入
            char level = battery_level(roundedNum); // 計算電量百分比

            if((rx_data[12] == 0x25) || (rx_data[12] == 0x4A))
            {
                if(((level <= 100) && (level >= 1)) || level == 0)
                {
                    if(level == 0)
                    {
                        memcpy(&concated[0], "\x13\x01\x41\x00\x00\x00\x00\x01\x31", 9);
                        Senser_Transport();

                        for(TX = 0; TX <= 10; TX ++) concated[TX]=0;
                    }
                    else
                    {
                        float float_level = level * 1.0001; // char -> float
                        strcat(concated, "\x13\x01");
                        strcat(concated, "\x41");
                        memcpy(&concated[3], &float_level, 4);
                        strcat(concated, "\x01\x31");

                        if ((concated[0] == '\x13') && (concated[1] == '\x01') && (concated[7] == '\x01') && (concated[8] == '\x31'))
                            Senser_Transport();

                        for(TX = 0; TX <= 10; TX ++) concated[TX]=0;
                    }
                }
            }
            Battery_Detection = 0;
        }
        Battery_Detection++;
#endif
    }
}

#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch(__even_in_range(UCA1IV,4))
    {
        case 0:break;                           // Vector 0 - no interrupt
        case 2:                                 // Vector 2 - RXIFG
            while (!(UCA1IFG&UCTXIFG));         // USCI_A0 TX buffer ready?
            rx_data[index++] = UCA1RXBUF;
        break;
        case 4:

        break;                                  // Vector 4 - TXIFG
        default: break;
    }
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void)
{
    __enable_interrupt();            //在中斷中開啟總中斷，代表允許中斷巢狀

    UART_Mobile_terminal();
    UART_Computer_terminal();
    Car_Warning_Distance();
    Steering_Gear_Control();
    Car_Speed();
    Check_connect();                 //確認數據接收

//    if((rx_data[12] == 37) || (rx_data[12] == 0x4A))
//    {
//        index = 0;
//    }
    index = 0;
}

void Car_Warning_Distance(void)
{
//    volatile
    uint16_t angle = 10;
//=============================自動煞車=================================
    if((range1 < WarningDistance) && (x == 0.0)) //前
    {
//        angle = 10;
        __delay_cycles(100);//可能需要調整秒數
        TA0CCR1 = PWM_PULSE_WIDE(0);
        TA0CCR2 = PWM_PULSE_WIDE(0);
        TA0CCR3 = PWM_PULSE_WIDE(0);
        TA0CCR4 = PWM_PULSE_WIDE(0);
        __delay_cycles(100);
        TA2CCR2 = Brakes_servo_lut[angle];

        Auto_STOP_Forward = true;
    }
    if((range2 < WarningDistance) && (x == 0.0)) //後
    {
//        angle = 10;
        __delay_cycles(100);//可能需要調整秒數
        TA0CCR1 = PWM_PULSE_WIDE(0);
        TA0CCR2 = PWM_PULSE_WIDE(0);
        TA0CCR3 = PWM_PULSE_WIDE(0);
        TA0CCR4 = PWM_PULSE_WIDE(0);
        __delay_cycles(100);
        TA2CCR2 = Brakes_servo_lut[angle];

        Auto_STOP_Back = true;
    }
//=============================前進煞車=================================
    if((range1 <= WarningDistance) && (x <= 100) && (x >= 1))
    {
//        angle = 10;
        __delay_cycles(100);//可能需要調整秒數
        TA0CCR1 = PWM_PULSE_WIDE(0);
        TA0CCR2 = PWM_PULSE_WIDE(0);
        TA0CCR3 = PWM_PULSE_WIDE(0);
        TA0CCR4 = PWM_PULSE_WIDE(0);
        __delay_cycles(100);
        TA2CCR2 = Brakes_servo_lut[angle];

        STOP_Forward = true;
    }
//=============================後退煞車=================================
    if((range2 <= WarningDistance) && (x >= -100) && (x <= -1))
    {
//        angle = 10;
        __delay_cycles(100);
        TA0CCR1 = PWM_PULSE_WIDE(0);
        TA0CCR2 = PWM_PULSE_WIDE(0);
        TA0CCR3 = PWM_PULSE_WIDE(0);
        TA0CCR4 = PWM_PULSE_WIDE(0);
        __delay_cycles(100);
        TA2CCR2 = Brakes_servo_lut[angle];

        STOP_Back = true;
    }
//=============================手動煞車=================================
    //TODO 需要寫個手動煞車單獨判斷
    if(STOP == true)
    {
//        angle = 10;
        __delay_cycles(100);
        TA0CCR1 = PWM_PULSE_WIDE(0);
        TA0CCR2 = PWM_PULSE_WIDE(0);
        TA0CCR3 = PWM_PULSE_WIDE(0);
        TA0CCR4 = PWM_PULSE_WIDE(0);
        __delay_cycles(100);
        TA2CCR2 = Brakes_servo_lut[angle];

        STOP = false;
        Manual_STOP = true;
    }
//=============================左轉禁止=================================
    if((range3 <= WarningDistance) && (y >= -179) && (y <= -1))
    {
        TA1CCR1 = servo_lut[90];
        __delay_cycles(100);

        STOP_Left = true;
    }
//=============================右轉禁止=================================
    if((range4 <= WarningDistance) && (y <= 179) && (y >= 1))
    {
        TA1CCR1 = servo_lut[90];
        __delay_cycles(100);

        STOP_Right = true;
    }
}

void Car_Speed(void)
{
//------------------------------煞車解除--------------------------------
    uint16_t angle = 60;

    if((!(range1 <= WarningDistance) && (x <= 100) && (x >= 1)) || (OPEN == true))
    {
//        angle = 60;
        TA2CCR2 = Brakes_servo_lut[angle];

        OPEN = false;
    }

    if((!(range2 <= WarningDistance) && (x >= -100) && (x <= -1)) || (OPEN == true))
    {
//        angle = 60;
        TA2CCR2 = Brakes_servo_lut[angle];

        OPEN = false;
    }

//------------------------------搖桿前進--------------------------------
    if(!(range1 <= WarningDistance))
    {
        if((x <= 100) && (x >= 1) && (MaxSpeed <= 100) && (MaxSpeed >= 1))
        {
            Speed = MaxSpeed + x;
            Forward = Speed / 2;
            __delay_cycles(100);
            TA0CCR1 = PWM_PULSE_WIDE(Forward);
            TA0CCR2 = PWM_PULSE_WIDE(0);
            TA0CCR3 = PWM_PULSE_WIDE(Forward);
            TA0CCR4 = PWM_PULSE_WIDE(0);
            __delay_cycles(100);
        }
    }
//------------------------------搖桿後退--------------------------------
    if(!(range2 <= WarningDistance))
    {
        if((x >= -100) && (x <= -1) && (MaxSpeed <= 100) && (MaxSpeed >= 1))
        {
            Speed = x + 100; //可以試試絕對值 fabs(x);用於浮點數 abs(x);
            Speed = 100 - Speed;
            Speed = Speed + MaxSpeed;
            Reverse = Speed / 2;
            __delay_cycles(100);
            TA0CCR1 = PWM_PULSE_WIDE(0);
            TA0CCR2 = PWM_PULSE_WIDE(Reverse);
            TA0CCR3 = PWM_PULSE_WIDE(0);
            TA0CCR4 = PWM_PULSE_WIDE(Reverse);
            __delay_cycles(100);
        }
    }
//------------------------------搖桿置中--------------------------------
    if(x == 0.0)
    {
        TA0CCR1 = PWM_PULSE_WIDE(0);
        TA0CCR2 = PWM_PULSE_WIDE(0);
        TA0CCR3 = PWM_PULSE_WIDE(0);
        TA0CCR4 = PWM_PULSE_WIDE(0);
    }
}

void Steering_Gear_Control(void)
{
//---------------前進後退左轉-----------------
    if(!(range3 <= WarningDistance))
    {
        if((y >= -89) && (y <= -1))
        {
            negative = y + 90;
            TA1CCR1 = servo_lut[negative];
            __delay_cycles(100);
        }
        else if((y >= -179) && (y <= -91))
        {
            Angle = y + 180;
            negative = 90 - Angle;
            TA1CCR1 = servo_lut[negative];
            __delay_cycles(100);
        }
        else if(y == -90)
        {
            TA1CCR1 = servo_lut[0];
            __delay_cycles(100);
        }
    }
//---------------前進後退右轉-----------------
    if(!(range4 <= WarningDistance))
    {
        if((y <= 89) && (y >= 1))
        {
            Positive = y + 90;
            TA1CCR1 = servo_lut[Positive];
            __delay_cycles(100);
        }
        else if((y <= 179) && (y >= 91))
        {
            Angle = y - 90;
            Positive = 180 - Angle ;
            TA1CCR1 = servo_lut[Positive];
            __delay_cycles(100);
        }
        else if(y == 90)
        {
            TA1CCR1 = servo_lut[180];
            __delay_cycles(100);
        }
    }
//------------------回正--------------------
    if((y == 0) || (y == -180) || (y == 180))
    {
        TA1CCR1 = servo_lut[90];
        __delay_cycles(100);
    }
}

void Check_connect(void)
{
    if((rx_data[12] == 0x25) || (rx_data[12] == 0x4A)) P6OUT ^= BIT3;

    else P6OUT &=~ BIT3;

//    if(rx_data[2] + rx_data[3] + rx_data[4] + rx_data[5] + rx_data[6] + rx_data[7] + rx_data[8] == 0x20A)  { P6OUT |= BIT3; }
//
//    else if(rx_data[2] + rx_data[3] + rx_data[4] + rx_data[5] + rx_data[6] + rx_data[7]  == 0x1BA) { P6OUT &=~ BIT3; }
}

void Stop_Event(void)
{
    if(STOP_Forward == true)
    {
        memcpy(&concated[0], "\x13\x01\x11\x53\x54\x4F\x50\x01\x31", 9);
        Senser_Transport();

        for(TX = 0; TX <= 10; TX ++) concated[TX]=0;

        STOP_Forward = false;
    }
    if(STOP_Back == true)
    {
        memcpy(&concated[0], "\x13\x01\x12\x53\x54\x4F\x50\x01\x31", 9);
        Senser_Transport();

        for(TX = 0; TX <= 10; TX ++) concated[TX]=0;

        STOP_Back = false;
    }
    if(STOP_Left == true)
    {
        memcpy(&concated[0], "\x13\x01\x13\x53\x54\x4F\x50\x01\x31", 9);
        Senser_Transport();

        for(TX = 0; TX <= 10; TX ++) concated[TX]=0;

        STOP_Left = false;
    }
    if(STOP_Right == true)
    {
        memcpy(&concated[0], "\x13\x01\x14\x53\x54\x4F\x50\x01\x31", 9);
        Senser_Transport();

        for(TX = 0; TX <= 10; TX ++) concated[TX]=0;

        STOP_Right = false;
    }
    if(Manual_STOP == true)
    {
        memcpy(&concated[0], "\x13\x01\x15\x53\x54\x4F\x50\x01\x31", 9);
        Senser_Transport();

        for(TX = 0; TX <= 10; TX ++) concated[TX]=0;

        Manual_STOP = false;
    }
    //TODO 自動動煞車事件
    if(Auto_STOP_Forward == true)
    {
        memcpy(&concated[0], "\x13\x01\x16\x53\x54\x4F\x50\x01\x31", 9);
        Senser_Transport();

        for(TX = 0; TX <= 10; TX ++) concated[TX]=0;

        Auto_STOP_Forward = false;
    }
    if(Auto_STOP_Back == true)
    {
        memcpy(&concated[0], "\x13\x01\x17\x53\x54\x4F\x50\x01\x31", 9);
        Senser_Transport();

        for(TX = 0; TX <= 10; TX ++) concated[TX]=0;

        Auto_STOP_Back = false;
    }
}

void Senser_Transport(void)
{
    while(!(UCA1IFG&UCTXIFG));
    {
        if(rx_data[7] == 0x30)
        {
            __delay_cycles(1000);
            AT_count = sizeof(CIPSEND_0) - 1;
            for(AT_string = 0; AT_string < AT_count; AT_string++)
            {
                UCA1TXBUF = CIPSEND_0[AT_string];
                __delay_cycles(100);
            }
            Transport_String();
        }
        else if(rx_data[7] == 0x31)
        {
            __delay_cycles(1000);
            AT_count = sizeof(CIPSEND_1) - 1;
            for(AT_string = 0; AT_string < AT_count; AT_string++)
            {
                UCA1TXBUF = CIPSEND_1[AT_string];
                __delay_cycles(100);
            }
            Transport_String();
        }
        else if(rx_data[7] == 0x32)
        {
            __delay_cycles(1000);
            AT_count = sizeof(CIPSEND_2) - 1;
            for(AT_string = 0; AT_string < AT_count; AT_string++)
            {
                UCA1TXBUF = CIPSEND_2[AT_string];
                __delay_cycles(100);
            }
            Transport_String();
        }
        else if(rx_data[7] == 0x33)
        {
            __delay_cycles(1000);
            AT_count = sizeof(CIPSEND_3) - 1;
            for(AT_string = 0; AT_string < AT_count; AT_string++)
            {
                UCA1TXBUF = CIPSEND_3[AT_string];
                __delay_cycles(100);
            }
            Transport_String();
        }
        else if(rx_data[7] == 0x34)
        {
            __delay_cycles(1000);
            AT_count = sizeof(CIPSEND_4) - 1;
            for(AT_string = 0; AT_string < AT_count; AT_string++)
            {
                UCA1TXBUF = CIPSEND_4[AT_string];
                __delay_cycles(100);
            }
            Transport_String();
        }
        return;
    }
}

void Transport_String(void)
{
    __delay_cycles(50000);
    AT_count = sizeof(concated) - 1;
    for(AT_string = 0; AT_string < AT_count; AT_string++)
    {
        UCA1TXBUF = concated[AT_string];
        __delay_cycles(100);
    }
    //AT_count = 0;
}

//void VL53L0X_Senser_Transport(void)
//{
//    while(!(UCA1IFG&UCTXIFG));
//    {
//        if(rx_data[7] == 0x30)
//        {
//            __delay_cycles(1000);
//            AT_count = sizeof(VL53L0X_CIPSEND_0) - 1;
//            for(AT_string = 0; AT_string < AT_count; AT_string++)
//            {
//                UCA1TXBUF = VL53L0X_CIPSEND_0[AT_string];
//                __delay_cycles(100);
//            }
//            VL53L0X_Transport_String();
//        }
//        else if(rx_data[7] == 0x31)
//        {
//            __delay_cycles(1000);
//            AT_count = sizeof(VL53L0X_CIPSEND_1) - 1;
//            for(AT_string = 0; AT_string < AT_count; AT_string++)
//            {
//                UCA1TXBUF = VL53L0X_CIPSEND_1[AT_string];
//                __delay_cycles(100);
//            }
//            VL53L0X_Transport_String();
//        }
//        else if(rx_data[7] == 0x32)
//        {
//            __delay_cycles(1000);
//            AT_count = sizeof(VL53L0X_CIPSEND_2) - 1;
//            for(AT_string = 0; AT_string < AT_count; AT_string++)
//            {
//                UCA1TXBUF = VL53L0X_CIPSEND_2[AT_string];
//                __delay_cycles(100);
//            }
//            VL53L0X_Transport_String();
//        }
//        else if(rx_data[7] == 0x33)
//        {
//            __delay_cycles(1000);
//            AT_count = sizeof(VL53L0X_CIPSEND_3) - 1;
//            for(AT_string = 0; AT_string < AT_count; AT_string++)
//            {
//                UCA1TXBUF = VL53L0X_CIPSEND_3[AT_string];
//                __delay_cycles(100);
//            }
//            VL53L0X_Transport_String();
//        }
//        else if(rx_data[7] == 0x34)
//        {
//            __delay_cycles(1000);
//            AT_count = sizeof(VL53L0X_CIPSEND_4) - 1;
//            for(AT_string = 0; AT_string < AT_count; AT_string++)
//            {
//                UCA1TXBUF = VL53L0X_CIPSEND_4[AT_string];
//                __delay_cycles(100);
//            }
//            VL53L0X_Transport_String();
//        }
//        return;
//    }
//}
//
//void VL53L0X_Transport_String(void)
//{
//    __delay_cycles(50000);
//    AT_count = sizeof(VL53L0X_concated) - 1;
//    for(AT_string = 0; AT_string < AT_count; AT_string++)
//    {
//        UCA1TXBUF = VL53L0X_concated[AT_string];
//        __delay_cycles(100);
//    }
//}

void MPU6500_Senser_Transport(void)
{
    while(!(UCA1IFG&UCTXIFG));
    {
        if(rx_data[7] == 0x30)
        {
            __delay_cycles(1000);
            AT_count = sizeof(MPU6500_CIPSEND_0) - 1;
            for(AT_string = 0; AT_string < AT_count; AT_string++)
            {
                UCA1TXBUF = MPU6500_CIPSEND_0[AT_string];
                __delay_cycles(100);
            }
            MPU6500_Transport_String();
        }
        else if(rx_data[7] == 0x31)
        {
            __delay_cycles(1000);
            AT_count = sizeof(MPU6500_CIPSEND_1) - 1;
            for(AT_string = 0; AT_string < AT_count; AT_string++)
            {
                UCA1TXBUF = MPU6500_CIPSEND_1[AT_string];
                __delay_cycles(100);
            }
            MPU6500_Transport_String();
        }
        else if(rx_data[7] == 0x32)
        {
            __delay_cycles(1000);
            AT_count = sizeof(MPU6500_CIPSEND_2) - 1;
            for(AT_string = 0; AT_string < AT_count; AT_string++)
            {
                UCA1TXBUF = MPU6500_CIPSEND_2[AT_string];
                __delay_cycles(100);
            }
            MPU6500_Transport_String();
        }
        else if(rx_data[7] == 0x33)
        {
            __delay_cycles(1000);
            AT_count = sizeof(MPU6500_CIPSEND_3) - 1;
            for(AT_string = 0; AT_string < AT_count; AT_string++)
            {
                UCA1TXBUF = MPU6500_CIPSEND_3[AT_string];
                __delay_cycles(100);
            }
            MPU6500_Transport_String();
        }
        else if(rx_data[7] == 0x34)
        {
            __delay_cycles(1000);
            AT_count = sizeof(MPU6500_CIPSEND_4) - 1;
            for(AT_string = 0; AT_string < AT_count; AT_string++)
            {
                UCA1TXBUF = MPU6500_CIPSEND_4[AT_string];
                __delay_cycles(100);
            }
            MPU6500_Transport_String();
        }
        return;
    }
}

void MPU6500_Transport_String(void)
{
    __delay_cycles(50000);
    AT_count = sizeof(MPU6500_concated) - 1;
    for(AT_string = 0; AT_string < AT_count; AT_string++)
    {
        UCA1TXBUF = MPU6500_concated[AT_string];
        __delay_cycles(100);
    }
    //AT_count = 0;
}

void UART_Mobile_terminal(void)
{
    if(!(check_header_A() && (check_footer_B() || check_footer_C()))) return; //頭尾碼不符合直接返回,不往下執行
//----------------------------------------------------------------------------------------------------
    unsigned char DataX[4] = { rx_data[21], rx_data[20], rx_data[19], rx_data[18] };
    x = *(float*) & DataX;
    x = x * 100;
    x = floorf(x);

    unsigned char DataY[4] = { rx_data[26], rx_data[25], rx_data[24], rx_data[23] };
    y = *(int*) & DataY;
//----------------------------------------------------------------------------------------------------
    if(rx_data[27] == 0x15)
    {
        MaxSpeed = (uint16_t)rx_data[31];
    }
//----------------------------------------------------------------------------------------------------
    else if(rx_data[27] == 0x16)
    {
        WarningDistance = (uint16_t)rx_data[31];
        WarningDistance = WarningDistance * 10;
    }
//----------------------------------------------------------------------------------------------------
    else if(rx_data[27] == 0x17)
    {
        BrakingDistance = (uint16_t)rx_data[31];
    }
    //TODO 要加上手動煞車
}

void UART_Computer_terminal(void)
{
    if(!check_header_A_1()) return; //頭尾碼不符合直接返回,不往下執行
//----------------------------------------------------------------------------------------------------
    unsigned char DataX[4] = { rx_data[18], rx_data[17], rx_data[16], rx_data[15] };
    x = *(float*) & DataX;
    x = x * 100;
    x = floorf(x);

    unsigned char DataY[4] = { rx_data[23], rx_data[22], rx_data[21], rx_data[20] };
    y = *(int*) & DataY;

    if(rx_data[24] == 0x13)
    {
        MaxSpeed = (uint16_t)rx_data[28];
    }
    if(rx_data[29] == 0x14)
    {
        WarningDistance = (uint16_t)rx_data[33];
        WarningDistance = WarningDistance * 10;
    }
    if(rx_data[34] == 0x15)
    {
        if((rx_data[35] == 0x53) && (rx_data[36] == 0x54) && (rx_data[37] == 0x4F) && (rx_data[38] == 0x50))
            STOP = true;
        else if((rx_data[35] == 0x4F) && (rx_data[36] == 0x50) && (rx_data[37] == 0x45) && (rx_data[38] == 0x4E))
            OPEN = true;
    }
//----------------------------------------------------------------------------------------------------
}

void AT_command(void)
{
    while (!(UCA1IFG&UCTXIFG));
    {
        __delay_cycles(100000);
        AT_count = sizeof(CWMODE) - 1;
        for(AT_string = 0; AT_string < AT_count; AT_string++)
        {
            UCA1TXBUF = CWMODE[AT_string];
            __delay_cycles(100);
        }
        __delay_cycles(100000);
        AT_count = sizeof(CIPMUX) - 1;
        for(AT_string = 0; AT_string < AT_count; AT_string++)
        {
            UCA1TXBUF = CIPMUX[AT_string];
            __delay_cycles(100);
        }
        __delay_cycles(100000);
        AT_count = sizeof(CIPSERVER) - 1;
        for(AT_string = 0; AT_string < AT_count; AT_string++)
        {
            UCA1TXBUF =CIPSERVER[AT_string];
            __delay_cycles(100);
        }

        for(AT_string = 0; AT_string < 5; AT_string++)
        {
            P6OUT ^= BIT3;          //初始化完成指示燈
            __delay_cycles(100000);
            P6OUT &=~ BIT3;
            __delay_cycles(100000);
        }
       /* __delay_cycles(100000);
        AT_count = sizeof(CWSAP) - 1;
        for(AT_string = 0; AT_string < AT_count; AT_string++)
        {
            UCA1TXBUF =CWSAP[AT_string];
            __delay_cycles(100);
        }*/
        return;
    }
}


void Servo_PWM_control(void)
{
    servo_stepval  = ( ((SERVO_MAX - SERVO_MIN) / SERVO_STEPS) );
    servo_stepnow  = SERVO_MIN;

    // Fill up the LUT // 填寫LUT
    for (i = 0; i<SERVO_STEPS; i++)
    {
        servo_stepnow += servo_stepval; //c += a , c = c+a
        servo_lut[i] = servo_stepnow;
    }
}

void Servo_Init(void)
{
    P2DIR   |= BIT0;                // P2.0 = output
    P2SEL   |= BIT0;                // P2.0 = TA1 output
    TA1CCTL1 = OUTMOD_7;            // TACCR1 reset/set      // 重置/重設
    TA1CCR0  = PWM_Period-1;        // PWM Period            // 週期
    TA1CCR1  = PWM_Duty;            // TACCR1 PWM Duty Cycle // 占空比
    TA1CTL   = TASSEL_2 + MC_1 + TACLR;     // SMCLK, upmode
}

void Brakes_Servo_PWM_control(void)
{
    Brakes_servo_stepval  = ( ((Brakes_SERVO_MAX - Brakes_SERVO_MIN) / Brakes_SERVO_STEPS) );
    Brakes_servo_stepnow  = Brakes_SERVO_MIN;

    // Fill up the LUT // 填寫LUT
    for (Brakes_i = 0; Brakes_i < Brakes_SERVO_STEPS; Brakes_i++)
    {
        Brakes_servo_stepnow += Brakes_servo_stepval; //c += a , c = c+a
        Brakes_servo_lut[Brakes_i] = Brakes_servo_stepnow;
    }
}

void Brakes_Servo_Init(void)
{
 //   P2DIR   |= BIT4;                // P2.0 = output
 //   P2SEL   |= BIT4;                // P2.0 = TA1 output
    P2DIR   |= BIT5;                // P2.0 = output
    P2SEL   |= BIT5;                // P2.0 = TA1 output
 //   TA2CCTL1= OUTMOD_7;             // TACCR1 reset/set      // 重置/重設
    TA2CCR0  = Brakes_PWM_Period-1;        // PWM Period            // 週期
 //   TA2CCR1  = Brakes_PWM_Duty;            // TACCR1 PWM Duty Cycle // 占空比
    TA2CCTL2 = OUTMOD_7;            // TACCR1 reset/set      // 重置/重設
    TA2CCR2  = Brakes_PWM_Duty;            // TACCR1 PWM Duty Cycle // 占空比
    TA2CTL   = TASSEL_2 + MC_1 + TACLR;     // SMCLK, upmode
}

void Timer_Interrupt_Init(void)
{
//======================================中斷======================================
    TB0CCR0 = 3276;                 //置入要比較的數值，約為3276/32768 ≈ 0.09997s一次中斷 6552 9828 8190 1638
    TB0CCTL0 = CCIE;                //捕獲比較器1開啟 CCIFG 位中斷
    TB0CTL |= TASSEL_1+MC_1+TACLR;  //選擇 SCLK32.768KHZ 作為時鐘
//===============================================================================
}

void UATR_Init(void)
{
    P4SEL |= BIT4+BIT5;               // P3.3,4 = USCI_A0 TXD/RXD
    //P2DIR &=~BIT1;
    //P2REN |= BIT1;
    //P2OUT |= BIT1;
    UCA1CTL1 |= UCSWRST;              // Put state machine in reset
    UCA1CTL1 |= UCSSEL_2;             // SMCLK
    UCA1BR0 = 9;                      // 1MHz 115200 (see User's Guide)
    UCA1BR1 = 0;                      // 1MHz 115200
    UCA1MCTL |= UCBRS_1 + UCBRF_0;    // Modulation UCBRSx=1, UCBRFx=0
    UCA1CTL1 &= ~UCSWRST;             // Initialize USCI state machine
    UCA1IE |= UCRXIE;
}

void TimerInit(void)
{
    P1DIR |= BIT2; // OPEN TA0CCTL1
    P1SEL |= BIT2; // BIN1
    P1DIR |= BIT3; // OPEN TA0CCTL2
    P1SEL |= BIT3; // BIN2
    TA0CCTL0 = OUTMOD_5; // CCR0 reset mode
    TA0CCR0 = PWM_PERIOD;
    TA0CCTL1 = OUTMOD_6; // p1.2 CCR1 toggle/reset mode FIN
    TA0CCR1 = PWM_PULSE_WIDE(0); //0%
    TA0CCTL2 = OUTMOD_6; // p1.3 CCR2 toggle/reset mode RIN
    TA0CCR2 = PWM_PULSE_WIDE(0); //0%
    TA0CTL = TASSEL_2 + MC_1 + TACLR; // SMCLK, up mode, clear TAR
//==================================================================
    P1DIR |= BIT4; // OPEN TA0CCTL3
    P1SEL |= BIT4; // AIN1
    P1DIR |= BIT5; // OPEN TA0CCTL4
    P1SEL |= BIT5; // AIN2
    TA0CCTL3 = OUTMOD_6; // p1.4 CCR3 toggle/reset mode
    TA0CCR3 = PWM_PULSE_WIDE(0); //0%
    TA0CCTL4 = OUTMOD_6; // p1.5 CCR4 toggle/reset mode
    TA0CCR4 = PWM_PULSE_WIDE(0); //0%
    TA0CTL = TASSEL_2 + MC_1 + TACLR; // SMCLK, up mode, clear TAR
//==================================================================
}

