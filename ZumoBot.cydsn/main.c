

#include <project.h>
#include <stdio.h>
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "I2C_made.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "IR.h"
#include "Ambient.h"
#include "Beep.h"

int rread(void);




///reflectance//

float p(int blackv, int curv, int whitev);
void tank_turnR(uint8 l_speed, uint8 r_speed, uint32 delay); //necessary for sumo
void tank_turnL(uint8 l_speed, uint8 r_speed, uint32 delay); //necessary for sumo
int main()
{
    struct sensors_ ref;
    struct sensors_ dig;
    CyGlobalIntEnable; 
    UART_1_Start();
  
    sensor_isr_StartEx(sensor_isr_handler);
    
    reflectance_start();
    
    IR_led_Write(1);
   
    float errorL = 0.0;
    float errorR = 0.0;
    float lasterrorl = 0.0;
    float lasterrorr = 0.0;
    float whitel1 = 5100.0;
    float whiter1 = 5200.0;
    float mspeed = 255.0;
    float kP = 276.4063;
    float kD = 0.0001;
    float dl = 0.0;
    float dr = 0.0;
    float motorspeedleft = 0.0;
    float motorspeedright = 0.0;
    int flag = 0;
    int count;
    int IR_val = 0;
    
    

    
    do //loop that brings the robot up to the starting line
    {
        
    reflectance_read(&ref);
    reflectance_digital(&dig);
    motor_start();
    motor_forward(100,15);
    
    }while(dig.r3 == 1 && dig.l3 == 1); //breaks when corner sensors see black
    
    motor_stop(0,0);
    
    do //loop waiting for IR input. Exits the loop upon receiving the signal
    
    {
    IR_val = get_IR();
    }while(!IR_val);
    
   
    motor_start();
    
    //loop for navigating the track. Starts when IR signal is received
    while(count < 3){
        
        
        CyDelayUs(1);
        reflectance_read(&ref);
        printf("%d %d %d %d \r\n", ref.l3, ref.l1, ref.r1, ref.r3);       //print out each period of reflectance sensors
        reflectance_digital(&dig);      //print out 0 or 1 according to results of reflectance period
        printf("%d %d %d %d \r\n", dig.l3, dig.l1, dig.r1, dig.r3);        //print out 0 or 1 according to results of reflectance period
        
        
        errorL = 23999 - ref.l1; //calculates error from middle left sensor
        errorR = 23999 - ref.r1; //calculatees error from middle right sensor
        dl = errorR - lasterrorl; //calculate the drivative for turning left (Used for KD)
        dr = errorL- lasterrorr; //calculate the drivative for turning right (Used for KD)
        
        lasterrorl = errorR;
        lasterrorr = errorL;
        
        motorspeedleft = kP * p(23999, ref.r1, whiter1) + kD * dl; //makes the calculation for how much to break the motor
        motorspeedright = kP * p(23999, ref.l1, whitel1) + kD * dr;
    
        
  
        
        
        
        
            
        if(dig.l3 == 0 && dig.r3 ==0 && flag == 0)// adds to the "count", when count = 3, the robot stops
        
        {
        flag = 1;
        count ++;
        }
         
        if(dig.l3 == 1 && dig.r3 == 1 && flag == 1)
        
        {
        flag = 0;
        }
        
        if(ref.r1 > 8000 && ref.l1 > 8000) //as long as the 2 middle sensors reading are above 8000, uses PD to steer, keeping in on the line
        
        {
        motor_turn( (uint8_t)(mspeed - motorspeedleft ), (uint8_t)(mspeed -  motorspeedright ), 1); 
        }
        
        
        else if (ref.r3 >= 22999 ) //makes the robot turn right sharply when the right corner sensor sees black
        
        {
         motor_turn(mspeed,0,1);
        }
        
        else if (ref.l3 >=17700 ) //makes the robot turn left sharply when the left corner sensor sees black
        
        {
                motor_turn(0,mspeed,1);
        }
       
    }
      
    motor_stop(0,0);   
        
    
        
    //Sumo
    /*
    CyDelayUs(1);
        reflectance_read(&ref);
        printf("%d %d %d %d \r\n", ref.l3, ref.l1, ref.r1, ref.r3);       //print out each period of reflectance sensors
        reflectance_digital(&dig);      //print out 0 or 1 according to results of reflectance period
        printf("%d %d %d %d \r\n", dig.l3, dig.l1, dig.r1, dig.r3);        //print out 0 or 1 according to results of reflectance period
    
        
    if(ref.l1 > 19000 && ref.r1 > 19000)  //moves the robot into the ring from the starting line
    {
    motor_forward(mspeed, 600);
    }
        
    else if (dig.r1 == 1 && dig.l1 == 1 && dig.r3 == 1  && dig.l3 == 1) //makes the robot go forward at full speed while all sensors see white
    {
    
    motor_forward(mspeed,1);
    }
    
    else if(ref.l3 > 10000) //makes the robot go back and turn right if the left corner sensor sees black
    {
        
    motor_backward(mspeed, 225);
    tank_turnR(mspeed, mspeed, 400);
    }
    
    else if(ref.r3 > 10000) //makes the robot go back and turn right if the left corner sensor sees black
    {
    motor_backward(mspeed, 225);
    tank_turnL(mspeed, mspeed, 400);
    }
    
    */
    
//}
}

float p(int blackv, int curv, int whitev){  //calculate the "p" for the PD controls

    float p = 0.0;
    
     
        p = ((float)blackv - curv)/((float)blackv - whitev);
        printf("%f \n", p);
        return p;
       
    
     
}





 



    
    //pilot mode//
/*int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
    
    int start = 0;
    
    
    
    
         
        
    do{
   
        printf("Enter number(1=forward, 2=tankleft, 3=tankright, 4=back): \n");
         scanf("%d", &start);
    
        
        
        if(start == 1){
    
        motor_start();              // motor start
        motor_forward(500,200);     // moving forward
        motor_stop();
        start = 0;
        }
    
        if(start == 2){
        motor_start(); 
        tank_turnR(200,200,100);     // turn right
    
        motor_stop();
        start = 0;
        }
    
        if(start == 3){
    
        motor_start();
        tank_turnL(200,200,100);     // turn left
        motor_stop();
        start = 0;
        }
    
    if(start == 4){
    
    motor_start(); 
    motor_backward(500,200);     // movinb backward
    motor_stop();
    start = 0;
    }
    
    if(start == 9) {
    motor_backward(0,1000);     // movinb backward
    motor_stop();
    start = 0;
    }
    
    if(start == 8){
    
    motor_start();              
    motor_forward(0,1000);     
    motor_stop();
    start = 0;
    }
    
        else{
        motor_stop();               // motor stop
        start = 0;
        }
    
    
    }while(start == 0);
    

    
    for(;;)
    {

    }
}*/

void tank_turnR(uint8 l_speed, uint8 r_speed, uint32 delay)
{
    MotorDirLeft_Write(1);      // set LeftMotor backward mode
    MotorDirRight_Write(0);
    PWM_WriteCompare1(l_speed); 
    PWM_WriteCompare2(r_speed); 
    CyDelay(delay);
}

void tank_turnL(uint8 l_speed, uint8 r_speed, uint32 delay)
{
    MotorDirLeft_Write(0);      // set LeftMotor backward mode
    MotorDirRight_Write(1);
    PWM_WriteCompare1(l_speed); 
    PWM_WriteCompare2(r_speed); 
    CyDelay(delay);
}
//
    




#if 0
int rread(void)
{
    SC0_SetDriveMode(PIN_DM_STRONG);
    SC0_Write(1);
    CyDelayUs(10);
    SC0_SetDriveMode(PIN_DM_DIG_HIZ);
    Timer_1_Start();
    uint16_t start = Timer_1_ReadCounter();
    uint16_t end = 0;
    while(!(Timer_1_ReadStatusRegister() & Timer_1_STATUS_TC)) {
        if(SC0_Read() == 0 && end == 0) {
            end = Timer_1_ReadCounter();
        }
    }
    Timer_1_Stop();
    
    return (start - end);
}
#endif

/* Don't remove the functions below */
int _write(int file, char *ptr, int len)
{
    (void)file; /* Parameter is not used, suppress unused argument warning */
	int n;
	for(n = 0; n < len; n++) {
        if(*ptr == '\n') UART_1_PutChar('\r');
		UART_1_PutChar(*ptr++);
	}
	return len;
}

int _read (int file, char *ptr, int count)
{
    int chs = 0;
    char ch;
 
    (void)file; /* Parameter is not used, suppress unused argument warning */
    while(count > 0) {
        ch = UART_1_GetChar();
        if(ch != 0) {
            UART_1_PutChar(ch);
            chs++;
            if(ch == '\r') {
                ch = '\n';
                UART_1_PutChar(ch);
            }
            *ptr++ = ch;
            count--;
            if(ch == '\n') break;
        }
    }
    return chs;
}

/* [] END OF FILE */
