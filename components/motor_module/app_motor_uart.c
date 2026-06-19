#include "app_motor_uart.h"

#define RXBUFF_LEN 256

uint8_t send_buff[50];

float g_Speed[4];
int Encoder_Offset[4];
int Encoder_Now[4];

uint8_t g_recv_flag; 
uint8_t g_recv_buff[RXBUFF_LEN];
uint8_t g_recv_buff_deal[RXBUFF_LEN];

//////////********************���Ͳ���********************///////////
//////////******************发送部分*****************///////////

//���͵������	Transmitter motor type
void send_motor_type(motor_type_t data)
{
	sprintf((char*)send_buff,"$mtype:%d#",data);
	Send_Motor_ArrayU8(send_buff, strlen((char*)send_buff));
	
}

//���͵������	Send motor dead zone
void send_motor_deadzone(uint16_t data)
{
	sprintf((char*)send_buff,"$deadzone:%d#",data);
	Send_Motor_ArrayU8(send_buff, strlen((char*)send_buff));
}

//���͵���Ż�����	Send motor magnetic ring pulse
void send_pulse_line(uint16_t data)
{
	sprintf((char*)send_buff,"$mline:%d#",data);
	Send_Motor_ArrayU8(send_buff, strlen((char*)send_buff));
}

//���͵�����ٱ�	Transmitting motor reduction ratio
void send_pulse_phase(uint16_t data)
{
	sprintf((char*)send_buff,"$mphase:%d#",data);
	Send_Motor_ArrayU8(send_buff, strlen((char*)send_buff));
}

//��������ֱ��	Send wheel diameter
void send_wheel_diameter(float data)
{
	sprintf((char*)send_buff,"$wdiameter:%.3f#",data);
	Send_Motor_ArrayU8(send_buff, strlen((char*)send_buff));
}

//����PID����	Send PID parameters
void send_motor_PID(float P,float I,float D)
{
	sprintf((char*)send_buff,"$mpid:%.3f,%.3f,%.3f#",P,I,D);
	Send_Motor_ArrayU8(send_buff, strlen((char*)send_buff));
}

//��Ҫ�������ݵĿ���	Switch that needs to receive data
void send_upload_data(bool ALLEncoder_Switch,bool TenEncoder_Switch,bool Speed_Switch)
{
	sprintf((char*)send_buff,"$upload:%d,%d,%d#",ALLEncoder_Switch,TenEncoder_Switch,Speed_Switch);
	Send_Motor_ArrayU8(send_buff, strlen((char*)send_buff));
}

//�����ٶ�	Controlling Speed
void Contrl_Speed(int16_t M1_speed,int16_t M2_speed,int16_t M3_speed,int16_t M4_speed)
{
	sprintf((char*)send_buff,"$spd:%d,%d,%d,%d#",M1_speed,M2_speed,M3_speed,M4_speed);
	Send_Motor_ArrayU8(send_buff, strlen((char*)send_buff));
}


//����pwm	Control PWM
void Contrl_Pwm(int16_t M1_pwm,int16_t M2_pwm,int16_t M3_pwm,int16_t M4_pwm)
{
	sprintf((char*)send_buff,"$pwm:%d,%d,%d,%d#",M1_pwm,M2_pwm,M3_pwm,M4_pwm);
	Send_Motor_ArrayU8(send_buff, strlen((char*)send_buff));
}


//////////********************���ղ���********************///////////
//////////*****************Receiving part****************///////////

//����������������ַ���(ָ������)  ԭʼ�ַ���  �ָ�����
//Incoming parameters: reserved string (pointer array) original string separator
void splitString(char* mystrArray[],char *str, const char *delimiter) 
{
    char *token = strtok(str, delimiter); //���ǵ�һ�ηָ�,��һ���ַ�ֵ	This is the first split, the first character value
		mystrArray[0] = token; //������һ�ηָ���ַ�		Keep the first split character
    int i =1;
	
    while (token != NULL) 
    {
        token = strtok(NULL, delimiter);
        mystrArray[i] = token;
        i++;
    }
}

//����������巢�͹��������ݣ�����ͨѶЭ��������򱣴�����
//Check the data sent from the driver board, and save the data that meets the communication protocol
void Deal_Control_Rxtemp(uint8_t rxtemp)
{
	static u8 step = 0;
	static u8 start_flag = 0;

	if(rxtemp == '$' && 	start_flag == 0)
	{
		start_flag = 1;
		memset(g_recv_buff,0,RXBUFF_LEN);//�������	Clear data
	}
	
	else if(start_flag == 1)
	{
			if(rxtemp == '#')
			{
				start_flag = 0;
				step = 0;
				g_recv_flag = 1;
				memcpy(g_recv_buff_deal,g_recv_buff,RXBUFF_LEN); //ֻ����ȷ�Żḳֵ	Only correct ones will be assigned
			}
			else
			{
				if(step >= RXBUFF_LEN-1)
				{
					start_flag = 0;
					step = 0;
					memset(g_recv_buff,0,RXBUFF_LEN);//��ս�������	Clear received data
				}
				else
				{
					g_recv_buff[step] = rxtemp;
					step++;
				}
			}
	}
	
}

//屣浽ݽиʽȻ׼ӡ
//将从驱动板保存的数据进行格式化，并准备打印
void Deal_data_real(void)
{
	 static uint8_t data[RXBUFF_LEN];
   uint8_t  length = 0;
	
	//����ı�����	Overall encoder
	 如果 ((strncmp("MAll",(字符*)g_recv_buff_deal,4)==0))
    {
        长度 = strlen((char*)g_recv_buff_deal)-5;
        for (uint8_t i = 0; i < length; i++)
        {
i] = g_recv_buff_deal[i+5]; //删除冒号
        }  
				数据[长度] = '';	

					
				char* strArray[10];;//ָ ȸݷָŶ  char 1ֽ   char* 4ֽ	 指针数组 长度由分割数量决定 char 1字节 char* 4字节
				char mystr_temp[4][10] = {'\0'}; 
				splitString(strArray,(char*)data, ", ");//Զи	按逗号分割
				for (int i = 0; i < 4; i++)
				{
						strcpy(mystr_temp[i],strArray[i]);
						Encoder_Now[i] = atoi(mystr_temp[i]);
				}
				
		}
		//10msʵʱ	10ms实时编码器数据
		否则 如果	((strncmp("MTEP",(字符指针)g_recv_buff_deal,4)==0))
    {
        长度 = strlen((char*)g_recv_buff_deal)-5;
        for (uint8_t i = 0; i < length; i++)
        {
            数据[i] = g_recv_buff_deal[i+5]; //删除冒号
        }  
				数据[长度] = '\0';		

				char* strArray[10];;//ָ ȸݷָŶ  char 1ֽ   char* 4ֽ		指针数组 长度由分割数量决定 char 1字节 char* 4字节
				char mystr_temp[4][10] = {'\0'}; 
				splitString(strArray,(char*)data, ", ");//Զи	按逗号分割
				for (int i = 0; i < 4; i++)
				{
						strcpy(mystr_temp[i],strArray[i]);
						编码器偏移量[i] = atoi(我的字符串临时变量[i]);
				}
		}
		//ٶ	速度
		else if	((strncmp("MSPD",(char*)g_recv_buff_deal,4)==0))
    {
        长度 = strlen((char*)g_recv_buff_deal)-5;
        for (uint8_t i = 0; i < length; i++)
        {
            数据[i] = g_recv_buff_deal[i+5]; //删除冒号
        }  
				数据[长度] = '\0';	
				
				char* strArray[10];;//ָ ȸݷָŶ  char 1ֽ   char* 4ֽ		指针数组 长度由分隔数决定 char 1字节 char* 4字节
				char mystr_temp[4][10] = {'\0'}; 
(strArray,(char*)数据, ", ");//Զи	按逗号分割
				for (int i = 0; i < 4; i++)
				{
						strcpy(mystr_temp[i],strArray[i]);
						g_Speed[i] = atof(mystr_temp[i]);
				}
		}
}
