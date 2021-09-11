#include "MathLib.h"







/*************************************************************************************************
*����:	Angle_Limiting
*����:	�ٽǴ���
*�β�: 	int16_t Angl_Err
*����:	��
*˵��:	��
*************************************************************************************************/
float Angle_Limiting(float Angl_Err,float limit)   
{
	if(Angl_Err < -limit/2)
	{
		Angl_Err += limit;
	}
	if(Angl_Err > limit/2)
	{
		Angl_Err -= limit;
	}
	return Angl_Err;
}





/*��ֵ��С���ƺ���*/
int16_t limit(int16_t x,int16_t max,int16_t min)
{
	if(x>max)
		x=max;
	else if(x<min)
		x=min;
	return x;
}

float float_limit(float x,float max,float min)
{
	if(x>max)
		x=max;
	else if(x<min)
		x=min;
	return x;
}

signed long limit_long(signed long x,signed long max,signed long min)
{
	if(x>max)
		x=max;
	else if(x<min)
		x=min;
	return x;
}



/*����ֵ���㺯��*/
int16_t int16_t_abs(int16_t x)				
{
	if(x < 0)
		x=-x;
	return x;
}

signed long long_abs(signed long x)			
{
	if(x < 0)
		x=-x;
	return x;
}

float float_abs(float x)					
{
	if(x < 0)
		x=-x;
	return x;
}




/*����������*/
int16_t Dead_Zone(int16_t Value,int16_t Limit)
{
	if(int16_t_abs(Value) < Limit)
		Value = 0;
	return Value;
}




/*��ֵ�������ƺ���*/
int16_t LIMIT_RAISE(int16_t raise,int16_t now,int16_t Limit)
{
	if(raise - now > Limit)
		raise = now + Limit;
	else if(raise - now < -Limit)
		raise = now - Limit;
	return raise;
}




/*�ҳ����ֵ*/
int16_t RETURN_MAX(int16_t*Value,int16_t num)
{
	static int16_t MAX_I;
	int16_t MAX = int16_t_abs(Value[0]);
	for(MAX_I = 1;MAX_I < num;MAX_I ++)
	{
		if(int16_t_abs(Value[MAX_I]) > MAX)
			MAX = int16_t_abs(Value[MAX_I]);
	}
	return MAX;
}



/*���ຯ��*/
int16_t Remainder(int16_t Divisor,int16_t divisor)
{
	int16_t Gain = Divisor / divisor;
	return (Divisor - Gain*divisor);
}



/*���ź���*/
int16_t int16_t_Sign(int16_t NUM)
{
	if(NUM >= 0)
	{
		return 1;
	}
	else if(NUM < 0)
	{
		return -1;
	}
	return 0;
}



int16_t Float_Sign(float NUM)
{
	if(NUM >= 0.0f)
	{
		return 1;
	}
	else if(NUM < 0.0f)
	{
		return -1;
	}
	return 0;
}









/*����ֵ��ת����*/
/*����˵����תʱ������������*/
fp32 INOVERSHOOT(fp32 IN,fp32 NOW,fp32 REFER)
{
		if(IN - NOW > REFER)
		{
			IN = IN - REFER;
			if(NOW >= IN)
			{
				IN = IN + REFER;
			}
		}
		else if(IN - NOW < -REFER)
		{
			IN = IN + REFER;
			if(NOW <= IN)
			{
				IN = IN - REFER;
			}
		}
		return IN;
}





