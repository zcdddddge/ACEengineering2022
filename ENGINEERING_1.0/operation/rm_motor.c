#include "rm_motor.h"
#include <stddef.h>
#include "parameter.h"

/**
 *�ļ����ݣ� 1.���ֵ����PID�ṹ�嶨��͵����ʵֵ�Ķ���
 *          2.����ıջ��㷨���ٶȻ����ٶ�λ�û���       
 *          3.������ٵ������ֵ���㷨
 *          4.��������ٽ紦��
 *          5.���ٶ������㷨
 *          6.�����ת���
 *          7.������̨���̲���㷨
 *��ע���ṹ�帳ֵ��ʼ����system�ļ���
 *      PID����������PID_Def�ļ���
 */

/*=========================����ջ��㷨==============================*/
/*
���ܣ��ٶȻ����ƣ�
����������ٶȻ�pid�ṹ��spid��Ŀ���ٶ�ֵsetSpeed��ʵ���ٶ�ֵactualSpeed
����������
*/
int16_t motor_speed_control(pid_parameter_t *spid, int16_t setSpeed, int16_t actualSpeed, int16_t current_limit)
{
    spid->SetValue = setSpeed;
    spid->ActualValue = actualSpeed;

    location_pid_int32(spid, spid->ActualValue);
    spid->out = int32_limit(spid->out, current_limit, -current_limit);

    return spid->out;
}

/*
*���ܣ�λ���ٶȴ����ջ�����
*���룺����ٶ�pid�ṹ�壬���λ��pid�ṹ�壬�����ʵλ�ã������ʵ�ٶȣ�λ��Ŀ��ֵ����������ֵ
*����������
*/
int16_t motor_position_speed_control(pid_parameter_t *speed_pid, pid_parameter_t *position_pid, int16_t actual_position, int16_t actual_speed, int16_t setPosition, int16_t current_limit)
{
    int32_t output;

    position_pid->SetValue = setPosition;                                    //λ�û��趨ֵΪ��������� �����ǽǶȴ��� (����ͷʼ����ǰ)
    speed_pid->SetValue = location_pid_int32(position_pid, actual_position); //�ٶȻ��趨ֵ��λ�û�����

    output = location_pid_int32(speed_pid, actual_speed);        //��������
    output = int32_limit(output, current_limit, -current_limit); //�������

    return output;
}

int16_t motor_position_Stepping(pid_parameter_t *speed_pid, pid_parameter_t *position_pid, int16_t actual_position, int16_t actual_speed, int16_t setPosition, int16_t current_limit)
{
    int32_t output;

    //position_pid->SetValue = setPosition;   //λ�û��趨ֵΪ��������� �����ǽǶȴ��� (����ͷʼ����ǰ)
    step_in_processing(position_pid, setPosition); //����PID �趨ֵ����

    speed_pid->SetValue = pid_regulator(position_pid, actual_position); //�ٶȻ��趨ֵ��λ�û�����

    output = location_pid_int32(speed_pid, actual_speed);        //��������
    output = int32_limit(output, current_limit, -current_limit); //�������

    return output;
}

/*=========================RM���һ���㷨========================*/
/*��ʵ����ֵ����(ͨ�ã�������ʲô������������ֻҪ�漰����Ȧ����ʽ�Ķ�������)
*���룺������ݽṹ�壬���ٱȳ˴����ȣ���Ȧ����ֵ
*��������ʼֵΪ0�������Ҹ�����Ȧ���ɵ�����
*/
void motor_actual_position(motor_measure_t *rmMotor, int16_t gear_Ratio, int16_t lap_encoder)
{
    if (rmMotor->first_Flag == 0) //��һ�ν���ʱ��¼����ֵ
    {
        rmMotor->last_position = rmMotor->position;
        rmMotor->first_Flag = 1;
    }
    rmMotor->actual_Position += angle_limiting_int16(rmMotor->position - rmMotor->last_position, lap_encoder); //��ֵ�ۼ�
    rmMotor->actual_Position = check_codevalue(rmMotor->actual_Position, gear_Ratio, lap_encoder);             //���ٽ�ֵ��λ����ֵ
    rmMotor->last_position = rmMotor->position;
}

//�ٽǴ���16λ����Ӧ�Ƕ���ֵ��
int16_t angle_limiting_int16(int16_t Angl_Err, int16_t lap_encoder)
{
    if (Angl_Err < -(lap_encoder / 2))
    {
        Angl_Err += (lap_encoder - 1);
    }
    if (Angl_Err > (lap_encoder / 2))
    {
        Angl_Err -= (lap_encoder - 1);
    }
    return Angl_Err;
}

//�ٽǴ���32λ�����ٱȼ���
int32_t angle_limiting_int32(int32_t Angl_Error, int16_t buff, int16_t lap_encoder)
{
    if (Angl_Error < -(buff * (lap_encoder / 2)))
    {
        Angl_Error += (buff * (lap_encoder - 1));
    }
    if (Angl_Error > (buff * (lap_encoder / 2)))
    {
        Angl_Error -= (buff * (lap_encoder - 1));
    }
    return Angl_Error;
}

//���ٽ�ֵ��λ����ֵ ��������360�ȵ�����ֵѭ�� DJI�����
int32_t check_codevalue(int32_t value, int16_t gear_Ratio, int16_t lap_encoder)
{
    if (value > (gear_Ratio * lap_encoder) / 2)
    {
        value = value - (gear_Ratio * lap_encoder);
    }
    if (value < (-(gear_Ratio * lap_encoder) / 2))
    {
        value = (gear_Ratio * lap_encoder) - value;
    }

    return value;
}

//�����ת���
static int16_t Block_Count = 0;

int16_t check_motor_block(int16_t position)
{
    static int16_t last_Position;
    if (int16_t_abs(last_Position - position) < 10)
        Block_Count++;
    else
        Block_Count = 0;
    last_Position = position;
    if (Block_Count > 100)
        return 1;
    else
        return 0;
}

//Y������������Ĳ�ǻ�ȡ(�Ե�������Ϊ0�ȣ�����Ϊ���ǣ�����Ϊ���ǣ���С��ΧΪ0~180��)
float Yaw_Different_Angle = 0;                    //��̨���̲��
int32_t Yaw_Middle_Code = (8192 * YAW_RATIO) / 2; //Y�������̵İ�ֵ

float get_yaw_different_angle(const motor_measure_t *yaw_position, int16_t Ratio)
{

    Yaw_Different_Angle = (yaw_position->actual_Position * 360) / (8192 * Ratio);

    return Yaw_Different_Angle;
}

/*
*��Ȧ����ֵ����������ת��
*����ֵ����ʵ����ֵ
*/
int16_t rotate_50_cirecle = 0;
int16_t last_code = 0;

int16_t encoder_real(int32_t read_code)
{
    //	int code = 0;
    int code_output = 0;

    if (read_code <= 49152)
    {
        //		code = (read_code) / 3072;
        //		code_output = read_code - code * 3072;//����Ȧ������ɵ�Ȧ��

        code_output = read_code % 3072;
        return code_output;
    }
    else if (read_code > 49152)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

/*
*���ܣ�Yaw����ʵλ�ã���Ȧ���������������Ҹ���
*���룺��ȡ������ʵ��Ȧ����ֵ����Ȧ�����������м�λ�õ�����ֵ
*���������м�λ��Ϊ��׼���ٽǴ���λ��ֵ�����Ȧ�����Ұ�Ȧ����
*/

int16_t yaw_actual_code_conversion(int16_t actual_code, int16_t max_code, int16_t middle_code)
{
    if (0 <= actual_code && actual_code <= middle_code) //������ֵ���м�Ϊ��׼
    {
        last_code = max_code - (middle_code - actual_code);
        if (last_code >= (max_code / 2) && last_code <= max_code) //�ٽǴ���
            last_code = last_code - max_code;
        else if (last_code >= 0 && last_code < (max_code / 2))
            last_code = last_code;
    }
    else if (middle_code < actual_code && actual_code <= 3072)
    {
        last_code = actual_code - middle_code;
        if (last_code >= (max_code / 2) && last_code <= max_code) //�ٽǴ���
            last_code = last_code - max_code;
        else if (last_code >= 0 && last_code < (max_code / 2))
            last_code = last_code;
    }
    else
        last_code = -1;

    return last_code;
}

//void trigger_motor_turn_back(void)
//{
//    if (shoot_control.block_time < BLOCK_TIME)
//    {
//        shoot_control.speed_set = shoot_control.trigger_speed_set;
//    }
//    else
//    {
//        shoot_control.speed_set = -shoot_control.trigger_speed_set;
//    }

//    if (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
//    {
//        shoot_control.block_time++;
//        shoot_control.reverse_time = 0;
//    }
//    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
//    {
//        shoot_control.reverse_time++;
//    }
//    else
//    {
//        shoot_control.block_time = 0;
//    }
//}
