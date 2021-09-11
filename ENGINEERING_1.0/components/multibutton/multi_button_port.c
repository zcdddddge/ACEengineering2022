#include "multi_button_port.h"
#include <stdio.h>

static button_control_t button_control;

void key_w_callback(void *button);
void key_s_callback(void *button);
void key_a_callback(void *button);
void key_d_callback(void *button);
void key_shift_callback(void *button);
void key_ctrl_callback(void *button);
void key_q_callback(void *button);
void key_e_callback(void *button);
void key_r_callback(void *button);
void key_f_callback(void *button);
void key_g_callback(void *button);
void key_z_callback(void *button);
void key_x_callback(void *button);
void key_c_callback(void *button);
void key_v_callback(void *button);
void key_b_shift_ctrl_callback(void *button);
void mouse_l_callback(void *button);
void mouse_r_callback(void *button);





/* Callback_key function */
void Callback_key(void *argument)
{
//	button_ticks(); //开启按键扫描
}






//按键状态读取接口
uint8_t read_key_w()
{
    return (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_W);
}
uint8_t read_key_s()
{
    return (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_S);
}
uint8_t read_key_a()
{
    return (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_A);
}
uint8_t read_key_d()
{
    return (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_D);
}
uint8_t read_key_shift()
{
    return (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT);
}
uint8_t read_key_ctrl()
{
    return (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL);
}
uint8_t read_key_q()
{
    return (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_Q);
}
uint8_t read_key_e()
{
    return (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_E);
}
uint8_t read_key_r()
{
    return (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_R);
}
uint8_t read_key_f()
{
    return (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_F);
}
uint8_t read_key_g()
{
    return (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_G);
}
uint8_t read_key_z()
{
    return (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_Z);
}
uint8_t read_key_x()
{
    return (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_X);
}
uint8_t read_key_c()
{
    return (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_C);
}
uint8_t read_key_v()
{
    return (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_V);
}
uint8_t read_key_b_shift_ctrl()
{
    return ((button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_B)&&(button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)&&(button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL));
}
uint8_t read_press_l()
{
	if ((button_control.chassis_button->chassis_RC->mouse.press_l == 1) && ((button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_Z) == 0) && ((button_control.chassis_button->chassis_RC->mouse.press_r == 0)))
		return 1;
	else
		return 0;
}
uint8_t read_press_r()
{
	if ((button_control.chassis_button->chassis_RC->mouse.press_r == 1) && ((button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_Z) == 0) && ((button_control.chassis_button->chassis_RC->mouse.press_l == 0)))
		return 1;
	else
		return 0;
}

void button_all_init(void)
{
    //button_control.chassis_button->chassis_RC = get_remote_control_point();
    button_control.chassis_button = get_chassis_control_point();

    //初始化按键对象
    button_init(&button_control.key_w, read_key_w, 1);
    button_init(&button_control.key_s, read_key_s, 1);
    button_init(&button_control.key_a, read_key_a, 1);
    button_init(&button_control.key_d, read_key_d, 1);
    button_init(&button_control.key_shift, read_key_shift, 1);
    button_init(&button_control.key_ctrl, read_key_ctrl, 1);
    button_init(&button_control.key_q, read_key_q, 1);
    button_init(&button_control.key_e, read_key_e, 1);
    button_init(&button_control.key_r, read_key_r, 1);
    button_init(&button_control.key_f, read_key_f, 1);
    button_init(&button_control.key_g, read_key_g, 1);
    button_init(&button_control.key_z, read_key_z, 1);
    button_init(&button_control.key_x, read_key_x, 1);
    button_init(&button_control.key_c, read_key_c, 1);
    button_init(&button_control.key_v, read_key_v, 1);
    button_init(&button_control.restart, read_key_b_shift_ctrl, 1);
	button_init(&button_control.mouse_r, read_press_r, 1);
	button_init(&button_control.mouse_l, read_press_l, 1);

    //注册按钮事件回调函数
    button_attach(&button_control.key_w, PRESS_DOWN, key_w_callback);
	button_attach(&button_control.key_w, PRESS_UP, key_w_callback);
    button_attach(&button_control.key_s, PRESS_DOWN, key_s_callback);
	button_attach(&button_control.key_s, PRESS_UP, key_s_callback);
    button_attach(&button_control.key_a, PRESS_DOWN, key_a_callback);
	button_attach(&button_control.key_a, PRESS_UP, key_a_callback);
    button_attach(&button_control.key_d, PRESS_DOWN, key_d_callback);
	button_attach(&button_control.key_d, PRESS_UP, key_d_callback);
    button_attach(&button_control.key_shift, PRESS_DOWN, key_shift_callback);
	button_attach(&button_control.key_shift, PRESS_UP, key_shift_callback);
    button_attach(&button_control.key_ctrl, PRESS_DOWN, key_ctrl_callback);
	button_attach(&button_control.key_ctrl, PRESS_UP, key_ctrl_callback);
//    button_attach(&button_control.key_q, PRESS_DOWN, key_q_callback);
//    button_attach(&button_control.key_e, PRESS_DOWN, key_e_callback);
    button_attach(&button_control.key_r, PRESS_DOWN, key_r_callback);
	button_attach(&button_control.key_r, PRESS_UP, key_r_callback);
    button_attach(&button_control.key_f, PRESS_DOWN, key_f_callback);
	button_attach(&button_control.key_f, PRESS_UP, key_f_callback);
    button_attach(&button_control.key_g, PRESS_DOWN, key_g_callback);
    button_attach(&button_control.key_z, PRESS_DOWN, key_z_callback);
    button_attach(&button_control.key_x, PRESS_DOWN, key_x_callback);
    button_attach(&button_control.key_c, PRESS_DOWN, key_c_callback);
    button_attach(&button_control.key_v, PRESS_DOWN, key_v_callback);
    button_attach(&button_control.restart, PRESS_DOWN, key_b_shift_ctrl_callback);
	button_attach(&button_control.mouse_r, PRESS_DOWN, mouse_r_callback);
	button_attach(&button_control.mouse_l, PRESS_DOWN, mouse_l_callback);

    //启动按键
    button_start(&button_control.key_a);
}
//**W 前
void key_w_callback(void *button)
{
    uint32_t btn_event_val;
    btn_event_val = get_button_event((struct Button *)button);
    switch (btn_event_val)
    {
    case PRESS_DOWN: //按键按下，每次按下都触发
		if (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT) //Shift    加速移动
		{
			button_control.chassis_button->Key_Press.KEY_W = HIGH_FORWARD_BACK_SPEED; //600.
		}
		else
		{
			button_control.chassis_button->Key_Press.KEY_W = NORMAL_FORWARD_BACK_SPEED;
		}
        break;
	case PRESS_UP: //按键弹起，每次松开都触发
		button_control.chassis_button->Key_Press.KEY_W = 0;
        break;

    case PRESS_REPEAT: //重复按下触发，变量 repeat 计数连击次数
        break;

    case SINGLE_CLICK: //单击按键事件 (按下松开才触发
        break;

    case DOUBLE_CLICK: //双击按键事件
        break;

    case LONG_PRESS_START: //达到长按时间阈值时触发一次
        break;

    case LONG_PRESS_HOLD: //长按期间一直触发
        break;
    }
}
//**S 后
void key_s_callback(void *button)
{
    switch ((uint32_t)get_button_event((struct Button *)button))
    {
    case PRESS_DOWN: //按键按下，每次按下都触发
		if (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT) //Shift    加速移动
		{
			button_control.chassis_button->Key_Press.KEY_S = HIGH_FORWARD_BACK_SPEED; //600.
		}
		else
		{
			button_control.chassis_button->Key_Press.KEY_S = NORMAL_FORWARD_BACK_SPEED;
		}
        break;
	case PRESS_UP: //按键弹起，每次松开都触发
		button_control.chassis_button->Key_Press.KEY_S = 0;
        break;
    }
}
//**A 左
void key_a_callback(void *button)
{
    switch ((uint32_t)get_button_event((struct Button *)button))
    {
    case PRESS_DOWN: //按键按下，每次按下都触发
		if (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT) //Shift    加速移动
		{
			button_control.chassis_button->Key_Press.KEY_A = HIGH_FORWARD_BACK_SPEED; //600.
		}
		else
		{
			button_control.chassis_button->Key_Press.KEY_A = NORMAL_FORWARD_BACK_SPEED;
		}
        break;
	case PRESS_UP: //按键弹起，每次松开都触发
		button_control.chassis_button->Key_Press.KEY_A = 0;
        break;
    }
}
//**D 右
void key_d_callback(void *button)
{
    switch ((uint32_t)get_button_event((struct Button *)button))
    {
    case PRESS_DOWN: //按键按下，每次按下都触发
		if (button_control.chassis_button->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT) //Shift    加速移动
		{
			button_control.chassis_button->Key_Press.KEY_D = HIGH_FORWARD_BACK_SPEED; //600.
		}
		else
		{
			button_control.chassis_button->Key_Press.KEY_D = NORMAL_FORWARD_BACK_SPEED;
		}
        break;
	case PRESS_UP: //按键弹起，每次松开都触发
		button_control.chassis_button->Key_Press.KEY_D = 0;
        break;
    }
}
//**Shife 电容放电
void key_shift_callback(void *button)
{
    switch ((uint32_t)get_button_event((struct Button *)button))
    {
    case PRESS_DOWN: //按键按下，每次按下都触发
		button_control.chassis_button->Key_Press.KEY_SHIFT = 1;
        button_control.chassis_button->SuperCap_discharge_flag = 1;
        break;
	case PRESS_UP: //按键弹起，每次松开都触发
		button_control.chassis_button->Key_Press.KEY_SHIFT = 0;
        button_control.chassis_button->SuperCap_discharge_flag = 0;
        break;
    }
}
//**Ctrl 炮台模式
void key_ctrl_callback(void *button)
{
    switch ((uint32_t)get_button_event((struct Button *)button))
    {
    case PRESS_DOWN: //按键按下，每次按下都触发
        button_control.chassis_button->Key_Press.KEY_CTRL = 1;
        button_control.chassis_button->chassis_mode = CHASSIS_BATTERY; //炮台模式
        break;
	case PRESS_UP: //按键弹起，每次松开都触发
		button_control.chassis_button->Key_Press.KEY_CTRL = 0;
        break;
    }
}
////**Q 左转
//void key_q_callback(void *button)
//{
//    switch ((uint32_t)get_button_event((struct Button *)button))
//    {
//    case PRESS_DOWN: //按键按下，每次按下都触发
//在云台板中做处理
//        break;
//    }
//}
////**E 右转
//void key_e_callback(void *button)
//{
//    switch ((uint32_t)get_button_event((struct Button *)button))
//    {
//    case PRESS_DOWN: //按键按下，每次按下都触发
//在云台板中做处理
//        break;
//    }
//}
//**R 小陀螺模式
void key_r_callback(void *button)
{
    switch ((uint32_t)get_button_event((struct Button *)button))
    {
    case PRESS_DOWN: //按键按下，每次按下都触发
		button_control.chassis_button->chassis_mode = CHASSIS_ROTATION; //小陀螺模式
        break;
	case PRESS_UP: //按键弹起，每次松开都触发
		button_control.chassis_button->Key_Press.KEY_CTRL = 0;
        break;
    }
}
//**F  扭腰模式
void key_f_callback(void *button)
{
    switch ((uint32_t)get_button_event((struct Button *)button))
    {
    case PRESS_DOWN: //按键按下，每次按下都触发
		button_control.chassis_button->chassis_mode = CHASSIS_TWIST_WAIST; //底盘扭腰模式
        break;
	case PRESS_UP: //按键弹起，每次松开都触发
		button_control.chassis_button->chassis_mode = CHASSIS_FOLLOW;
        break;
    }
}
//**G 补给模式
void key_g_callback(void *button)
{
    switch ((uint32_t)get_button_event((struct Button *)button))
    {
    case PRESS_DOWN: //按键按下，每次按下都触发

        break;
    }
}
void key_z_callback(void *button)
{
    switch ((uint32_t)get_button_event((struct Button *)button))
    {
    case PRESS_DOWN: //按键按下，每次按下都触发

        break;
    }
}
//**X 打符模式
void key_x_callback(void *button)
{
    switch ((uint32_t)get_button_event((struct Button *)button))
    {
    case PRESS_DOWN: //按键按下，每次按下都触发

        break;
    }
}
void key_c_callback(void *button)
{
    switch ((uint32_t)get_button_event((struct Button *)button))
    {
    case PRESS_DOWN: //按键按下，每次按下都触发
		button_control.chassis_button->Fire_task_control->shoot_speed = FRICTION_THREE_SHOOT_SPEED;
        break;
    }
}
void key_v_callback(void *button)
{
    switch ((uint32_t)get_button_event((struct Button *)button))
    {
    case PRESS_DOWN: //按键按下，每次按下都触发
		button_control.chassis_button->Fire_task_control->shoot_speed = FRICTION_ONE_SHOOT_SPEED;
        break;
    }
}
void key_b_shift_ctrl_callback(void *button)
{
    switch ((uint32_t)get_button_event((struct Button *)button))
    {
    case PRESS_DOWN: //按键按下，每次按下都触发
		
        break;
    }
}
void mouse_l_callback(void *button)
{
    switch ((uint32_t)get_button_event((struct Button *)button))
    {
    case PRESS_DOWN: //按键按下，每次按下都触发
		button_control.chassis_button->Fire_task_control->fire_workstatus = FIRE;
        break;
	case PRESS_UP: //按键弹起，每次松开都触发
		button_control.chassis_button->Fire_task_control->fire_workstatus = STOP_FIRE;
        break;
    }
}
void mouse_r_callback(void *button)
{
    switch ((uint32_t)get_button_event((struct Button *)button))
    {
    case PRESS_DOWN: //按键按下，每次按下都触发
		button_control.chassis_button->chassis_mode = CHASSIS_BATTERY;//CHASSIS_FOLLOW;  //开炮台模式，测试自瞄
        break;
	case PRESS_UP: //按键弹起，每次松开都触发
		button_control.chassis_button->chassis_mode = CHASSIS_FOLLOW;
        break;
    }
}

