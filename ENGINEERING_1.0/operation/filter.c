#include "filter.h"

/*********************************************************************һ�׿�����***********************************************************************/
/**
  * @author  Liu heng
  * һ�׿������˲�������RoboMaster��̳
  *   һά�������˲���
  *   ʹ��ʱ�ȶ���һ��kalmanָ�룬Ȼ�����kalmanCreate()����һ���˲���
  *   ÿ�ζ�ȡ�����������ݺ󼴿ɵ���KalmanFilter()�������ݽ����˲�
  *          ʹ��ʾ��
  *          extKalman_t p;                  //����һ���������˲����ṹ��
  *          float SersorData;             //��Ҫ�����˲�������
  *          KalmanCreate(&p,20,200);      //��ʼ�����˲�����Q=20 R=200����
  *          while(1)
  *          {
  *             SersorData = sersor();                     //��ȡ����
  *             SersorData = KalmanFilter(&p,SersorData);  //�����ݽ����˲�
  *          }
  */

/**
  * @name   kalmanCreate
  * @brief  ����һ���������˲���
  * @param  p:  �˲���
  *         T_Q:ϵͳ����Э����
  *         T_R:��������Э����
  *
  * @retval none
  * @attention R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
  *		       	��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
  */
void KalmanCreate(extKalman_t *p, float T_Q, float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

/**
  * @name   KalmanFilter
  * @brief  �������˲���
  * @param  p:  �˲���
  *         dat:���˲�����
  * @retval �˲��������
  * @attention Z(k)��ϵͳ����,������ֵ   X(k|k)�ǿ������˲����ֵ,���������
  *            A=1 B=0 H=1 I=1  W(K)  V(k)�Ǹ�˹������,�����ڲ���ֵ����,���Բ��ù�
  *            �����ǿ�������5�����Ĺ�ʽ
  *            һ��H'��Ϊ������,����Ϊת�þ���
  */

float KalmanFilter(extKalman_t *p, float dat)
{
    p->X_mid = p->A * p->X_last;                    //���㵱ǰʱ�̵�Ԥ����(1)         x(k|k-1) = A*X(k-1|k-1) + B*U(k) + W(K)
    p->P_mid = p->A * p->P_last + p->Q;             //���㵱ǰʱ��Ԥ������Э����(2)    p(k|k-1) = A*p(k-1|k-1)*A' + Q
    p->kg = p->P_mid / (p->P_mid + p->R);           //����klaman����(4)               kg(k) = p(k|k-1)*H' / (H*p(k|k-1)*H'+R)
    p->X_now = p->X_mid + p->kg * (dat - p->X_mid); //���㵱ǰʱ�̵����Ž��(3)         x(k|k) = X(k|k-1) + kg(k)*(Z(k) - H*X(k|k-1))
    p->P_now = (1 - p->kg) * p->P_mid;              //���㵱ǰʱ�����Ž����Э����(5)    p(k|k) = (I-kg(k)*H) * P(k|k-1)

    p->P_last = p->P_now; //״̬����
    p->X_last = p->X_now;

    return p->X_now; //���Ԥ����x(k|k)
}

/*******************************************************************���׿�����***************************************************************/

/////*
////*���ܣ��Ӿ����������ݳ�ʼ��
////*
////*/
//void MiniPC_Kalman_Data_Init(void)
//{
//   {
//       kalman_Filter_Init.A_data[0] = 1; // 1 0 1 0
//       kalman_Filter_Init.A_data[2] = 1; // 0 1 0 1
//       kalman_Filter_Init.A_data[5] = 1; // 0 0 1 0
//       kalman_Filter_Init.A_data[7] = 1; // 0 0 0 1
//       kalman_Filter_Init.A_data[10] = 1;
//       kalman_Filter_Init.A_data[15] = 1;

//       //�۲����
//       kalman_Filter_Init.H_data[0] = 1; // 1 0 0 0
//       kalman_Filter_Init.H_data[5] = 1; // 0 1 0 0
//                                         // 0 0 0 0
//                                         // 0 0 0 0

//       //״̬ת��Э�������
//       kalman_Filter_Init.P_data[0] = 1;  // 1 0 0 0
//       kalman_Filter_Init.P_data[5] = 1;  // 0 1 0 0
//       kalman_Filter_Init.P_data[10] = 1; // 0 0 1 0
//       kalman_Filter_Init.P_data[15] = 1; // 0 0 0 1

//       //�۲���������
//       kalman_Filter_Init.R_data[0] = 1; // 1 0 0 1
//       kalman_Filter_Init.R_data[3] = 1; // 0 0 0 0
//                                         // 0 0 0 0
//                                         // 0 0 0 0

//       //״̬ת��Э�������
//       kalman_Filter_Init.Q_data[0] = 10;    // {10}  {0}   {0}     {0}
//       kalman_Filter_Init.Q_data[5] = 10;    // {0}   {10}  {0}     {0}
//       kalman_Filter_Init.Q_data[10] = 0.01; // {0}   {0}   {0.01}  {0}
//       kalman_Filter_Init.Q_data[15] = 0.01; // {0}   {0}   {0}     {0.01}
//   }
//   Kalman_Filter_Init(&kalman_Filter, &kalman_Filter_Init);
//}

/*
*���ܣ��Ӿ����ݿ�������ʼ��
*���룺
*/
kalman_filter_init_t kalman_Filter_Init = {0};
kalman_filter_t kalman_Filter = {0};
extern float len;

void Kalman_Filter_Init(kalman_filter_t *F, kalman_filter_init_t *I)
{
    mat_init(&F->xhat, 4, 1, (float *)I->xhat_data);           //4*1 ����ֵ(k-1)
    mat_init(&F->xhatminus, 4, 1, (float *)I->xhatminus_data); //4*1 ����ֵ(k)
    mat_init(&F->z, 2, 1, (float *)I->z_data);                 //2*1 ����ֵ
    mat_init(&F->Pminus, 4, 4, (float *)I->Pminus_data);       //4*4 �µ�Э�������
    mat_init(&F->k, 4, 2, (float *)I->K_data);                 //4*2 kalman����
    mat_init(&F->p, 4, 4, (float *)I->P_data);                 //4*4 Э�������
    mat_init(&F->AT, 4, 4, (float *)I->AT_data);               //4*4 ״̬����
    mat_init(&F->HT, 4, 2, (float *)I->HT_data);               //4*2
    mat_init(&F->A, 4, 4, (float *)I->A_data);                 //4*4
    mat_init(&F->H, 2, 4, (float *)I->H_data);                 //2*4
    mat_init(&F->Q, 4, 4, (float *)I->Q_data);                 //4*4
    mat_init(&F->R, 2, 2, (float *)I->R_data);                 //2*2
    mat_trans(&F->A, &F->AT);
    mat_trans(&F->H, &F->HT);
}

/*
*���ƣ��Ӿ����ݿ���������
*���ܣ���С���Դ��ص����ݽ��п������˲�����
*���룺1���Ӿ����������ݽṹ��  2���ش��Ƕ�  3���ش����ٶ�
*�����auto_yaw_angle_kf    auto_pitch_angle_kf   auto_yaw_speed
*/
void Kalman_Filter_Calc(kalman_filter_t *F, float signal1, float signal2)
{
    float TEMP_data441[16] = {0, 0, 0, 0,
                              0, 0, 0, 0,
                              0, 0, 0, 0,
                              0, 0, 0, 0};
    float TEMP_data442[16] = {0, 0, 0, 0,
                              0, 0, 0, 0,
                              0, 0, 0, 0,
                              0, 0, 0, 0};
    float TEMP_data21[2] = {0, 0};
    float TEMP_data41[16] = {1, 0, 0, 0,
                             0, 1, 0, 0,
                             0, 0, 1, 0,
                             0, 0, 0, 1};
    float TEMP_data24[8] = {0, 0, 0, 0,
                            0, 0, 0, 0};
    float TEMP_data22[4] = {0, 0, 0, 0};
    float TEMP_data222[4] = {0, 0, 0, 0};
    float TEMP_data42[8] = {0, 0, 0, 0,
                            0, 0, 0, 0};
    float TEMP_data212[2] = {0, 0};
    float TEMP_data412[4] = {0, 0, 0, 0};

    mat TEMP441, TEMP442, TEMP21, TEMP41, TEMP24, TEMP22, TEMP222, TEMP42, TEMP212, TEMP412;

    mat_init(&TEMP441, 4, 4, (float *)TEMP_data441);
    mat_init(&TEMP442, 4, 4, (float *)TEMP_data442);
    mat_init(&TEMP21, 2, 1, (float *)TEMP_data21);
    mat_init(&TEMP41, 4, 4, (float *)TEMP_data41);
    mat_init(&TEMP24, 2, 4, (float *)TEMP_data24);
    mat_init(&TEMP22, 2, 2, (float *)TEMP_data22);
    mat_init(&TEMP222, 2, 2, (float *)TEMP_data222);
    mat_init(&TEMP42, 4, 2, (float *)TEMP_data42);
    mat_init(&TEMP212, 2, 1, (float *)TEMP_data212);
    mat_init(&TEMP412, 4, 1, (float *)TEMP_data412);

    F->z.pData[0] = signal1;
    F->z.pData[1] = signal2;

    //! 1. xhat'(k)= A xhat(k-1) - ���㵱ǰʱ�̵�Ԥ����
    mat_mult(&F->A, &F->xhat, &F->xhatminus); //4*4 ,4*1 -->4*1

    //! 2. P'(k) = A P(k-1) AT + Q - ���㵱ǰʱ��Ԥ������Э����
    mat_mult(&F->A, &F->p, &F->Pminus);     //44 44 44
    mat_mult(&F->Pminus, &F->AT, &TEMP441); //44 44 44
    mat_add(&TEMP441, &F->Q, &F->Pminus);   //44 44 44

    //! 3. K(k) = P'(k) HT / (H P'(k) HT + R) - ����klaman����
    mat_mult(&F->H, &F->Pminus, &TEMP24); //24 44 24
    mat_mult(&TEMP24, &F->HT, &TEMP22);   //24 42 22
    mat_add(&TEMP22, &F->R, &TEMP222);    //22 22 22

    mat_inv(&TEMP222, &TEMP22);            //42  44 ����Ӧ����2*2 ����Ϊ  2*2
    mat_mult(&F->Pminus, &F->HT, &TEMP42); //44 42  42
    mat_mult(&TEMP42, &TEMP22, &F->k);     //42 44

    //! 4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k)) - ���㵱ǰʱ�̵����Ž��
    mat_mult(&F->H, &F->xhatminus, &TEMP21);    //24  41  21
    mat_sub(&F->z, &TEMP21, &TEMP212);          //21  21  21
    mat_mult(&F->k, &TEMP212, &TEMP412);        //42  41  21  -> 42 21  41
    mat_add(&F->xhatminus, &TEMP412, &F->xhat); //41  21  41  ->41 41 41

    //! 5. P(k) = (1-K(k)H)P'(k) - ���㵱ǰʱ�����Ž����Э����
    mat_mult(&F->k, &F->H, &TEMP441);      //42  24  44
    mat_sub(&TEMP41, &TEMP441, &TEMP442);  //44  44  44
    mat_mult(&TEMP442, &F->Pminus, &F->p); //44  44  44

    F->filtered_value[0] = F->xhat.pData[0];
    F->filtered_value[1] = F->xhat.pData[1];
    F->filtered_value[2] = F->xhat.pData[2];
    F->filtered_value[3] = F->xhat.pData[3];

    //	vision_auto_data.auto_yaw_angle_kf = F->xhat.pData[0];
    //	vision_auto_data.auto_pitch_angle_kf = F->xhat.pData[1];
    //	vision_auto_data.auto_yaw_speed = F->xhat.pData[2];
}
