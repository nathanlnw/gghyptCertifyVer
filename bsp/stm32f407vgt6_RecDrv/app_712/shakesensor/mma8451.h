#ifndef _MMA845X_H_

#define _MMA845X_H_


 typedef  struct _SENSOR
{
    u8  LayingdownCounter;    // �����෭�жϼ�ʱ
    u8  Layingdown_Keeping;  //  �෭��������ʱ�䣬������������λ���
    u8  CollisionCounter;         // ��ײ��������ʱ��
    u8  Collision_Keeping;      //  ��ײ����״̬����ʱ�� 
}SENSOR;

extern SENSOR   ShakeSenor; 


extern void   Sensor_Keep_Process(void);
void mma8451_driver_init(void);
 uint8_t mma8451_config( uint16_t param1, uint16_t param2 ) ;


#endif  /* _MMA845X_H_ */




