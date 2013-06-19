#ifndef _MMA845X_H_

#define _MMA845X_H_


 typedef  struct _SENSOR
{
    u8  LayingdownCounter;    // 连续侧翻判断计时
    u8  Layingdown_Keeping;  //  侧翻报警保持时间，超过了若报警位清除
    u8  CollisionCounter;         // 碰撞报警持续时间
    u8  Collision_Keeping;      //  碰撞报警状态保持时间 
}SENSOR;

extern SENSOR   ShakeSenor; 


extern void   Sensor_Keep_Process(void);
void mma8451_driver_init(void);
 uint8_t mma8451_config( uint16_t param1, uint16_t param2 ) ;


#endif  /* _MMA845X_H_ */




