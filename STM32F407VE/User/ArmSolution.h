void ArmDriver_Init(void);
void SetServoAngle(int nServo,float angle);
void ArmSolution(double x,double y);
void Arm_Grab(void);



extern int grab_flag;//抓取标志，因为不知道为什么读取到上位机操作的时候不能直接运行抓取函数
void Slow_Pwm(uint8_t nServo);

extern int Object_pos[6][2];
extern uint8_t Object_pos_index;


