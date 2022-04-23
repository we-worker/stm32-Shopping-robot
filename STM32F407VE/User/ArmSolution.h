void ArmDriver_Init(void);
void SetServoAngle(int nServo,float angle);
void ArmSolution(double x,double y);
void Arm_Grab(void);



void Slow_Pwm(uint8_t nServo);

extern int Object_pos[6][2];
extern uint8_t Object_pos_index;


