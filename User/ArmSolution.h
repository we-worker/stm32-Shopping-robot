void ArmDriver_Init(void);
void SetServoAngle(int nServo,float angle);
void ArmSolution(double x,double y);

extern float arm_angle4;
extern int height;
void Slow_Pwm(uint8_t nServo);
