//****************************************************************************************************************
// SMC3.ini is basic Motor PID driver designed for motion simulators with upto 3 motors (written for UNO R3)
//          
//****************************************************************************************************************

// Uncomment the following line to reverse the direction of Motor 1.

#define REVERSE_MOTOR1

// BOF preprocessor bug prevention - leave me at the top of the arduino-code

#if 1
__asm volatile ("nop");
#endif

#include <SoftwareSerial.h>

#define COM0 0                          // hardware Serial Port
#define COM1 1                          // Software Serial Port
#define START_BYTE '['                  // Start Byte for serial commands
#define END_BYTE ']'                    // End Byte for serial commands
#define PROCESS_PERIOD_uS 250           // 244 -> 4096 per second approx
#define TIMER_UPPER_LIMIT 4294966000    // Used to check for timer wraparound 
#define TIMER_LOWER_LIMIT 1000          // Used to check for timer wraparound

unsigned long TimesUp=0;           // Counter used to see if it's time to calculate next PID update

int Feedback1 = 512;
int Feedback2 = 512;
int Target1 = 512;
int Target2 = 512;

unsigned int RxByte[2]={0};         // Current byte received from each of the two comm ports
int BufferEnd[2]={-1};              // Rx Buffer end index for each of the two comm ports
unsigned int RxBuffer[5][2]={0};    // 5 byte Rx Command Buffer for each of the two comm ports
unsigned long LoopCount	= 0;        // unsigned 32bit, 0 to 4,294,967,295 to count times round loop
unsigned long LastCount = 0;        // loop counter the last time it was read so can calc delta
byte errorcount	= 0;                // serial receive error detected by invalid packet start/end bytes
unsigned int CommsTimeout = 0;      // used to reduce motor power if there has been no comms for a while
byte PowerScale = 7;                // used to divide(shift) PID result changes when in low power

const int R_PWM_1 = 2;          // ENA output pin for Motor H-Bridge 1 (ie PortD bit position) 
const int R_PWM_2 = 4;          // ENA output pin for Motor H-Bridge 2 (ie PortD bit position) 
const int L_PWM_1 = 9;          // PWM output pin for Motor 1   
const int L_PWM_2 = 10;          // PWM output pin for Motor 2   
const int FeedbackPin1 = A0;   // Motor 1 feedback pin
const int FeedbackPin2 = A1;   // Motor 2 feedback pin

int CenterOffset1 = 0;    // Adjust center offset of feedback position
int CenterOffset2 = 0;    // Adjust center offset of feedback position

// Currently disabled
int CutoffLimitMax1 = 1000;    // The position beyond which the motors are disabled
int CutoffLimitMax2 = 1000;    // The position beyond which the motors are disabled
int CutoffLimitMin1 = 23;      // The position beyond which the motors are disabled
int CutoffLimitMin2 = 23;      // The position beyond which the motors are disabled

int InputClipMax1 = 650;    // The input position beyond which the target input is clipped (default: 923)
int InputClipMax2 = 650;    // The input position beyond which the target input is clipped (default: 923)
int InputClipMin1 = 373;      // The input position beyond which the target input is clipped (default: 100)
int InputClipMin2 = 373;      // The input position beyond which the target input is clipped (default: 100)

int PIDProcessDivider = 1;  // divider for the PID process timer
int PIDProcessCounter = 0;
int SerialFeedbackCounter = 0;
int SerialFeedbackEnabled = 0;
int SerialFeedbackPort = 0;

int Ks1 = 0;
long Kp1_x100 = 400;		// initial value 400,0,0
long Ki1_x100 = 0;
long Kd1_x100 = 0;

int Ks2 = 0;
long Kp2_x100 = 400;		// initial value 400,0,0
long Ki2_x100 = 0;
long Kd2_x100 = 0;

int PWMout1 = 0;
int PWMout2 = 0;

int Disable1 = 1;                     //Motor stop flag
int Disable2 = 1;                     //Motor stop flag

int PWMoffset1 = 100; // PMWMin1
int PWMoffset2 = 100; // PMWMin2
int PWMmax1 = 150;
int PWMmax2 = 150;

int LiftFactor1 = 255-PWMmax1;   // Default: 0; Increase PWM when driving motor in direction it has to work harder 
int LiftFactor2 = 255-PWMmax2;   // Default: 0; Increase PWM when driving motor in direction it has to work harder 

void setup()
{
    Serial.begin(500000);     //115200

    pinMode(R_PWM_1, OUTPUT);
    pinMode(L_PWM_1, OUTPUT);
    pinMode(FeedbackPin1, INPUT);

    pinMode(R_PWM_2, OUTPUT);
    pinMode(L_PWM_2, OUTPUT);
    pinMode(FeedbackPin2, INPUT);

    digitalWrite(R_PWM_1, LOW);
    digitalWrite(L_PWM_1, LOW);
    DisableMotor1();

    digitalWrite(R_PWM_2, LOW);
    digitalWrite(L_PWM_2, LOW);
    DisableMotor2();

    // set analog prescale to 16
//    sbi(ADCSRA,ADPS2);
//    cbi(ADCSRA,ADPS1);
//    cbi(ADCSRA,ADPS0);
}

void SendTwoValues(int id, int v1, int v2)
{
    Serial.write(START_BYTE);
    Serial.write(id);
    Serial.write(v1);
    Serial.write(v2);
    Serial.write(END_BYTE);
}

void SendValue(int id, int value)
{
    int low,high;

    high=value/256;
    low=value-(high*256);

    Serial.write(START_BYTE);
    Serial.write(id);
    Serial.write(high);
    Serial.write(low);
    Serial.write(END_BYTE);
}

int DeltaLoopCount()
{
    unsigned long Delta;
    
    if ( (LastCount==0) || ((LoopCount-LastCount)>32000) )
    {
        Delta = 0;
        LastCount = LoopCount;
    }
    else
    {
         Delta = LoopCount-LastCount;
         LastCount  = LoopCount; 
    }    
    return (int)Delta;
}

void ParseCommand(int ComPort)
{
    CommsTimeout = 0;    // reset the comms timeout counter to indicate we are getting packets
    
    switch (RxBuffer[0][ComPort]) 
    {
        case 'A':
            Target1=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort]+CenterOffset1;
#ifdef REVERSE_MOTOR1
            Target1=1023-Target1;
#endif

            if (Target1>InputClipMax1) {
                Target1=InputClipMax1;
            }
            else if (Target1<InputClipMin1) {
                Target1=InputClipMin1;
            }
            break;
        case 'B':
            Target2=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort]+CenterOffset2;

            if (Target2>InputClipMax2) {
                Target2=InputClipMax2;
            }
            else if (Target2<InputClipMin2) {
                Target2=InputClipMin2;
            }
            break;
        case 'r':
            if (RxBuffer[1][ComPort]=='d')      // rd - Read a value from the ArduinoPID - next byte identifies value to read
            {
                switch (RxBuffer[2][ComPort]) 
                {
                    case 'A':
#ifdef REVERSE_MOTOR1
                        SendTwoValues('A',(1023-Feedback1-CenterOffset1)/4,(1023-Target1-CenterOffset1)/4);
#else
                        SendTwoValues('A',(Feedback1+CenterOffset1)/4,(Target1+CenterOffset1)/4);
#endif
                        break;
                    case 'B':
                        SendTwoValues('B',(Feedback2+CenterOffset2)/4,(Target2+CenterOffset2)/4);
                        break;
                    case 'a':
                        //SendTwoValues('a',PIDProcessDivider*16 + Disable1+(Disable2*2)+(Disable3*4),constrain(PWMout1,0,255));
                        SendTwoValues('a',PIDProcessDivider*16 + Disable1+(Disable2*2),constrain(PWMout1,0,255));
                        break;
                    case 'b':
                        //SendTwoValues('b',PIDProcessDivider*16 + Disable1+(Disable2*2)+(Disable3*4),constrain(PWMout1,0,255));
                        SendTwoValues('b',PIDProcessDivider*16 + Disable1+(Disable2*2),constrain(PWMout2,0,255));
                        break;
                    case 'D':
                        SendValue('D',Kp1_x100);
                        break;
                    case 'E':
                        SendValue('E',Kp2_x100);
                        break;
                    case 'G':
                        SendValue('G',Ki1_x100);
                        break;
                    case 'H':
                        SendValue('H',Ki2_x100);
                        break;
                    case 'J':
                        SendValue('J',Kd1_x100);
                        break;
                    case 'K':
                        SendValue('K',Kd2_x100);
                        break;
                    case 'M':
                        SendValue('M',Ks1);
                        break;
                    case 'N':
                        SendValue('N',Ks2);
                        break;
                    case 'P':
                        SendTwoValues('P',PWMoffset1,PWMmax1);
                        break;
                    case 'Q':
                        SendTwoValues('Q',PWMoffset2,PWMmax2);
                        break;
                    case 'S':
                        SendTwoValues('S',CutoffLimitMin1,InputClipMin1);
                        break;
                    case 'T':
                        SendTwoValues('T',CutoffLimitMin2,InputClipMin2);
                        break;
                    case 'Y':
                        // TODO remove?
                        SendTwoValues('Y',PIDProcessDivider*16 + Disable1,0);  // Second byte not yet used
                        break;
                    case 'Z':
                        SendValue('Z',DeltaLoopCount());
//                        SendValue('Z',errorcount);            // *** TEMP CODE FOR ETESTING
                        break;
                    case '~':
                        //SendTwoValues('~',Timer1FreqkHz,0);  // PWM Frequencies to set
                        break;
                     case '?':
                        break;
//                    default:
                }
            }
            break;
        case 'D':
            Kp1_x100=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            break;
        case 'E':
            Kp2_x100=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            break;
        case 'G':
            Ki1_x100=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            break;
        case 'H':
            Ki2_x100=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            break;
        case 'J':
            Kd1_x100=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            break;
        case 'K':
            Kd2_x100=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            break;
        case 'M':
            Ks1=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];       
            break;
        case 'N':
            Ks2=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];       
            break;
        case 'P':
            PWMoffset1=RxBuffer[1][ComPort];
            PWMmax1=RxBuffer[2][ComPort];
            break;
        case 'Q':
            PWMoffset2=RxBuffer[1][ComPort];
            PWMmax2=RxBuffer[2][ComPort];
            break;
        case 'S':
            CutoffLimitMin1=RxBuffer[1][ComPort];
            CutoffLimitMax1=1023-CutoffLimitMin1;
            InputClipMin1=RxBuffer[2][ComPort];
            InputClipMax1=1023-InputClipMin1;
            break;
        case 'T':
            CutoffLimitMin2=RxBuffer[1][ComPort];
            CutoffLimitMax2=1023-CutoffLimitMin2;
            InputClipMin2=RxBuffer[2][ComPort];
            InputClipMax2=1023-InputClipMin2;
            break;
        case 'V':
            //DeadZone1=RxBuffer[1][ComPort];
            //PWMrev1=RxBuffer[2][ComPort];
            break;
        case 'W':
            //DeadZone2=RxBuffer[1][ComPort];
            //PWMrev2=RxBuffer[2][ComPort];
            break;
        case 'Z':
            PIDProcessDivider=constrain(RxBuffer[1][ComPort],1,10);
            // ???=RxBuffer[2][ComPort];   // second byte not yet used
            break;
        case '~':
            //Timer1FreqkHz=RxBuffer[1][ComPort];
            //InitialisePWMTimer1(Timer1FreqkHz * 1000);
            break;
        case 'm':
            if (RxBuffer[1][ComPort]=='o' && RxBuffer[2][ComPort]=='1')   // Monitor motor 1 (auto sends position and feedback data to host
            {
                SerialFeedbackEnabled= (ComPort << 4) | 1;   // Motor 1
            }
            if (RxBuffer[1][ComPort]=='o' && RxBuffer[2][ComPort]=='2')   // Monitor motor 2 (auto sends position and feedback data to host
            {
                SerialFeedbackEnabled= (ComPort << 4) | 2;   // Motor 2
            }
            if (RxBuffer[1][ComPort]=='o' && RxBuffer[2][ComPort]=='0')   // Switch off auto monitoring data feedback
            {
                SerialFeedbackEnabled=0;
            }
            break;
        case 'v':
            if (RxBuffer[1][ComPort]=='e' && RxBuffer[2][ComPort]=='r')   // Send back the SMC3 software version
            {
                SendValue('v',70);     // Software Version - divide by 100 to get version - ie 101= ver1.01
            }
            break;
        case 'e':
            if (RxBuffer[1][ComPort]=='n' && RxBuffer[2][ComPort]=='a')   // Enable all motors
            {
                Disable1=0;
                Disable2=0;
            }
            else if (RxBuffer[1][ComPort]=='n' && RxBuffer[2][ComPort]=='1')   // Enable motor 1
            {
                Disable1=0;
            }
            else if (RxBuffer[1][ComPort]=='n' && RxBuffer[2][ComPort]=='2')   // Enable motor 2
            {
                Disable2=0;
            }
            break;
        case '?':
            break;
//        default: 
    }  
}

void CheckSerial0()
{
    while(Serial.available()) 
    {
        if(BufferEnd[COM0]==-1)
        {
            RxByte[COM0] = Serial.read();
            if(RxByte[COM0] != START_BYTE){BufferEnd[COM0]=-1;errorcount++;}else{BufferEnd[COM0]=0;}
        }
        else
        {
            RxByte[COM0] = Serial.read();
            RxBuffer[BufferEnd[COM0]][COM0]=RxByte[COM0];
            BufferEnd[COM0]++;
            if(BufferEnd[COM0] > 3)
            {
                if(RxBuffer[3][COM0]==END_BYTE)
                {
                    ParseCommand(COM0);
                }
                else
                {
                    errorcount++;
                }
                BufferEnd[COM0]=-1;
            }
        }
    }
}

void SetOutputsMotor1()
{
    if((Target1 > (Feedback1)) || (Target1 < (Feedback1)))
    {
        if (PWMout1 >= 0)  
        {                                    
            // Drive Motor Forward 
            PWMout1+=PWMoffset1;
            if(PWMout1 > (PWMmax1)){
                PWMout1=PWMmax1;
            }
            digitalWrite(L_PWM_1, LOW);
            analogWrite(R_PWM_1, PWMout1);
        }  
        else 
        {                                              
            // Drive Motor Backwards 
            PWMout1 = abs(PWMout1);
            PWMout1+=PWMoffset1;
            if(PWMout1 > PWMmax1){
                PWMout1=PWMmax1;
            }
            //drive harder
            PWMout1+=LiftFactor1;
            digitalWrite(R_PWM_1, LOW);
            analogWrite(L_PWM_1, PWMout1);
        }
    }
    else
    {
        // Brake Motor 
        digitalWrite(R_PWM_1, LOW);
        digitalWrite(L_PWM_1, LOW);
        PWMout1=PWMoffset1;
    }
}

void SetOutputsMotor2()
{
    if((Target2 > (Feedback2)) || (Target2 < (Feedback2)))
    {
        if (PWMout2 >= 0)  
        {                                    
            // Drive Motor Forward 
            PWMout2+=PWMoffset2;
            if(PWMout2 > (PWMmax2)){
                PWMout2=PWMmax2;
            }
            digitalWrite(L_PWM_2, LOW);
            analogWrite(R_PWM_2, PWMout2);
        }  
        else 
        {                                              
            // Drive Motor Backwards 
            PWMout2 = abs(PWMout2);
            PWMout2+=PWMoffset2;
            if(PWMout2 > PWMmax2){
                PWMout2=PWMmax2;
            }
            //drive harder
            PWMout2+=LiftFactor2;
            digitalWrite(R_PWM_2, LOW);
            analogWrite(L_PWM_2, PWMout2);
        }
    }
    else
    {
        // Brake Motor 
        digitalWrite(R_PWM_2, LOW);
        digitalWrite(L_PWM_2, LOW);
        PWMout2=PWMoffset2;
    }
}

int CalcMotor1PID(int TargetPosition, int CurrentPosition)   
{
    static int Error=0;                                                     
    static long pTerm_x100=0;
    static long dTerm_x100=0;
    static long iTerm_x100=0;
    static int CumError=0;
    static int LastPosition=0;
    static int DeltaPosition=0;
    static int KdFilterCount=0;

    Error = TargetPosition - CurrentPosition;
    if (abs(Error)<=0)//DeadZone1)
    {
        CumError = 0;
    }
    else
    {
        CumError += Error;
        CumError = constrain(CumError,-1024,1024);            
    }         

    pTerm_x100 = Kp1_x100 * (long)Error;                    // Error can only be between +/-1023 and Kp1_100 is constrained to 0-1000 so can work with type long
    iTerm_x100 = (Ki1_x100 * (long)CumError);        // was >>6

    KdFilterCount++;
    if (KdFilterCount >= Ks1)          // Apply a level of filtering to Kd parameter to help reduce motor noise
    {
        DeltaPosition = (CurrentPosition - LastPosition);
        LastPosition = CurrentPosition;
        dTerm_x100 = (Kd1_x100 * (long)DeltaPosition);   // was <<5
        KdFilterCount=0;    
    }
    
    return constrain((pTerm_x100 + iTerm_x100 - dTerm_x100) >> PowerScale ,-255,255);   //  the /128 (PowerScale) is an approximation to bring x100 terms back to units.  Accurate/consistent enough for this function.
}

int CalcMotor2PID(int TargetPosition, int CurrentPosition)   
{
    static int Error=0;                                                     
    static long pTerm_x100=0;
    static long dTerm_x100=0;
    static long iTerm_x100=0;
    static int CumError=0;
    static int LastPosition=0;
    static int DeltaPosition=0;
    static int KdFilterCount=0;

    Error = TargetPosition - CurrentPosition;
    if (abs(Error)<=0)//DeadZone2)
    {
        CumError = 0;
    }
    else
    {
        CumError += Error;
        CumError = constrain(CumError,-1024,1024);            
    }         

    pTerm_x100 = Kp2_x100 * (long)Error;                    // Error can only be between +/-1023 and Kp1_100 is constrained to 0-1000 so can work with type long
    iTerm_x100 = (Ki2_x100 * (long)CumError);        // was >>6

    KdFilterCount++;
    if (KdFilterCount >= Ks2)          // Apply a level of filtering to Kd parameter to help reduce motor noise
    {
        DeltaPosition = (CurrentPosition - LastPosition);
        LastPosition = CurrentPosition;
        dTerm_x100 = (Kd2_x100 * (long)DeltaPosition);   // was <<5
        KdFilterCount=0;    
    }
    
    return constrain((pTerm_x100 + iTerm_x100 - dTerm_x100) >> PowerScale ,-255,255);   //  the /128 (PowerScale) is an approximation to bring x100 terms back to units.  Accurate/consistent enough for this function.
}

void DisableMotor1()
{
    Disable1 = 1;

    digitalWrite(R_PWM_1, LOW);
    digitalWrite(L_PWM_1, LOW);
}

void DisableMotor2()
{
    Disable2 = 1;

    digitalWrite(R_PWM_2, LOW);
    digitalWrite(L_PWM_2, LOW);
}

void TogglePin()
{
   static int PinOut=0;
   PinOut=1-PinOut;
   digitalWrite(8, PinOut); 
}

void loop()
{
    Disable1=0;
    Disable2=0;

    // Some temporary debug stuff
    //   Serial.write('S');
    //   Serial.write(TCCR1A);
    //   Serial.write(TCCR1B);
    //   Serial.write(OCR1AL);
    //   Serial.write(OCR1AH);

    // Initialise the PID ready timer

    TimesUp = micros();
    PIDProcessCounter=0;
    
    while (1==1) 
    {
        // Wait until its time and then update PID calcs for first motor

        while ((micros() - TimesUp) < PROCESS_PERIOD_uS) { ; }
        TimesUp += PROCESS_PERIOD_uS;
        TogglePin();                      // Used for testing to monitor PID timing on Oscilloscope

        PIDProcessCounter++;

        if (PIDProcessCounter >= PIDProcessDivider)
        {
            PIDProcessCounter=0;
            
            // Check and Update Motor 1 drive
            Feedback1 = analogRead(FeedbackPin1);
            if ((Feedback1 > CutoffLimitMax1) || (Feedback1 < CutoffLimitMin1)) { 
                //DisableMotor1(); 
            }
            PWMout1=CalcMotor1PID(Target1,Feedback1);
            if (Disable1==0) 
            { 
                SetOutputsMotor1(); 
            }
            else
            {
                PWMout1=0;
            }

            // Check and Update Motor 2 drive
            Feedback2 = analogRead(FeedbackPin2);
            if ((Feedback2 > CutoffLimitMax2) || (Feedback2 < CutoffLimitMin2)) { 
                //DisableMotor2(); 
            }
            PWMout2=CalcMotor2PID(Target2,Feedback2);
            if (Disable2==0) 
            { 
                SetOutputsMotor2(); 
            }
            else
            {
                PWMout2=0;
            }

            //
            LoopCount++;
        }
        
        CheckSerial0();

        SerialFeedbackCounter++;
        if (SerialFeedbackCounter >= 80)  // every 20ms send back position, pwm and motor status updates if enabled 
        {
            SerialFeedbackPort = (SerialFeedbackEnabled >> 4) & 0x01;
            if  ((SerialFeedbackEnabled & 0x03) == 1)     // Monitor Motor 1
            {
#ifdef REVERSE_MOTOR1
               SendTwoValues('A',(1023-Feedback1-CenterOffset1)/4,(1023-Target1-CenterOffset1)/4);
#else
               SendTwoValues('A',(Feedback1+CenterOffset1)/4,(Target1+CenterOffset1)/4);
#endif
               //SendTwoValues('a',PIDProcessDivider*16 + Disable1+(Disable2*2)+(Disable3*4),constrain(PWMout1,0,255));
               SendTwoValues('a',PIDProcessDivider*16 + Disable1+(Disable2*2),constrain(PWMout1,0,255));
            }
            else if  ((SerialFeedbackEnabled & 0x03) == 2)     // Monitor Motor 2
            {
               SendTwoValues('B',(Feedback2+CenterOffset2)/4,(Target2+CenterOffset2)/4);
               //SendTwoValues('a',PIDProcessDivider*16 + Disable1+(Disable2*2)+(Disable3*4),constrain(PWMout2,0,255));
               SendTwoValues('b',PIDProcessDivider*16 + Disable1+(Disable2*2),constrain(PWMout2,0,255));
            }

            SerialFeedbackCounter = 0;
        }

        CommsTimeout++;
        if (CommsTimeout >= 60000)       // 15 second timeout ie 60000 * PROCESS_PERIOD_uS
        {
            CommsTimeout = 60000;        // So the counter doesn't overflow
            PowerScale = 9; 
        }
        else
        {
            PowerScale = 7;              // Number of bits to shift PID result (ie divide by 128 - approximate for /100)
        }
    }
}
