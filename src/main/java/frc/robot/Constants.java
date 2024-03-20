package frc.robot;

public final class Constants {

    public static final class AutoConstants{
        public static final double kWaitTime7Pts = 7;
    }

    public static final class IntakeConstants
    {
       
        public static final int ROTATE_MOTOR_ID = 14;
        public static final int SPIN_MOTOR_ID = 16;

        public static final double kP = 0.02;//0.01;
        public static final double kI = 0.0000003;//0.0000003;
        public static final double kD = 0.0000018;//0.0000018;
        public static final double kIz = 30.0;
        public static final double kFF = 0.0;

        public static final double kP2 = 0.02;//0.05;
        public static final double kI2 = 0.0000003;//0.0000003;
        public static final double kD2 = 0.0000018;//0.0000018;
        public static final double kIz2 = 30.0;
        public static final double kFF2 = 0.0;

        public static final double kRotationSetpointHigh = 300;
        public static final double kRotationSetpointLow = 0;

        //max output of the feeder
        public static final double kMaxFeedOutput = 0.3;

        //max output at intake class
        public static final double kMaxAbsOutput = 0.75;

        //max output at robot container class
        public static final double kMaxAbsOutputRBExtended = 0.7;
        public static final double kMaxAbsOutputRBRetracted = 0.6;
        public static double collectSpeed=-1;
        public static double releaseSpeed = 1;
    }

    public static final class ShooterConstants 
     {

        public static final int MOTOR_1_ID = 18;  //MAKE SURE TO SWAP THE ID AGAIN
        public static final int MOTOR_2_ID = 19;
        //max output at shooter class
        public static final double kMaxAbsOutput = 1;
        //max output at robot container class for high shot
        public static final double kMaxAbsOutputRBHigh = -0.8;
        //max output at robot container class for low shot
        public static final double kMaxAbsOutputRBLow = -0.2;
       
        public static final double KPShoot = 0.02;
        public static final double kIShoot = 0.0000003;
        public static final double kDShoot = 0.0000018;
        public static final double kIzShoot = 30;
        public static final double kFFShoot = 0;
        public static final double kShooterLowMaxOutput = 0.1;
    }

    public static final class LiftConstants 
    {
        public static final int MOTOR_ID = 20;
        //max output at shooter class
        public static final double kMaxAbsOutput = 1;
        //max output at robot container class for going up
        public static final double kMaxAbsOutputRBUp = 1;
        //max output at robot container class for going down
        public static final double kMaxAbsOutputRBDown = -1;

        public static final int peakCurrentLimit = 30; // the peak current, in amps

        public static final int peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms

        public static final int continuousCurrentLimit = 28; // the current to maintain if the peak limit is triggered
        
        public static final double kPMoving = 0.02;
        public static final double kIMoving = 0.0000003;
        public static final double kDMoving = 0.0000018;
        public static final double kIzMoving = 30.0;
        public static final double kFFMoving = 0.0;

        public static final double kPHolding = 0.1;
        public static final double kIHolding = 0.0;
        public static final double kDHolding = 0.0;
        public static final double kFHolding = 0.0;

        public static final int ENCODER_PORT_A = 1;
        public static final int ENCODER_PORT_B = 2;

        public static final boolean REVERSE_ENCODER = false;

        public static final int TIMEOUT_MS = 30;

        public static final double speed = 1;

        public static final double kLowShootPos = -500;
        public static final double kShootPos = -350;
        public static final double kUnderChainPos = 0;

        public static final double kLiftPos = -780;
        public static final double kRetractPos = -1;

        
    }

    public static final class PS4GamePad{

            /*Button Constants */

            public static final int ButtonSquare = 1;
            public static final int ButtonX = 2;
            public static final int ButtonTriangle = 4;
            public static final int ButtonCircle = 3;
            public static final int ButtonR2 = 8;
            public static final int ButtonL2 = 7;
            public static final int ButtonL1 = 5;
            public static final int ButtonR1 = 6;
            public static final int ButtonL3 = 13;
            public static final int ButtonR3 = 14;
            public static final int ButtonShare = 9;
            public static final int ButtonOption = 10;
            public static final int PSButton = 15;
            public static final int TouchPad = 14;
            public static final int joystickPort = 0;


    }
    
    public static final class Swerve {

        public static final double maxSpeedMperS = 3;
        public static final double maxAngularRate = 1.5;

    }

}