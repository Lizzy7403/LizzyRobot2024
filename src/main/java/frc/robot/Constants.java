package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

    public static final class IntakeConstants
    {
       
        public static final int ROTATE_MOTOR_ID = 14;
        public static final int SPIN_MOTOR_ID = 16;

        public static final double kP = 0.01;
        public static final double kI = 0.0000003;
        public static final double kD = 0.0000018;
        public static final double kIz = 30.0;
        public static final double kFF = 0.0;

        public static final double kRotationSetpointHigh = 300;
        public static final double kRotationSetpointLow = 0;

        //max output at intake class
        public static final double kMaxAbsOutput = 0.5;

        //max output at robot container class
        public static final double kMaxAbsOutputRBExtended = 0.1;
        public static final double kMaxAbsOutputRBRetracted = 0.5;
        public static double collectSpeed=0.1;
    }

    public static final class ShooterConstants 
     {

        public static final int MOTOR_1_ID = 18;
        public static final int MOTOR_2_ID = 19;
        //max output at shooter class
        public static final double kMaxAbsOutput = 0.3;
        //max output at robot container class for high shot
        public static final double kMaxAbsOutputRBHigh = .3;
        //max output at robot container class for low shot
        public static final double kMaxAbsOutputRBLow = .15;
    }

    public static final class LiftConstants 
    {
        public static final int MOTOR_ID = 20;
        //max output at shooter class
        public static final double kMaxAbsOutput = 0.3;
        //max output at robot container class for going up
        public static final double kMaxAbsOutputRBUp = 0.3;
        //max output at robot container class for going down
        public static final double kMaxAbsOutputRBDown = -0.3;

        public static final int peakCurrentLimit = 30; // the peak current, in amps

        public static final int peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms

        public static final int continuousCurrentLimit = 28; // the current to maintain if the peak limit is triggered
        
        public static final double kPMoving = 0.1;
        public static final double kIMoving = 0.0;
        public static final double kDMoving = 0.0;
        public static final double kFMoving = 0.0;

        public static final double kPHolding = 0.1;
        public static final double kIHolding = 0.0;
        public static final double kDHolding = 0.0;
        public static final double kFHolding = 0.0;

        public static final int ENCODER_PORT_A = 1;
        public static final int ENCODER_PORT_B = 2;

        public static final boolean REVERSE_ENCODER = false;

        public static final int TIMEOUT_MS = 30;
        
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
            public static final int TouchPad = 16;
            public static final int joystickPort = 0;


    }
    
    public static final class Swerve {

        public static final double maxSpeedMperS = 3;
        public static final double maxAngularRate = 1.5;

    }

    // Constantes del ejemplo auto, las cuales deben de estar especificadas a cada robot
    public static final class AutoConstants 
    { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
      
    }
}