package frc.robot.subsystems;

// Importing the CANSparkMax class from the REV Robotics library
// This class provides methods to control the Spark MAX motor controller
import com.revrobotics.CANSparkMax;

// Importing the RelativeEncoder class from the REV Robotics library
// This class provides methods to get the position and velocity of the encoder
import com.revrobotics.RelativeEncoder;

// Importing the SparkMaxPIDController class from the REV Robotics library
// This class provides methods to control a motor using PID
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

// Importing the ControlType enum from the REV Robotics library
// This enum provides constants to specify the control type for the PID controller

// Importing the MotorType enum from the REV Robotics library
// This enum provides constants to specify the type of motor (brushed or brushless)

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Importing the SubsystemBase class from the WPILib library
// This class provides the base for creating subsystems, which are major parts of the robot
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Importing the Constants class from the robot's code
// This class contains constant values used throughout the robot's code
import frc.robot.Constants;

// The Intake class represents the intake subsystem of the robot
public class Intake extends SubsystemBase {

  // The motor that rotates the entire intake
  // This is a Spark MAX motor controller controlling a brushless motor
  private CANSparkMax m_intakeMotorRotate;

  private static boolean isExtended = false;

  // The motor that rolls the intake tape
  // This is also a Spark MAX motor controller controlling a brushless motor
  private CANSparkMax m_intakeMotorRoller;

  // The PID controller for the rotation motor
  // This is used to control the position of the intake
  private SparkPIDController m_pidRotateController;
  private SparkPIDController m_pidRotateControllerUp;


  // The encoder for the rotation motor
  // This is used to get the position and velocity of the intake
  private final RelativeEncoder  m_rotateEncoder;
  


  private final DigitalInput limitSwitch1 = new DigitalInput(0);
  private final DigitalInput limitSwitch2 = new DigitalInput(1);
  private final DigitalInput limitSwitch3 = new DigitalInput(2);




  // The constructor for the Intake class 
    // This is called when an Intake object is created
  public Intake() {

    // Initializing the rotation motor with its ID and specifying that it's a brushless motor
    m_intakeMotorRotate = new CANSparkMax(Constants.IntakeConstants.ROTATE_MOTOR_ID ,MotorType.kBrushless);

    // Initializing the roller motor with its ID and specifying that it's a brushless motor
    m_intakeMotorRoller = new CANSparkMax(Constants.IntakeConstants.SPIN_MOTOR_ID ,MotorType.kBrushed);
    m_intakeMotorRoller.setOpenLoopRampRate(0);

    // Getting the encoder from the rotation motor
    m_rotateEncoder = m_intakeMotorRotate.getEncoder();

    // Getting the PID controller from the rotation motor
    m_pidRotateController = m_intakeMotorRotate.getPIDController();

    // Setting the encoder as the feedback device for the PID controller
    m_pidRotateController.setFeedbackDevice(m_rotateEncoder);

    // Setting the PID constants for the PID controller
    m_pidRotateController.setP(Constants.IntakeConstants.kP);
    m_pidRotateController.setI(Constants.IntakeConstants.kI);
    m_pidRotateController.setD(Constants.IntakeConstants.kD);
    m_pidRotateController.setIZone(Constants.IntakeConstants.kIz);
    m_pidRotateController.setFF(Constants.IntakeConstants.kFF);
    // Setting the output range for the PID controller
    m_pidRotateController.setOutputRange(-1. * Constants.IntakeConstants.kMaxAbsOutputRBExtended, Constants.IntakeConstants.kMaxAbsOutputRBRetracted);

    
    m_pidRotateControllerUp = m_intakeMotorRotate.getPIDController();

    // Setting the encoder as the feedback device for the PID controller
    m_pidRotateControllerUp.setFeedbackDevice(m_rotateEncoder);

    m_pidRotateControllerUp.setP(Constants.IntakeConstants.kP2);
    m_pidRotateControllerUp.setI(Constants.IntakeConstants.kI2);
    m_pidRotateControllerUp.setD(Constants.IntakeConstants.kD2);
    m_pidRotateControllerUp.setIZone(Constants.IntakeConstants.kIz2);
    m_pidRotateControllerUp.setFF(Constants.IntakeConstants.kFF2);
    // Setting the output range for the PID controller
    m_pidRotateControllerUp.setOutputRange(-1. * Constants.IntakeConstants.kMaxAbsOutputRBExtended, Constants.IntakeConstants.kMaxAbsOutputRBRetracted);
  
    
  
  
  }

  public static boolean isExtended() {
      return isExtended;
  }

  public static void setIsExtended(boolean isExtended){
    Intake.isExtended = isExtended;
  }

  public void setPID(double kP, double kI, double kD, double kIz, double kFF){
    m_pidRotateControllerUp.setP(kP);
    m_pidRotateControllerUp.setI(kI);
    m_pidRotateControllerUp.setD(kD);
    m_pidRotateControllerUp.setIZone(kIz);
    m_pidRotateControllerUp.setFF(kFF);

  }

  public void resetPID(){
    m_pidRotateControllerUp.setP(Constants.LiftConstants.kPMoving);
    m_pidRotateControllerUp.setI(Constants.LiftConstants.kIMoving);
    m_pidRotateControllerUp.setD(Constants.LiftConstants.kDMoving);
    m_pidRotateControllerUp.setIZone(Constants.LiftConstants.kIzMoving);
    m_pidRotateControllerUp.setFF(Constants.LiftConstants.kFFMoving);
  }

  public void rotateIntakeWithPID(double setpoint, double kP, double kI, double kD, double kIz, double kFF) {
    m_pidRotateController.setP(kP);
    m_pidRotateController.setI(kI);
    m_pidRotateController.setD(kD);
    m_pidRotateController.setIZone(kIz);
    m_pidRotateController.setFF(kFF);
    m_pidRotateController.setReference(setpoint, ControlType.kPosition);
  }

  // Method to rotate the intake to a specific position
  // The setpoint parameter is the desired position of the intake
  public void rotateIntake(double setpoint) {
    m_pidRotateController.setReference(setpoint, ControlType.kPosition);
  }

 public void rotateIntakeUp(double setpoint) {
    m_pidRotateControllerUp.setReference(setpoint, ControlType.kPosition);
}

  public void rotateFree(double speed){

    m_intakeMotorRotate.set(speed);
  }

  public void centerNote(double speed, int time)
  {
    m_intakeMotorRotate.set(speed);
    m_intakeMotorRotate.setCANTimeout(time);
    m_intakeMotorRotate.set(-speed);
    m_intakeMotorRotate.setCANTimeout(time);
    m_intakeMotorRotate.set(speed);
    m_intakeMotorRotate.setCANTimeout(time);
    m_intakeMotorRotate.set(-speed);
    m_intakeMotorRotate.setCANTimeout(time);

  }

  

  public boolean getLimitSwitch(){
    

    if(limitSwitch1.get() || limitSwitch2.get() || limitSwitch3.get()){
      return true;
    
    }
    else
    return false;
  
  }

  // Method to stop rotating the intake
  // This is done by setting the desired position of the intake to 0
  public void stopRotateIntake() {
    m_pidRotateController.setReference(0, ControlType.kVelocity);

  }

  // Method to spin the intake at a specific speed
  // The speed parameter is a double value between -1.0 and 1.0
  public void spinIntake(double speed) {
    m_intakeMotorRoller.set(speed);
  }
  public void feed(double speed){
    
    m_intakeMotorRoller.set(speed);

  }

  // Method to get the current position of the intake
  // This is done by getting the position from the encoder
  public double getRotateEncoderPosition() {
    return m_rotateEncoder.getPosition();
  }

  // Method to reset the position of the intake
  // This is done by setting the position of the encoder to 0
  public void resetRotateEncoder() {
    m_rotateEncoder.setPosition(0);
  }

  // This method will be called once per scheduler run
  // Currently, it does not perform any operations
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake ENC POS", m_rotateEncoder.getPosition());
    //SmartDashboard.putNumber("Intake ENC SP", m_rotateEncoder.getVelocity());
    SmartDashboard.putBoolean("Intake limitswitch", getLimitSwitch());
    
  }
}