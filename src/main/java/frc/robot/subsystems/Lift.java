package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Importing the SubsystemBase class from the WPILib library
// This class provides the base for creating subsystems, which are major parts of the robot
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Importing the Constants class from the robot's code
// This class contains constant values used throughout the robot's code
//import frc.robot.Constants;
import frc.robot.*;

public class Lift extends SubsystemBase {


  private final CANSparkMax motor;

  private SparkPIDController pidLiftController;

  private final RelativeEncoder liftEncoder;
  
  private DoubleSolenoid m_doubleSolenoid;

  public Lift() {
    motor = new CANSparkMax(Constants.LiftConstants.MOTOR_ID, MotorType.kBrushless);
    motor.setOpenLoopRampRate(1);
    
    

    m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    liftEncoder = motor.getEncoder();

    pidLiftController = motor.getPIDController();

    pidLiftController.setFeedbackDevice(liftEncoder);
  
    pidLiftController.setP(Constants.LiftConstants.kPMoving);
    pidLiftController.setI(Constants.LiftConstants.kIMoving);
    pidLiftController.setD(Constants.LiftConstants.kDMoving);
    pidLiftController.setIZone(Constants.LiftConstants.kIzMoving);
    pidLiftController.setFF(Constants.LiftConstants.kFFMoving);

    motor.setClosedLoopRampRate(1);

    //output range
    pidLiftController.setOutputRange( -1 * Constants.LiftConstants.kMaxAbsOutput, Constants.LiftConstants.kMaxAbsOutput);
  }

  public void setPosition(double position) {
    pidLiftController.setReference(position, CANSparkBase.ControlType.kPosition);
  }

  public void moveLift(double speed){

    motor.set(speed);
  }


  public void resetLiftEncoder() {
    liftEncoder.setPosition(0);
  }

  public void stopLift() {
    motor.set(0);
  }

  public void lockPosition(double kP, double kI, double kD, double kIz, double kFF) {
    setPID(kP, kI, kD, kIz, kFF);
    pidLiftController.setReference(liftEncoder.getPosition(), CANSparkBase.ControlType.kPosition);
  }

  public void setPID(double kP, double kI, double kD, double kIz, double kFF){
    pidLiftController.setP(kP);
    pidLiftController.setI(kI);
    pidLiftController.setD(kD);
    pidLiftController.setIZone(kIz);
    pidLiftController.setFF(kFF);

  }

  public void resetPID(){
    pidLiftController.setP(Constants.LiftConstants.kPMoving);
    pidLiftController.setI(Constants.LiftConstants.kIMoving);
    pidLiftController.setD(Constants.LiftConstants.kDMoving);
    pidLiftController.setIZone(Constants.LiftConstants.kIzMoving);
    pidLiftController.setFF(Constants.LiftConstants.kFFMoving);
  }

  public void closeSolanoid() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void openSolanoid() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  

  @Override
  public void periodic() {

   // SmartDashboard.putNumber("LIFT ENCODER", liftEncoder.getPosition());
    //SmartDashboard.putNumber("Intake ENC SP", m_rotateEncoder.getVelocity());

  }
}