package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// Importing the SubsystemBase class from the WPILib library
// This class provides the base for creating subsystems, which are major parts of the robot
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Importing the Constants class from the robot's code
// This class contains constant values used throughout the robot's code
//import frc.robot.Constants;
import frc.robot.*;

public class Lift extends SubsystemBase {


  private final CANSparkMax motor;

  private SparkPIDController pidRotateController;

  private final RelativeEncoder encoder;
  
  private DoubleSolenoid m_doubleSolenoid;

  public Lift() {
    motor = new CANSparkMax(Constants.LiftConstants.MOTOR_ID, MotorType.kBrushless);

    m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    encoder = motor.getEncoder();

    pidRotateController = motor.getPIDController();

    pidRotateController.setFeedbackDevice(encoder);
  
    pidRotateController.setP(Constants.LiftConstants.kPMoving);
    pidRotateController.setI(Constants.LiftConstants.kIMoving);
    pidRotateController.setD(Constants.LiftConstants.kDMoving);
    pidRotateController.setIZone(Constants.LiftConstants.kIzMoving);
    pidRotateController.setFF(Constants.LiftConstants.kFFMoving);

    motor.setClosedLoopRampRate(1);

    //output range
    pidRotateController.setOutputRange( -1 * Constants.LiftConstants.kMaxAbsOutput, Constants.LiftConstants.kMaxAbsOutput);
  }

  public void setPosition(double position) {
    pidRotateController.setReference(position, CANSparkBase.ControlType.kPosition);
  }

  public void resetRotateEncoder() {
    encoder.setPosition(0);
  }

  public void stopLift() {
    motor.set(0);
  }

  public void lockPosition(double kP, double kI, double kD, double kIz, double kFF) {
    setPID(kP, kI, kD, kIz, kFF);
    pidRotateController.setReference(encoder.getPosition(), CANSparkBase.ControlType.kPosition);
  }

  public void setPID(double kP, double kI, double kD, double kIz, double kFF){
    pidRotateController.setP(kP);
    pidRotateController.setI(kI);
    pidRotateController.setD(kD);
    pidRotateController.setIZone(kIz);
    pidRotateController.setFF(kFF);

  }

  public void resetPID(){
    pidRotateController.setP(Constants.LiftConstants.kPMoving);
    pidRotateController.setI(Constants.LiftConstants.kIMoving);
    pidRotateController.setD(Constants.LiftConstants.kDMoving);
    pidRotateController.setIZone(Constants.LiftConstants.kIzMoving);
    pidRotateController.setFF(Constants.LiftConstants.kFFMoving);
  }

  public void closeSolanoid() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void openSolanoid() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  

  @Override
  public void periodic() {
  }
}