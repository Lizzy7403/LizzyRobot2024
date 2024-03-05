// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.CollectCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ToggleIntakeCommand;
import frc.robot.commands.ToggleLiftCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Intake;


public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final Shooter shooter = new Shooter();
  private final Lift lift = new Lift();
  private final Intake intake =new Intake();

  
   /* Setting up bindings for necessary control of the swerve drive platform */
  private final PS4Controller joystick = new PS4Controller(Constants.PS4GamePad.joystickPort);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private double MaxSpeed = Constants.Swerve.maxSpeedMperS; // 6 meters per second desired top speed
  private double MaxAngularRate = Constants.Swerve.maxAngularRate * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() 
  {
    
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));


    JoystickButton brakeSwerve = new JoystickButton(joystick, PS4Controller.Button.kCircle.value);
    brakeSwerve.whileTrue(drivetrain.applyRequest(() -> brake));

    JoystickButton pointToDirection = new JoystickButton(joystick, PS4Controller.Button.kSquare.value);
    pointToDirection.whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
    
    // reset the field-centric heading on Option Button pressed 
    JoystickButton resetHeading = new JoystickButton(joystick, PS4Controller.Button.kOptions.value);//resets the field relative of the robot towards the direction its currenty facing
    resetHeading.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    //calls a  parallel command to start first the shooter and a few seconds later the feeder
    JoystickButton shootHigh = new JoystickButton(joystick,PS4Controller.Button.kR2.value); 
     //shootHigh.onTrue((Commands.parallel(Commands.waitSeconds(1).asProxy().andThen(new Feed(intake,0.2).withTimeout(1)),new RotateIntakeCommand(intake, Constants.IntakeConstants.kRotationSetpointHigh))));
    shootHigh.onTrue((Commands.parallel(Commands.waitSeconds(1).asProxy().andThen(new FeedCommand(intake, Constants.IntakeConstants.kMaxAbsOutputRBRetracted).withTimeout(1)),new ShootCommand(shooter, Constants.ShooterConstants.kMaxAbsOutputRBHigh).withTimeout(2))));

    JoystickButton shootLow = new JoystickButton(joystick,PS4Controller.Button.kR1.value);
    shootLow.onTrue((Commands.parallel(Commands.waitSeconds(1).asProxy().andThen(new FeedCommand(intake, Constants.IntakeConstants.kMaxAbsOutputRBRetracted).withTimeout(1)),new ShootCommand(shooter, Constants.ShooterConstants.kMaxAbsOutputRBLow).withTimeout(2))));

    JoystickButton toggleLift = new JoystickButton(joystick, PS4Controller.Button.kTriangle.value);
    toggleLift.onTrue(new ToggleLiftCommand(lift));
    
    JoystickButton spinIntake = new JoystickButton(joystick, PS4Controller.Button.kCross.value);
    spinIntake.whileTrue(new CollectCommand(intake, Constants.IntakeConstants.kMaxAbsOutputRBExtended, Constants.IntakeConstants.kMaxAbsOutputRBRetracted));

    JoystickButton moveIntake = new JoystickButton(joystick, PS4Controller.Button.kL1.value);
    moveIntake.onTrue(new ToggleIntakeCommand(intake, Constants.IntakeConstants.kRotationSetpointHigh, Constants.IntakeConstants.kRotationSetpointLow));
    

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }


  public RobotContainer() 
  {
    configureBindings();
  }

  public Command getAutonomousCommand() { //Uses the program from Path planner to create an autonomous code
    return Commands.print("No autonomous command configured");
  }
}
