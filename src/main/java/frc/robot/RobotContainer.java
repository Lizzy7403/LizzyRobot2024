// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.annotation.Retention;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;

// PATH PLANNER LIBRARIES

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.MoveLiftCommand;
import frc.robot.commands.RotateFree;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.commands.Collect;
import frc.robot.commands.Feed;
import frc.robot.commands.MoveLift;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootPID;
import frc.robot.commands.SolenoidDown;
import frc.robot.commands.SolenoidUp;
import frc.robot.commands.ToggleIntakeCommand;
import frc.robot.commands.ToggleSolenoid;
import frc.robot.commands.RotateIntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Intake;

public class RobotContainer {

  private final Shooter shooter = new Shooter();
  private final Lift lift = new Lift();
  private final Intake intake = new Intake();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final PS4Controller joystick = new PS4Controller(Constants.PS4GamePad.joystickPort);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private double MaxSpeed = Constants.Swerve.maxSpeedMperS; // 6 meters per second desired top speed
  private double MaxAngularRate = Constants.Swerve.maxAngularRate * Math.PI; // 3/4 of a rotation per second max angular
                                                                             // velocity

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  ////////////////////////////////////////////// AUTO CONFIGURATION
  ////////////////////////////////////////////// ./////////////////////

  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  private Command autopath1 = drivetrain.getAutoPath("autopath1");
  private Command autopath2 = drivetrain.getAutoPath("autopath2");
  private Command autopath3 = drivetrain.getAutoPath("autopath3");
  private Command autopath4 = drivetrain.getAutoPath("autopath4");
  private Command autopath5 = drivetrain.getAutoPath("autopath5");
  private Command autopath6 = drivetrain.getAutoPath("autopath6");
  private Command autopath7 = drivetrain.getAutoPath("autopath7");
  private Command autopath75 = drivetrain.getAutoPath("autopath7.5");
  private Command autopath8 = drivetrain.getAutoPath("autopath8");
  private Command autopath9 = drivetrain.getAutoPath("autopath9");
  private Command autopath10 = drivetrain.getAutoPath("autopath10");


  private void configureBindings() {

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    JoystickButton pointToDirection = new JoystickButton(joystick, PS4Controller.Button.kPS.value);
    pointToDirection.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // SHOOTING BUTTONS

    JoystickButton shootManual = new JoystickButton(joystick, PS4Controller.Button.kCross.value);
    shootManual.whileTrue(new ShootCommand(shooter, -0.1));

    // calls a parallel command to start first the shooter and a few seconds later
    // the feeder
    JoystickButton shootHigh = new JoystickButton(joystick, PS4Controller.Button.kCircle.value);
    /*
     * shootHigh.onTrue(Commands.parallel(
     * Commands.waitSeconds(4).asProxy().andThen(new Feed(intake,
     * 1).withTimeout(1)),
     * new ShootCommand(shooter,
     * Constants.ShooterConstants.kMaxAbsOutputRBHigh).withTimeout(5.5),
     * new MoveLiftCommand(lift, Constants.LiftConstants.kShootPos))
     * .andThen(new MoveLiftCommand(lift, LiftConstants.kUnderChainPos)));
     */

    shootHigh.onTrue(Commands.parallel(
        Commands.waitSeconds(1.5).asProxy().andThen(new Feed(intake, 1).withTimeout(1)),
        new ShootCommand(shooter, Constants.ShooterConstants.kMaxAbsOutputRBHigh).withTimeout(2.5)));

    JoystickButton shootLow = new JoystickButton(joystick, PS4Controller.Button.kCross.value);
    shootLow.whileTrue(Commands.parallel(new ShootCommand(shooter, -0.2).withTimeout(1),
        Commands.waitSeconds(0.5).andThen(new Feed(intake, 1))));

    // INTAKE BUTTON)

    JoystickButton collectNote = new JoystickButton(joystick, PS4Controller.Button.kR1.value);
    collectNote.whileTrue(new Collect(intake, Constants.IntakeConstants.collectSpeed));

    JoystickButton releaseNote = new JoystickButton(joystick, PS4Controller.Button.kL1.value);
    releaseNote.whileTrue(new Feed(intake, Constants.IntakeConstants.releaseSpeed));

    JoystickButton rotateIn = new JoystickButton(joystick, PS4Controller.Button.kL2.value);
    // rotateFree.whileTrue(new RotateFree(intake,
    // Constants.IntakeConstants.kMaxAbsOutputRBExtended));
    rotateIn.onTrue(new RotateIntakeCommand(intake, 2));

    JoystickButton rotateOut = new JoystickButton(joystick, PS4Controller.Button.kR2.value);
    rotateOut.onTrue(new RotateIntakeCommand(intake, 55));

    // LIFT BUTTONS

    JoystickButton moveLiftUp = new JoystickButton(joystick, PS4Controller.Button.kOptions.value);
    moveLiftUp.whileTrue(new MoveLift(lift, Constants.LiftConstants.speed));

    JoystickButton moveLiftDown = new JoystickButton(joystick, PS4Controller.Button.kShare.value);
    moveLiftDown.whileTrue(new MoveLift(lift, -Constants.LiftConstants.speed));

    JoystickButton upSoleButton = new JoystickButton(joystick, PS4Controller.Button.kSquare.value);

    upSoleButton.onTrue(new SolenoidDown(lift));

    JoystickButton downSoleButton = new JoystickButton(joystick, PS4Controller.Button.kTriangle.value);

    downSoleButton.onTrue(new SolenoidUp(lift));

    JoystickButton prepForLow = new JoystickButton(joystick, PS4Controller.Button.kR3.value);

    prepForLow.onTrue(new MoveLiftCommand(lift, -500));

    JoystickButton endLow = new JoystickButton(joystick, PS4Controller.Button.kL3.value);

    endLow.onTrue(new MoveLiftCommand(lift, 0));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    lift.setLiftEncoder(0);
    intake.resetRotateEncoder();
    shooter.resetShooterEncoder();

    m_chooser.addOption("BlueAutoLeftSpeaker", blueAutoLeftSpeaker());
    m_chooser.addOption("BlueAutoCenterSpeaker", blueAutoCenterSpeaker());
    m_chooser.addOption("BlueAutoRightSpeaker", blueAutoRightSpeaker());
    m_chooser.addOption("17PointAuto", blueAutoCenterSpeaker17());
    SmartDashboard.putData("Sendable Chooser", m_chooser);

    configureBindings();
  }

  private Command safetyShoot() {
    return Commands.parallel(
        Commands.waitSeconds(2).asProxy().andThen(new Feed(intake, 1).withTimeout(1.5)),
        new ShootCommand(shooter, Constants.ShooterConstants.kMaxAbsOutputRBHigh).withTimeout(3.5));
  }

  private Command autoShoot() {
    return Commands.parallel(
        Commands.waitSeconds(1.5).asProxy().andThen(new Feed(intake, 1).withTimeout(1)),
        new ShootCommand(shooter, Constants.ShooterConstants.kMaxAbsOutputRBHigh).withTimeout(2.5));
  }

  private Command riskyShoot() {
    return Commands.parallel(
        Commands.waitSeconds(1.25).asProxy().andThen(new Feed(intake, 1).withTimeout(0.75)),
        new ShootCommand(shooter, Constants.ShooterConstants.kMaxAbsOutputRBHigh).withTimeout(2));
  }

  private Command autoCenter() {

    return new Collect(intake, Constants.IntakeConstants.collectSpeed).withTimeout(0.2)
        .andThen(new Feed(intake, 1).withTimeout(0.2)
            .andThen(new Collect(intake, IntakeConstants.collectSpeed).withTimeout(0.2)));
  }

  // AUTOS

  private Command blueAutoLeftSpeaker() {

    return autoShoot().andThen(new RotateIntakeCommand(intake, 55))
        .andThen(
            Commands.parallel(autopath1, new Collect(intake, Constants.IntakeConstants.collectSpeed).withTimeout(2)))
        .andThen(new RotateIntakeCommand(intake, 0)).andThen(
            Commands.parallel(autopath2,
                Commands.waitSeconds(1).andThen(autoCenter()).andThen(autoCenter()).andThen(autoCenter())))
        .andThen(autoShoot()).andThen(new MoveLiftCommand(lift, LiftConstants.kUnderChainPos))
        .andThen(autopath3);

  }

  private Command blueAutoCenterSpeaker() {

    return riskyShoot().andThen(new RotateIntakeCommand(intake, 55))
        .andThen(
            Commands.parallel(autopath4, new Collect(intake, Constants.IntakeConstants.collectSpeed).withTimeout(1.5)))
        .andThen(new RotateIntakeCommand(intake, 0)).andThen(
            Commands.parallel(autopath5, Commands.waitSeconds(1).andThen(autoCenter()).andThen(autoCenter())))
        .andThen(riskyShoot()).andThen(new MoveLiftCommand(lift, LiftConstants.kUnderChainPos))
        .andThen(autopath6);
  }

  private Command blueAutoRightSpeaker() {

    return safetyShoot().andThen(new RotateIntakeCommand(intake, 55))
        .andThen(
            Commands.parallel(autopath8, new Collect(intake, Constants.IntakeConstants.collectSpeed).withTimeout(1.5)))
        .andThen(new RotateIntakeCommand(intake, 0)).andThen(
            Commands.parallel(autopath9, Commands.waitSeconds(1).andThen(autoCenter()).andThen(autoCenter()).andThen(autoCenter())))
        .andThen(safetyShoot()).andThen(new MoveLiftCommand(lift, LiftConstants.kUnderChainPos))
        .andThen(autopath10);
  }

  
   private Command blueAutoCenterSpeaker17(){
    
    return autoShoot().andThen(new RotateIntakeCommand(intake, 55))
    .andThen( Commands.parallel(autopath4, new Collect(intake,
    Constants.IntakeConstants.collectSpeed).withTimeout(2)))
    .andThen(new RotateIntakeCommand(intake, 0)).andThen(
    Commands.parallel(autopath5,
    Commands.waitSeconds(1).andThen(autoCenter()).andThen(autoCenter()).andThen(
    autoCenter()))).andThen(autoShoot()).andThen(
    Commands.parallel(autopath7, new Collect(intake,
    Constants.IntakeConstants.collectSpeed).withTimeout(2))).andThen(Commands.parallel(autopath75, autoCenter().andThen(autoCenter()))).andThen(autoShoot());
    }
   

  public Command getAutonomousCommand() { // Uses the program from Path planner to create an autonomous code
    return m_chooser.getSelected();

    /*
     * autoShoot().andThen(new RotateIntakeCommand(intake, 55))
     * .andThen(
     * Commands.parallel(autopath1, new Collect(intake,
     * Constants.IntakeConstants.collectSpeed).withTimeout(2)))
     * .andThen(new RotateIntakeCommand(intake, 0).withTimeout(2))
     * .andThen(autopath2).andThen(autoShoot()).andThen(new MoveLiftCommand(lift,
     * LiftConstants.kUnderChainPos));
     */
  }
}
