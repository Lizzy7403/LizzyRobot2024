// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  private final Intake intake =new Intake();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final PS4Controller joystick = new PS4Controller(Constants.PS4GamePad.joystickPort);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private double MaxSpeed = Constants.Swerve.maxSpeedMperS; // 6 meters per second desired top speed
  private double MaxAngularRate = Constants.Swerve.maxAngularRate * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  ////////////////////////////////////////////// AUTO CONFIGURATION ./////////////////////

   //private String m_autoSelected;
   //private SendableChooser<Command> m_chooser = new SendableChooser<>();

    private Command autopath1 = drivetrain.getAutoPath("autopath1");
   private Command autopath2 = drivetrain.getAutoPath("autopath2");
   // private Command autopath3 = drivetrain.getAutoPath("autopath3");
    //private Command autopath4 = drivetrain.getAutoPath("autopath4");
    //private Command autopath5 = drivetrain.getAutoPath("autopath5");
    //private Command autopath6 = drivetrain.getAutoPath("autopath6");

    private Command shootSp = Commands.parallel(
        Commands.waitSeconds(1).asProxy().andThen(new Feed(intake,1).withTimeout(2)),
        new ShootCommand(shooter, Constants.ShooterConstants.kMaxAbsOutputRBHigh).withTimeout(3)
      );

    /*private Command blueAutoLeft = shootSp.andThen(new RotateIntakeCommand(intake, 55))
    .andThen(Commands.parallel(autopath1,new Collect(intake,Constants.IntakeConstants.collectSpeed).withTimeout(3)))
    .andThen(new RotateIntakeCommand(intake, 0).withTimeout(2))
    .andThen(autopath2).andThen(shootSp).andThen(Commands.parallel(autopath3,new Collect(intake,Constants.IntakeConstants.collectSpeed).withTimeout(3)));*/

    /*private Command blueAutoCenter = shootSp.andThen(new RotateIntakeCommand(intake, 55))
    .andThen(Commands.parallel(autopath4,new Collect(intake,Constants.IntakeConstants.collectSpeed).withTimeout(3)))
    .andThen(new RotateIntakeCommand(intake, 0).withTimeout(2))
    .andThen(autopath5).andThen(shootSp).andThen(Commands.parallel(autopath3,new Collect(intake,Constants.IntakeConstants.collectSpeed).withTimeout(3)));*/

    /*private  Command blueAutoRight = shootSp.andThen(new RotateIntakeCommand(intake, 55))
    .andThen(Commands.parallel(autopath4,new Collect(intake,Constants.IntakeConstants.collectSpeed).withTimeout(3)))
    .andThen(new RotateIntakeCommand(intake, 0).withTimeout(2))
    .andThen(autopath5).andThen(shootSp)6.andThen(Commands.parallel(autopath3,new Collect(intake,Constants.IntakeConstants.collectSpeed).withTimeout(3)));*/


  private void configureBindings() 
  {
    
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));


   JoystickButton pointToDirection = new JoystickButton(joystick, PS4Controller.Button.kPS.value);
   pointToDirection.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

   
   // SHOOTING BUTTONS

       JoystickButton shootManual = new JoystickButton(joystick,PS4Controller.Button.kCross.value);
       shootManual.whileTrue(new ShootCommand(shooter, -0.1));

   //calls a  parallel command to start first the shooter and a few seconds later the feeder
   JoystickButton shootHigh = new JoystickButton(joystick,PS4Controller.Button.kCircle.value); 
    shootHigh.onTrue( Commands.parallel(
        Commands.waitSeconds(4).asProxy().andThen(new Feed(intake,1).withTimeout(1)),
        new ShootCommand(shooter, Constants.ShooterConstants.kMaxAbsOutputRBHigh).withTimeout(5.5),
        new MoveLiftCommand(lift, Constants.LiftConstants.kShootPos)
      ).andThen(new MoveLiftCommand(lift, LiftConstants.kUnderChainPos)));
    
 
   
JoystickButton shootLow = new JoystickButton(joystick,PS4Controller.Button.kCross.value);
shootLow.whileTrue(Commands.parallel(new ShootCommand(shooter, -0.2).withTimeout(1),Commands.waitSeconds(0.5).andThen(new Feed(intake,1))));
    
//INTAKE BUTTON)

    JoystickButton collectNote = new JoystickButton(joystick , PS4Controller.Button.kR1.value);
    collectNote.whileTrue(new Collect(intake, Constants.IntakeConstants.collectSpeed));
    
    JoystickButton releaseNote = new JoystickButton(joystick , PS4Controller.Button.kL1.value);
    releaseNote.whileTrue(new Feed(intake, Constants.IntakeConstants.releaseSpeed));


    JoystickButton rotateIn = new JoystickButton(joystick , PS4Controller.Button.kL2.value);
    //rotateFree.whileTrue(new RotateFree(intake, Constants.IntakeConstants.kMaxAbsOutputRBExtended));
    rotateIn.onTrue(new RotateIntakeCommand(intake, 2));


    JoystickButton rotateOut = new JoystickButton(joystick , PS4Controller.Button.kR2.value);
     rotateOut.onTrue(new RotateIntakeCommand(intake, 55));

    // LIFT BUTTONS

    JoystickButton moveLiftUp = new JoystickButton(joystick , PS4Controller.Button.kOptions.value);
    moveLiftUp.whileTrue(new MoveLift(lift, Constants.LiftConstants.speed));

     JoystickButton moveLiftDown = new JoystickButton(joystick , PS4Controller.Button.kShare.value);
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


  public RobotContainer() 
  {
    lift.setLiftEncoder(Constants.LiftConstants.kShootPos);
    intake.resetRotateEncoder();
    shooter.resetShooterEncoder();
   
    //AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Sendable Chooser", m_chooser);
   

    configureBindings();
  }

  public Command getAutonomousCommand() { //Uses the program from Path planner to create an autonomous code
    
    return shootSp.andThen(new RotateIntakeCommand(intake, 55))
    .andThen(Commands.parallel(autopath1,new Collect(intake,Constants.IntakeConstants.collectSpeed).withTimeout(2)))
    .andThen(new RotateIntakeCommand(intake, 0).withTimeout(2))
    .andThen(autopath2).andThen(Commands.parallel(
        Commands.waitSeconds(1).asProxy().andThen(new Feed(intake,1).withTimeout(2)),
        new ShootCommand(shooter, Constants.ShooterConstants.kMaxAbsOutputRBHigh).withTimeout(3)
      ).andThen(new MoveLiftCommand(lift, LiftConstants.kUnderChainPos)));
}
}
