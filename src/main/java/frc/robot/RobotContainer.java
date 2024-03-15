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
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  ////////////////////////////////////////////// AUTO INSTANCE VARIABLES.

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private Command PathBlueLeft1 = drivetrain.getAutoPath("PathBlueLeft1");
    private Command PathBlueLeft2 = drivetrain.getAutoPath("PathBlueLeft2");


 



  private void configureBindings() 
  {
    
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));


  //  JoystickButton brakeSwerve = new JoystickButton(joystick, PS4Controller.Button.kCircle.value);
  //  brakeSwerve.whileTrue(drivetrain.applyRequest(() -> brake));

   JoystickButton pointToDirection = new JoystickButton(joystick, PS4Controller.Button.kPS.value);
   pointToDirection.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

   
   // SHOOTING BUTTONS

       JoystickButton shootManual = new JoystickButton(joystick,PS4Controller.Button.kCross.value);
       shootManual.whileTrue(new ShootCommand(shooter, -0.1));

   //calls a  parallel command to start first the shooter and a few seconds later the feeder
   JoystickButton shootHigh = new JoystickButton(joystick,PS4Controller.Button.kCircle.value); 
    shootHigh.onTrue(
    (Commands.parallel(
      Commands.waitSeconds(3).asProxy().andThen(new Feed(intake,1).withTimeout(1)),
      new ShootCommand(shooter, Constants.ShooterConstants.kMaxAbsOutputRBHigh).withTimeout(5.5),
      new MoveLiftCommand(lift, Constants.LiftConstants.kShootPos)
     )).andThen(new MoveLiftCommand(lift, LiftConstants.kUnderChainPos)));
     // new MoveLiftCommand(lift, Constants.LiftConstants.kShootPos)


  // shootHigh.whileTrue(new ShootCommand(shooter, Constants.ShooterConstants.kMaxAbsOutputRBHigh));
///////////////////////////////////////////////////////
   
JoystickButton shootLow = new JoystickButton(joystick,PS4Controller.Button.kCross.value);
    //shootLow.onTrue(Commands.parallel(Commands.waitSeconds(0.13).asProxy().andThen(new Feed(intake,1).withTimeout(1)), new RotateIntakeCommand(intake,35,1)));

    //shootLow.onTrue(Commands.parallel(Commands.waitSeconds(1.6).asProxy().andThen(new Feed(intake,1).withTimeout(1)),
   // new RotateIntakeCommand(intake, 40 )));//Calls the shootCommand with a speed parameter that makes it shoot low

   //shootLow.onTrue(Commands.parallel(Commands.waitSeconds(2).asProxy().andThen(new Feed(intake,1).withTimeout(1)), new RotateIntakeCommand(intake,22)));
   //shootLow.onTrue(Commands.parallel(Commands.waitSeconds(0.13).asProxy().andThen(new Feed(intake,1).withTimeout(1)), new RotateIntakeCommand(intake,35,1)));
/*shootLow.onTrue(new MoveLiftCommand(lift, -420)
.andThen(Commands.parallel(new ShootCommand(shooter, -0.2).withTimeout(0.7),new Feed(intake, 0.8).withTimeout(2))
.andThen(new MoveLiftCommand(lift, -480)).andThen(new SolenoidUp(lift).withTimeout(.9))
.andThen(new SolenoidDown(lift).withTimeout(0.2))).andThen(new MoveLiftCommand(lift,0)));
*/

shootLow.whileTrue(Commands.parallel(new ShootCommand(shooter, -0.2),Commands.waitSeconds(0.5).andThen(new Feed(intake,1))));
    
//INTAKE BUTTON)

    JoystickButton collectNote = new JoystickButton(joystick , PS4Controller.Button.kR1.value);
    collectNote.whileTrue(new Collect(intake, -Constants.IntakeConstants.collectSpeed));
    
    JoystickButton releaseNote = new JoystickButton(joystick , PS4Controller.Button.kL1.value);
    releaseNote.whileTrue(new Feed(intake, Constants.IntakeConstants.releaseSpeed));


    JoystickButton rotateIn = new JoystickButton(joystick , PS4Controller.Button.kL2.value);
    //rotateFree.whileTrue(new RotateFree(intake, Constants.IntakeConstants.kMaxAbsOutputRBExtended));
    rotateIn.onTrue(new RotateIntakeCommand(intake, 2));


    JoystickButton rotateOut = new JoystickButton(joystick , PS4Controller.Button.kR2.value);
    //rotateInverse.whileTrue(new RotateFree(intake, -Constants.IntakeConstants.kMaxAbsOutputRBRetracted));
     rotateOut.onTrue(new RotateIntakeCommand(intake, 55));

     //JoystickButton toggleIntake = new JoystickButton(joystick, PS4Controller.Button.kTriangle.value);

     //toggleIntake.onTrue(new ToggleIntakeCommand(intake, 48, 0));

     

    // LIFT BUTTONS

    JoystickButton moveLiftUp = new JoystickButton(joystick , PS4Controller.Button.kOptions.value);
    moveLiftUp.whileTrue(new MoveLift(lift, Constants.LiftConstants.speed));

     JoystickButton moveLiftDown = new JoystickButton(joystick , PS4Controller.Button.kShare.value);
    moveLiftDown.whileTrue(new MoveLift(lift, -Constants.LiftConstants.speed));

    //JoystickButton closeSoleButton = new JoystickButton(joystick, PS4Controller.Button.kL3.value);
    
    //closeSoleButton.onTrue(new SolenoidUp(lift));

    //JoystickButton openSoleButton = new JoystickButton(joystick, PS4Controller.Button.kR3.value);
    //openSoleButton.onTrue(new SolenoidDown(lift));

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
    intake.resetRotateEncoder();
    lift.resetLiftEncoder();
    shooter.resetShooterEncoder();

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Auto1", kCustomAuto);
    m_chooser.addOption("Auto2", kCustomAuto);
    m_chooser.addOption("Auto3", kCustomAuto);
    m_chooser.addOption("Auto4", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

  


    configureBindings();
  }

  public Command getAutonomousCommand() { //Uses the program from Path planner to create an autonomous code
    return Commands.print("No autonomous command configured");
  }
}
