// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;


public class ToggleSolenoid extends Command {

  private final Lift lift;


  /** Creates a new SolenoidUp. */
  public ToggleSolenoid(Lift lift) {
   addRequirements(lift);// here to declare subsystem dependencies.
   this.lift = lift;
   

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(lift.isSolenoidUp()){
      lift.openSolenoid();
    } else {
      lift.closeSolenoid();
    }

  }

  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(lift.isSolenoidUp()){
      lift.openSolenoid();
    } else {
      lift.closeSolenoid();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lift.setIsSolenoidUp(!lift.isSolenoidUp());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
