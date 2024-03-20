// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;


public class SolenoidFinal extends Command {

  private final Lift lift;


  /** Creates a new SolenoidUp. */
  public SolenoidFinal(Lift lift) {
   addRequirements(lift);// here to declare subsystem dependencies.
   this.lift = lift;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    lift.moveFinalSolenoid();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    lift.moveFinalSolenoid();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
