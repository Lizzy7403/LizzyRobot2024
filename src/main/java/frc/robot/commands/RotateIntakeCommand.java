package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RotateIntakeCommand extends Command {

    private final Intake intake;
    private final double setpoint;
    private final boolean isExtended;
    
    // PID constants
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kIz;
    private final double kFF;

    private boolean isFinished = false;

    public RotateIntakeCommand(Intake intake, double setpoint, boolean isExtended, double kP, double kI, double kD, double kIz, double kFF) {
        this.intake = intake;
        this.setpoint = setpoint;
        this.isExtended = isExtended;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kIz = kIz;
        this.kFF = kFF;
    }

    // The initialize method is called once when the command is started
    // For this command, the intake starts rotating to the specified setpoint when the command is started
    @Override
    public void initialize() {
        intake.rotateIntakeWithPID(setpoint, kP, kI, kD, kIz, kFF);
    }

    // The execute method is called repeatedly until the command ends
    // For this command, it checks if the intake has reached the setpoint
    // If the intake's position is within 5 units of the setpoint, the command is marked as finished
    @Override
    public void execute() {
        if (Math.abs(intake.getRotateEncoderPosition() - setpoint) < 1) {
            if(isExtended){
                intake.resetRotateEncoder();
            }
            isFinished = true;
        }
    }

    // The end method is called once when the command ends
    // For this command, if the command was interrupted, it stops the intake from rotating
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            intake.stopRotateIntake(); // Optionally stop the rotation if the command is interrupted
        }
    }

    // The isFinished method is called to determine when the command is finished
    // For this command, it is finished when the intake has reached the setpoint
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}