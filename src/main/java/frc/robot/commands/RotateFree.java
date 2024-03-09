package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
// Importing the CommandBase class from the WPILib library
// This class provides the base for creating commands, which are actions that the robot can

// Importing the Intake class from the robot's code
// This class represents the intake subsystem of the robot
import frc.robot.subsystems.Intake;

// The SpinIntakeCommand class represents a command to spin the intake
public class RotateFree extends Command {

    // The intake subsystem that this command will operate on
    private final Intake intake;

    // The speed at which the intake should spin
    // This is a double value between -1.0 and 1.0
    private final double speed;


    // The constructor for the SpinIntakeCommand class
    // This is called when a SpinIntakeCommand object is created
    // The Intake object and the speed passed as parameters are the subsystem and the speed that the command will operate on
    public RotateFree(Intake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
    
        // This command requires the intake subsystem
        // This means that no other command that requires the intake subsystem can run at the same time as this command
        //addRequirements(this.intake);
    }

  

    // The initialize method is called once when the command is started
    // For this command, the intake starts spinning at the specified speed when the command is started
    @Override
    public void initialize() {

     intake.rotateFree(speed);


    }

    // The end method is called once when the command ends
    // For this command, the intake stops spinning when the command ends
    // The boolean parameter interrupted is true if the command ended because it was interrupted
    @Override
    public void end(boolean interrupted) {
        intake.rotateFree(0.0); // Stop the intake roller when the command ends
    }
}