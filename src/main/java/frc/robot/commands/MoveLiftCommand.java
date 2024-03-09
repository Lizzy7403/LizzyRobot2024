package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;

public class MoveLiftCommand extends Command {

    private final Lift lift;

    private final double position;

    //Constructor for the MoveLiftCommand class
    public MoveLiftCommand(Lift lift, double position) { //requires a lift subsystem as a parameter
        this.lift = lift;
        this.position = position;
        addRequirements(this.lift);    
    }

   
     // The initialize method is called once when the command is started
    // For this command, the intake starts spinning at the specified speed when the command is started
    @Override
    public void initialize() {
        lift.setPosition(position);
    }

       // The execute method is called repeatedly until the command ends
    // For this command, it checks if the intake has reached the setpoint
    // If the intake's position is within 5 units of the setpoint, the command is marked as finished
    @Override
    public void execute() {
      
        }
    

 // The end method is called once when the command ends
    // For this command, the lift stops spinning when the command ends
    // The boolean parameter interrupted is true if the command ended because it was interrupted
    @Override
    public void end(boolean interrupted) {
        if(interrupted){
            lift.stopLift();
        }


    }

    @Override
    public boolean isFinished() {
        return true; // This command completes immediately after setting the speed
    }
}
