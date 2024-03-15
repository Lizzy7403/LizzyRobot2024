package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;

public class ToggleLift extends Command {

    //private static boolean isExtended = false;

    public ToggleLift (Lift lift){
        lift.closeSolenoid();

    }

    public void end(boolean interrupted){

    } 
    public boolean isFinished()
    {
        return false;
    }

}
