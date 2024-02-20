package frc.robot.commands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ToggleLift extends Command {

    private static boolean isExtended = false;

    public ToggleLift (Lift lift){
        
        if (isExtended)
        { 
            lift.closeSolanoid();
            isExtended = false;
        }
        else
        {
            lift.openSolanoid();
            isExtended = true;
        }
    }

    public void robotPeriodic(){
        SmartDashboard.putBoolean("Solenoid Value ",isExtended);
    }

    public void end(boolean interrupted){

    } 
    public boolean isFinished()
    {
        return true;
    }

}
