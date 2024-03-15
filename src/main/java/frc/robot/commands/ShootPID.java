package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;

public class ShootPID extends Command {

    private final Shooter shooter;

    private final double position;
  

    //Constructor for the MoveLiftCommand class
    public ShootPID(Shooter shooter, double position) { //requires a lift subsystem as a parameter
        this.shooter = shooter;
        this.position = position;
        addRequirements(this.shooter);    
    }

   
     // The initialize method is called once when the command is started
    // For this command, the intake starts spinning at the specified speed when the command is started
    @Override
    public void initialize() {
        shooter.setPosition(position);
    }

       // Theexecute method is called repeatedly until the command ends
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
            shooter.stopShooter();
         //   shooter.resetShooterEncoder();
        }


    }

    @Override
    public boolean isFinished() {
        if(Math.abs(shooter.getShooterPosition()-position)<3){
            
            return true;

        }
        return false;

}
}

