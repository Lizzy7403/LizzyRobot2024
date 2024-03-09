package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
    private final Shooter shooter;
    private final double speed;


    //Constructor for the shootcommand class, requires a shooter and a speed as parameters
    public ShootCommand(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        //addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterSpeed(speed);
    }

          // The execute method is called repeatedly until the command ends
    // For this command, it checks if the intake has reached the setpoint
    // If the intake's position is within 5 units of the setpoint, the command is marked as finished
    @Override

   


    public void end(boolean interrupted) {

       shooter.stopShooter(); // Stop the intake roller when the command ends
    }

    // The isFinished method is called to determine when the command is finished
    // For this command, it is never finished on its own, it will run until it's explicitly interrupted

    @Override
    public boolean isFinished() {
        return false ; // This command will run until it's explicitly interrupted
    }


}
