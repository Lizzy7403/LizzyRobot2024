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
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterSpeed(speed);
    }

    public void end(boolean interrupted) {
        shooter.stopShooter(); // Stop the shooter when the command ends
    }


    @Override
    public boolean isFinished() {
        shooter.stopShooter();
        return true; // This command completes immediately after setting the speed
    }
}
