package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPosition;

public class ShooterPositionCmd extends Command {
    private ShooterPosition shooterPositionSubsytem;
    private double angle;

    public ShooterPositionCmd(ShooterPosition shooterPositionSubsytem, double angle) {
        this.shooterPositionSubsytem = shooterPositionSubsytem;
        this.angle = angle;
        addRequirements(shooterPositionSubsytem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterPositionSubsytem.ChangeAngle(angle);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
