package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPosition;

public class ShooterPositionCmd extends Command {
    private ShooterPosition shooterPositionSubsytem;

    public ShooterPositionCmd(ShooterPosition shooterPositionSubsytem) {
        this.shooterPositionSubsytem = shooterPositionSubsytem;
        addRequirements(shooterPositionSubsytem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterPositionSubsytem.ChangeAngle(10);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
