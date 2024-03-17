package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class ShooterStartWithSensorCmd extends Command {
    private final ShooterSubsystem shooter;
    private final TransferSubsystem transfer;

    public ShooterStartWithSensorCmd(ShooterSubsystem shooter, TransferSubsystem transfer) {
        this.shooter = shooter;
        this.transfer = transfer;
        addRequirements(shooter, transfer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.run();
        transfer.stop();
    }

    @Override
    public void end(boolean interrupted) {
        //shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
