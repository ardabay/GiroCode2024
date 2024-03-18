package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class TestCmd extends Command {
    private ShooterSubsystem shooter;
    private TransferSubsystem transfer;
    private Intake intake;

    public TestCmd(ShooterSubsystem shooter, TransferSubsystem transfer, Intake intake) {
        this.shooter = shooter;
        this.transfer = transfer;
        this.intake = intake;
        addRequirements(shooter, transfer, intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.run();
        transfer.run();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        transfer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
