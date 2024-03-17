package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class IntakeStartWithSensorCmd extends Command {
    private final Intake intake;
    private final ShooterSubsystem shooter;
    private final TransferSubsystem transfer;

    public IntakeStartWithSensorCmd(Intake intake, ShooterSubsystem shooter, TransferSubsystem transfer) {
        this.intake = intake;
        this.shooter = shooter;
        this.transfer = transfer;
        addRequirements(intake, shooter, transfer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.run();
        shooter.stop();
        transfer.stop();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
