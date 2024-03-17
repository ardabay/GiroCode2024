package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransferSubsystem;

public class TransferContinueWithDriverCmd extends Command {
    private final TransferSubsystem transfer;

    public TransferContinueWithDriverCmd(TransferSubsystem transfer) {
        this.transfer = transfer;
        addRequirements(transfer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        transfer.run();
    }

    @Override
    public void end(boolean interrupted) {
        transfer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
