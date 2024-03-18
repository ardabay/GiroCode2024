package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends SubsystemBase {

    private final CANSparkMax leftMotor = new CANSparkMax(8,
            MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(3,
            MotorType.kBrushless);

    double speed = 0.00;

    public TransferSubsystem() {

    }

    public void run() {

        leftMotor.set(-speed);
        rightMotor.set(-speed);
        if (speed < 1) {
            speed += 0.01;
        }

    }

    public void stop() {
        leftMotor.set(-speed);
        rightMotor.set(-speed);
        if (speed > 0) {
          speed -= 0.01;
        }
    }
}
