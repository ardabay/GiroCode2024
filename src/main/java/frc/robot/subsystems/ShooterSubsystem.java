package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(20, MotorType.kBrushless);
  double speed = 0.0;

  public ShooterSubsystem() {

  }

  public void run() {
    leftMotor.set(speed);
    rightMotor.set(-speed);
    if (speed < 1) {
      speed += 0.01;
    }
  }

  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }
}
