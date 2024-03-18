package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final CANSparkMax topMotor = new CANSparkMax(22, MotorType.kBrushless); // Burayı
                                                                                                                        // Unutma
    private final CANSparkMax bottomMotor = new CANSparkMax(21,
            MotorType.kBrushless); // Burayı Unutma
    double speed = 0.00;

    public Intake() {

    }

    public void run() {
        SmartDashboard.putString("Intake State", "Intake.");
        topMotor.set(speed);
        bottomMotor.set(speed);
        if (speed < 0.60) {
            speed += 0.01;
        }
    }

    public void stop() {
        topMotor.set(speed);
        bottomMotor.set(speed);
        if (speed > 0) {
          speed -= 0.01;
        }
    }

}
