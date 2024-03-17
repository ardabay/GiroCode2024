package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterPosition extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(32, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(33, MotorType.kBrushless);

    private RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightMotor.getEncoder();

    private double leftEncoderfirstPosition = 0;
    private double rightEncoderfirstPosition = 0;

    private final PIDController PidController;

    public ShooterPosition() {
        leftEncoder.setPositionConversionFactor(Constants.ShooterPositionConstans.kAngleEncoderRot2Degrees);
        rightEncoder.setPositionConversionFactor(Constants.ShooterPositionConstans.kAngleEncoderRot2Degrees);
        leftEncoderfirstPosition = leftEncoder.getPosition();
        rightEncoderfirstPosition = rightEncoder.getPosition();
        SmartDashboard.putNumber("Left Encoder Value", leftEncoderfirstPosition);
        SmartDashboard.putNumber("Right Encoder Value", rightEncoderfirstPosition);

        PidController = new PIDController(0.01, 0, 0);
        PidController.enableContinuousInput(-180, 180);

    }

    public void ChangeAngle(double angle) {

        double leftPosition = -leftEncoder.getPosition() + leftEncoderfirstPosition;
        double rightPosition = rightEncoder.getPosition() - rightEncoderfirstPosition;

        /*leftEncoder.setPosition(-(angle + leftEncoder.getPosition() - leftEncoderfirstPosition));
        rightEncoder.setPosition(angle + rightEncoder.getPosition() - rightEncoderfirstPosition);*/


        leftMotor.set(-PidController.calculate(leftPosition, angle));
        rightMotor.set(PidController.calculate(rightPosition, angle));

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Encoder Value", leftEncoder.getPosition() - leftEncoderfirstPosition);
        SmartDashboard.putNumber("Right Encoder Value", rightEncoder.getPosition() - rightEncoderfirstPosition);
    }
}