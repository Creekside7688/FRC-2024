package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);

    private final DigitalInput sensor = new DigitalInput(IntakeConstants.SENSOR_CHANNEL);
    private final RelativeEncoder encoder = motor.getEncoder();

    private final SparkPIDController rotationController = motor.getPIDController();

    public Intake() {
        motor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
        motor.setIdleMode(IntakeConstants.IDLE_MODE);

        rotationController.setP(IntakeConstants.P);
    }

    @Override
    public void periodic() {
    }

    public boolean getSensor() {
        return sensor.get();
    }

    public boolean isNotePresent() {
        return !getSensor();
    }

    public double getRPM() {
        return encoder.getVelocity() / IntakeConstants.INTAKE_GEAR_RATIO;
    }

    public void setRPM(double rpm) {
        rotationController.setReference(rpm, ControlType.kVelocity);
    }

    public void run(double speed) {
        motor.set(speed);
    }
}
