package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final static CANSparkMax motor = new CANSparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput sensor;
    private final RelativeEncoder encoder;
    private final SparkPIDController rotationController;
    
    public final String pickUpSpeedKey = "pick up speed";
    public double pickUpSpeedDefault = 0.8;

    public Intake() {
        Preferences.initDouble(pickUpSpeedKey, pickUpSpeedDefault);
        
        
        motor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
        motor.setIdleMode(IntakeConstants.IDLE_MODE);

        sensor = new DigitalInput(IntakeConstants.SENSOR_CHANNEL);

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(1);
        rotationController = motor.getPIDController();
        rotationController.setP(IntakeConstants.P);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Note", this.hasNote());
        SmartDashboard.putBoolean("Intaking", encoder.getVelocity() > 10);
        SmartDashboard.putNumber("intake amps", motor.getOutputCurrent());
    }

    public boolean hasNote() {
        return !sensor.get();
    }

    public static void getMotorAU() {
        SmartDashboard.putNumber("Ink. Motor Amp Usage", motor.getOutputCurrent());
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

    public void updateDashboard(){
        SmartDashboard.putNumber("intake RPM", encoder.getVelocity());
    }
}
