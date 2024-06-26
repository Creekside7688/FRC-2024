package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.ModuleConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    private final SparkPIDController drivePIDController;
    private final SparkPIDController turnPIDController;

    private double angularOffset = 0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int driveMotor, int turnMotor, double angularOffset) {
        this.driveMotor = new CANSparkMax(driveMotor, MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotor, MotorType.kBrushless);

        this.driveMotor.restoreFactoryDefaults();
        this.turnMotor.restoreFactoryDefaults();

        this.driveEncoder = this.driveMotor.getEncoder();
        this.turnEncoder = this.turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

        this.drivePIDController = this.driveMotor.getPIDController();
        this.turnPIDController = this.turnMotor.getPIDController();
        this.drivePIDController.setFeedbackDevice(driveEncoder);
        this.turnPIDController.setFeedbackDevice(turnEncoder);

        this.driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_POSITION_FACTOR);
        this.driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_VELOCITY_FACTOR);

        this.turnEncoder.setPositionConversionFactor(ModuleConstants.TURN_ENCODER_POSITION_FACTOR);
        this.turnEncoder.setVelocityConversionFactor(ModuleConstants.TURN_ENCODER_VELOCITY_FACTOR);

        // Invert the encoder because the output shaft rotates the opposite direction in the modules.
        this.turnEncoder.setInverted(ModuleConstants.TURN_ENCODER_INVERTED);

        /*
         * Enable PID wrap around for the turning motor. This will allow the PID controller to go through 0 to get to the setpoint i.e. going from 350 degrees to 10 degrees will go
         * through 0 rather than the other direction which is a longer route.
         */
        this.turnPIDController.setPositionPIDWrappingEnabled(true);
        this.turnPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURN_PID_MINIMUM_INPUT);
        this.turnPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURN_PID_MAXIMUM_INPUT);

        this.drivePIDController.setP(ModuleConstants.DRIVE_P);
        this.drivePIDController.setI(ModuleConstants.DRIVE_I);
        this.drivePIDController.setD(ModuleConstants.DRIVE_D);
        this.drivePIDController.setFF(ModuleConstants.DRIVE_FF);
        this.drivePIDController.setOutputRange(ModuleConstants.DRIVE_MINIMUM_OUTPUT, ModuleConstants.DRIVE_MAXIMUM_OUTPUT);

        this.turnPIDController.setP(ModuleConstants.TURN_P);
        this.turnPIDController.setI(ModuleConstants.TURN_I);
        this.turnPIDController.setD(ModuleConstants.TURN_D);
        this.turnPIDController.setFF(ModuleConstants.TURN_FF);
        this.turnPIDController.setOutputRange(ModuleConstants.TURN_MINIMUM_OUTPUT, ModuleConstants.TURN_MAXIMUM_OUTPUT);

        this.driveMotor.setIdleMode(ModuleConstants.DRIVE_IDLE_MODE);
        this.turnMotor.setIdleMode(ModuleConstants.TURN_IDLE_MODE);
        this.driveMotor.setSmartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
        this.turnMotor.setSmartCurrentLimit(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        this.driveMotor.burnFlash();
        this.turnMotor.burnFlash();

        // Set the angular offset of the module.
        this.angularOffset = angularOffset;
        this.desiredState.angle = new Rotation2d(turnEncoder.getPosition());
        this.driveEncoder.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            new Rotation2d(turnEncoder.getPosition() - angularOffset)
        );
    }

    /**
     * Returns the current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            new Rotation2d(turnEncoder.getPosition() - angularOffset)
        );
    }

    /**
     * Returns the desired state of the module.
     * @return The desired state of the module.
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;

    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedState = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle.plus(Rotation2d.fromRadians(angularOffset))
        );

        // Optimize to prevent having to turn more than 90 degrees.
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            correctedState,
            new Rotation2d(turnEncoder.getPosition())
        );

        // Drive towards setpoints.
        drivePIDController.setReference(
            optimizedState.speedMetersPerSecond,
            CANSparkMax.ControlType.kVelocity
        );

        turnPIDController.setReference(
            optimizedState.angle.getRadians(),
            CANSparkMax.ControlType.kPosition
        );

        this.desiredState = desiredState;
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }
}
