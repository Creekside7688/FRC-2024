package frc.robot.swerve;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.SimCameraProperties;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.SwerveUtils;
import frc.robot.auto.PhotonRunnable;
import frc.robot.constants.AutonomousConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.FL_DRIVE_MOTOR,
        DriveConstants.FL_TURN_MOTOR,
        DriveConstants.FL_OFFSET
    );

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.FR_DRIVE_MOTOR,
        DriveConstants.FR_TURN_MOTOR,
        DriveConstants.FR_OFFSET
    );

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.BL_DRIVE_MOTOR,
        DriveConstants.BL_TURN_MOTOR,
        DriveConstants.BL_OFFSET
    );

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.BR_DRIVE_MOTOR,
        DriveConstants.BR_TURN_MOTOR,
        DriveConstants.BR_OFFSET
    );

    // Gyro
    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB1);

    /*
     * Kalman Filter Configuration. These can be "tuned-to-taste" based on how much you trust your various sensors. Smaller numbers will cause the
     * filter to "trust" the estimate from that particular component more than the others. This in turn means the particular component will have a
     * stronger influence on the final pose estimate.
     */

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This matrix is in the form [x, y,
     * theta]ᵀ, with units in meters and radians, then meters.
     */
    private static final Vector<N3> stateDeviations = VecBuilder.fill(0.1, 0.1, 0.1);

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision less. This matrix is in the
     * form [x, y, theta]ᵀ, with units in meters and radians.
     */
    private static final Vector<N3> visionMeasurementDeviations = VecBuilder.fill(1.5, 1.5, 1.5);

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field = new Field2d();

    private OriginPosition originPosition = kBlueAllianceWallRightSide;
    private boolean sawTag = false;

    private double currentRotation = 0.0;
    private double currentTranslationDirection = 0.0;
    private double currentTranslationMagnitude = 0.0;

    // Slew Rate filters to control acceleration.
    private SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(DriveConstants.MAGNITUDE_SLEW_RATE);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(DriveConstants.ROTATION_SLEW_RATE);

    private double previousTime = WPIUtilJNI.now() * 1e-6;

    public SwerveDrive() {
        this.zeroHeading();


        this.poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.SWERVE_KINEMATICS,
            this.getRotation2d(),
            this.getModulePositions(),
            new Pose2d(),
            stateDeviations,
            visionMeasurementDeviations
        );

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            this::driveRelative,
            AutonomousConstants.pathFollowConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }

                return false;
            },
            this
        );

        SimCameraProperties cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(75.76079874010732));

        SmartDashboard.putData("Swerve", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", () -> frontRight.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", () -> backLeft.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle", () -> backRight.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> backRight.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", () -> getRotation2d().getRadians(), null);
            }
        });
    }

    @Override
    public void periodic() {

        // Update pose estimator with encoder data
        poseEstimator.update(this.getRotation2d(), this.getModulePositions());

        // Get the latest vision pose

        // If the vision pose has changed

        // Add the pose to the dashboard
        Pose2d dashboardPose = poseEstimator.getEstimatedPosition();

        if(originPosition == kRedAllianceWallRightSide) {
            dashboardPose = flipAlliance(dashboardPose);
        }

        // Update the pose on the field.
        field.setRobotPose(dashboardPose);

        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("Heading", this.getRotation2d().getDegrees());
    }

    /**
     * Drives the robot using controller input.
     */
    public void drive(double xSpeed, double ySpeed, double rSpeed, boolean limitSpeed, boolean fieldRelative, boolean rateLimit) {

        // Cube the inputs for fine control at low speeds.
        xSpeed = Math.pow(xSpeed, 3);
        ySpeed = Math.pow(ySpeed, 3);
        rSpeed = Math.pow(rSpeed, 3);

        double xSpeedCommand;
        double ySpeedCommand;

        // If we want to ratelimit
        if(rateLimit) {
            // Get the elapsed time since the last period
            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - previousTime;

            // Convert the inputs to polar coordinates
            double inputTranslationDirection = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMagnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            double directionSlewRate;

            // If we are moving
            if(currentTranslationMagnitude != 0.0) {
                // Apply a slew rate.
                directionSlewRate = Math.abs(DriveConstants.DIRECTION_SLEW_RATE / currentTranslationMagnitude);

                // If we are not moving
            } else {
                // Set an infinite slew rate
                directionSlewRate = 500.0;
            }

            // Find the minimum difference between the input and current direction
            double angleDif = SwerveUtils.angleDifference(inputTranslationDirection, currentTranslationDirection);

            // If the difference is less than 0.45 radians
            if(angleDif < (0.45 * Math.PI)) {
                // Step towards the input direction
                currentTranslationDirection = SwerveUtils
                    .stepTowardsCircular(currentTranslationDirection, inputTranslationDirection, directionSlewRate * elapsedTime);

                // Limit the magnitude
                currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);

                // If the difference is greater than 0.85 radians
            } else if(angleDif > 0.85 * Math.PI) {
                // If the robot is moving
                if(currentTranslationMagnitude > 1e-4) {
                    // Remove the magnitude
                    currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);

                    // Otherwise
                } else {
                    // Wrap the angle and calcualte
                    currentTranslationDirection = SwerveUtils.wrapAngle(currentTranslationDirection + Math.PI);
                    currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
                }

                // Otherwise
            } else {
                // Step towards the input direction, but remove the magnitude.
                currentTranslationDirection = SwerveUtils
                    .stepTowardsCircular(currentTranslationDirection, inputTranslationDirection, directionSlewRate * elapsedTime);
                currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
            }

            previousTime = currentTime;

            // Calculate the commanded speeds
            xSpeedCommand = currentTranslationMagnitude * Math.cos(currentTranslationDirection);
            ySpeedCommand = currentTranslationMagnitude * Math.sin(currentTranslationDirection);
            currentRotation = rotationLimiter.calculate(rSpeed);

        } else {
            xSpeedCommand = xSpeed;
            ySpeedCommand = ySpeed;
            currentRotation = rSpeed;
        }

        // Convert the speeds into percentages of the maximum speed.
        double xSpeedDelivered = xSpeedCommand
            * (limitSpeed ? DriveConstants.MAXIMUM_LIMITED_SPEED_METRES_PER_SECOND : DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);

        double ySpeedDelivered = ySpeedCommand
            * (limitSpeed ? DriveConstants.MAXIMUM_LIMITED_SPEED_METRES_PER_SECOND : DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);

        double rotDelivered = currentRotation * (limitSpeed ? DriveConstants.MAXIMUM_LIMITED_ANGULAR_SPEED_RADIANS_PER_SECOND
            : DriveConstants.MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND);

        // Calculate the desired module states based on if we are driving field relative or not.
        SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, this.getRotation2d())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
        );

        // Desaturate the wheel speeds to prevent any speeds from exceeding the maximum.
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Drives the robot using field relative chassis speeds.
     */
    public void driveRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement. Use for defense or when the robot needs to be stationary.
     */
    public void lockPosition() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Returns the current state of the swerve drive in the form of a chassis speeds object.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(
            new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
            }
        );
    }

    /**
     * Returns the current state of the swerve drive in the form of a swerve module state array.
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    /**
     * Returns the estimated position.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the current rotation of the robot.
     */
    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    /**
     * Returns how fast the robot is turning in degrees per second.
     */
    public double getTurnRate() {
        return gyro.getRate() * (DriveConstants.GYRO_INVERTED ? -1.0 : 1.0);
    }

    /**
     * Returns a formatted string of the current pose of the robot.
     */
    public String getFomattedPose() {
        Pose2d pose = this.getPose();

        return String.format(
            "(%.3f, %.3f) %.2f degrees",
            pose.getX(),
            pose.getY(),
            pose.getRotation().getDegrees()
        );
    }

    /**
     * Resets odometry to a specified pose.
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(this.getRotation2d(), this.getModulePositions(), pose);
    }

    /**
     * Resets the position on the field to 0, 0, 0-degrees, with forward being downfield. This resets what "forward" is for field oriented driving.
     */
    public void resetPose() {
        this.setPose(new Pose2d());
    }

    /**
     * Sets the module states of the swerve drive.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        backLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
    }

    /**
     * Zeros the heading of the robot.
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Transforms a pose to the opposite alliance's coordinate system. (0,0) is always on the right corner of your alliance wall, so for 2023, the
     * field elements are at different coordinates for each alliance.
     * 
     * @param pose pose to transform to the other alliance
     * @return pose relative to the other alliance's coordinate system
     */
    private Pose2d flipAlliance(Pose2d pose) {
        return pose.relativeTo(VisionConstants.FLIPPING_POSE);
    }

    /**
     * Sets the alliance. This is used to configure the origin of the AprilTag map
     * 
     * @param alliance alliance
     */
    public void setAlliance(Alliance alliance) {
        boolean allianceChanged = false;

        switch(alliance) {
            case Blue:
                allianceChanged = (originPosition == kRedAllianceWallRightSide);
                originPosition = kBlueAllianceWallRightSide;
                break;
            case Red:
                allianceChanged = (originPosition == kBlueAllianceWallRightSide);
                originPosition = kRedAllianceWallRightSide;
                break;
            default:
                // Something went wrong
        }

        // If the alliance was changed and we saw a tag
        if(allianceChanged && sawTag) {
            // Flip the pose
            Pose2d newPose = flipAlliance(this.getPose());
            poseEstimator.resetPosition(this.getRotation2d(), this.getModulePositions(), newPose);
        }
    }

    public void addDashboardWidgets(ShuffleboardTab tab) {
        tab.add("Field", field).withPosition(0, 0).withSize(6, 4);
        tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
    }
}
