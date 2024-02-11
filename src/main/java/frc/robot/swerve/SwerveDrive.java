package frc.robot.swerve;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import org.photonvision.EstimatedRobotPose;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others.
    // This in turn means the particualr component will have a stronger influence
    // on the final pose estimate.

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians, then meters.
     */
    private static final Vector<N3> stateDevs = VecBuilder.fill(0.1, 0.1, 0.1);

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision less. This matrix is in the form [x, y, theta]ᵀ, with units
     * in meters and radians.
     */
    private static final Vector<N3> visionMeasurementDevs = VecBuilder.fill(1.5, 1.5, 1.5);

    private final SwerveDrivePoseEstimator poseEstimator;
    private final PhotonRunnable photonEstimator;

    private final Field2d field2d = new Field2d();
    private final Notifier photonNotifier;

    private OriginPosition originPosition = kBlueAllianceWallRightSide;
    private boolean sawTag = false;

    // Gyro.
    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    private double currentRotation = 0.0;
    private double currentTranslationDirection = 0.0;
    private double currentTranslationMagnitude = 0.0;

    // Slew Rate filters to control acceleration.
    private SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(DriveConstants.MAGNITUDE_SLEW_RATE);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(DriveConstants.ROTATION_SLEW_RATE);

    private double previousTime = WPIUtilJNI.now() * 1e-6;

    public SwerveDrive(PhotonRunnable photonEstimator) {
        this.zeroHeading();

        this.photonEstimator = photonEstimator;
        this.photonNotifier = new Notifier(this.photonEstimator);

        this.poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.SWERVE_KINEMATICS,
            this.getRotation2d(),
            this.getModulePositions(),
            new Pose2d(),
            stateDevs,
            visionMeasurementDevs
        );

        photonNotifier.setName("PhotonRunnable");
        photonNotifier.startPeriodic(0.02);

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            this::driveRelative,
            AutonomousConstants.pathFollowConfig,
            () -> false,
            this
        );
    }

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors
        poseEstimator.update(this.getRotation2d(), this.getModulePositions());

        EstimatedRobotPose visionPose = photonEstimator.grabLatestEstimatedPose();
        if(visionPose != null) {
            // New pose from vision
            sawTag = true;
            Pose2d pose2d = visionPose.estimatedPose.toPose2d();

            if(originPosition != kBlueAllianceWallRightSide) {
                pose2d = flipAlliance(pose2d);
            }

            poseEstimator.addVisionMeasurement(pose2d, visionPose.timestampSeconds);
        }

        // Set the pose on the dashboard
        Pose2d dashboardPose = poseEstimator.getEstimatedPosition();

        if(originPosition == kRedAllianceWallRightSide) {
            // Flip the pose when red, since the dashboard field photo cannot be rotated
            dashboardPose = flipAlliance(dashboardPose);
        }

        field2d.setRobotPose(dashboardPose);
    }

    /**
     * Returns the estimated position.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

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
     * Resets odometry to a specified pose.
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(this.getRotation2d(), this.getModulePositions(), pose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets what "forward" is for field oriented driving.
     */
    public void resetPose() {
        this.setPose(new Pose2d());
    }

    public void drive(double xSpeed, double ySpeed, double rSpeed, boolean fieldRelative, boolean rateLimit) {
        xSpeed = Math.pow(xSpeed, 3);
        ySpeed = Math.pow(ySpeed, 3);
        rSpeed = Math.pow(rSpeed, 3);

        double xSpeedCommand;
        double ySpeedCommand;

        if(rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDirection = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMagnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate based on acceleration estimate
            double directionSlewRate;

            if(currentTranslationMagnitude != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.DIRECTION_SLEW_RATE / currentTranslationMagnitude);
            } else {
                directionSlewRate = 500.0; // Instantaneous if not moving
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - previousTime;
            double angleDif = SwerveUtils.angleDifference(inputTranslationDirection, currentTranslationDirection);

            if(angleDif < 0.45 * Math.PI) {
                currentTranslationDirection = SwerveUtils.stepTowardsCircular(currentTranslationDirection, inputTranslationDirection, directionSlewRate * elapsedTime);
                currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
            } else if(angleDif > 0.85 * Math.PI) {
                if(currentTranslationMagnitude > 1e-4) {
                    currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
                } else {
                    currentTranslationDirection = SwerveUtils.wrapAngle(currentTranslationDirection + Math.PI);
                    currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
                }
            } else {
                currentTranslationDirection = SwerveUtils.stepTowardsCircular(currentTranslationDirection, inputTranslationDirection, directionSlewRate * elapsedTime);
                currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
            }
            previousTime = currentTime;

            xSpeedCommand = currentTranslationMagnitude * Math.cos(currentTranslationDirection);
            ySpeedCommand = currentTranslationMagnitude * Math.sin(currentTranslationDirection);
            currentRotation = rotationLimiter.calculate(rSpeed);

        } else {
            xSpeedCommand = xSpeed;
            ySpeedCommand = ySpeed;
            currentRotation = rSpeed;
        }

        // Convert the commanded speeds into proper units
        double xSpeedDelivered = xSpeedCommand * DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND;
        double ySpeedDelivered = ySpeedCommand * DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND;
        double rotDelivered = currentRotation * DriveConstants.MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND;

        SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, this.getRotation2d())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

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

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        backLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    /**
     * How fast the robot is turning in degrees per second.
     */
    public double getTurnRate() {
        return gyro.getRate() * (DriveConstants.GYRO_INVERTED ? -1.0 : 1.0);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    private String getFomattedPose() {
        Pose2d pose = this.getPose();

        return String.format(
            "(%.3f, %.3f) %.2f degrees",
            pose.getX(),
            pose.getY(),
            pose.getRotation().getDegrees()
        );
    }

    public void addDashboardWidgets(ShuffleboardTab tab) {
        tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
        tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
    }

    /**
     * Transforms a pose to the opposite alliance's coordinate system. (0,0) is always on the right corner of your alliance wall, so for 2023, the field elements are at different
     * coordinates for each alliance.
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
                // No valid alliance data. Nothing we can do about it
        }

        if(allianceChanged && sawTag) {
            // The alliance changed, which changes the coordinate system.
            // Since a tag was seen, and the tags are all relative to the coordinate system, the estimated pose
            // needs to be transformed to the new coordinate system.
            Pose2d newPose = flipAlliance(this.getPose());
            poseEstimator.resetPosition(this.getRotation2d(), this.getModulePositions(), newPose);
        }
    }
}
