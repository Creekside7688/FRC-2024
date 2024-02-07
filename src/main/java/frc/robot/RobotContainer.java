package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.lib.zylve.Controller;

import frc.robot.autocommands.AmpScore;
import frc.robot.autocommands.ShootNote;

import frc.robot.auto.PoseEstimator;

import frc.robot.constants.OperatorConstants;
import frc.robot.elevator.Elevator;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.swerve.SwerveDrive;

public class RobotContainer {
    
    
    
    private final SwerveDrive swerveDrive = new SwerveDrive();
    private final PoseEstimator poseEstimator = new PoseEstimator(swerveDrive::getRotation2d, swerveDrive::getModulePositions);

    @SuppressWarnings("unused")
    private final Elevator elevator = new Elevator();


    @SuppressWarnings("unused")
    private final Intake intake = new Intake();

    @SuppressWarnings("unused")
    private final Shooter shooter = new Shooter();

    private final AmpScore AmpScore = new AmpScore(elevator, intake);
    private final ShootNote shootnotefeed = new ShootNote(shooter, intake);


    Controller controller = new Controller(OperatorConstants.CONTROLLER_PORT);

    SendableChooser<Command> autoSelector = new SendableChooser<>();

    public RobotContainer() {
        configureButtonBindings();

        autoSelector.addOption("Left Auto", new PathPlannerAuto("Left Auto"));
        autoSelector.addOption("Right Auto", new PathPlannerAuto("Right Auto"));
        autoSelector.setDefaultOption("Middle Auto", new PathPlannerAuto("Middle Auto"));

        NamedCommands.registerCommand("Amp Score", AmpScore);
        NamedCommands.registerCommand("Shoot Note", shootnotefeed);
        

        Shuffleboard.getTab("auto").add(autoSelector);

        swerveDrive.setDefaultCommand(
            new RunCommand(
                () -> swerveDrive.drive(
                    -MathUtil.applyDeadband(controller.getLeftY(), OperatorConstants.DEADBAND),
                    -MathUtil.applyDeadband(controller.getLeftX(), OperatorConstants.DEADBAND),
                    -MathUtil.applyDeadband(controller.getRightX(), OperatorConstants.DEADBAND),
                    true, true),
                swerveDrive));
    }

    private void configureButtonBindings() {
        controller.getLeftStick()
            .whileTrue(new RunCommand(
                () -> swerveDrive.lockPosition(),
                swerveDrive));

        controller.getRightStick()
            .whileTrue(new RunCommand(
                () -> swerveDrive.zeroHeading(),
                swerveDrive));
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }

    public void onAllianceChanged(Alliance alliance) {
        poseEstimator.setAlliance(alliance);
    }
}
