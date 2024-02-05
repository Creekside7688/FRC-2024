package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.lib.zylve.Controller;
import frc.robot.constants.OperatorConstants;
import frc.robot.elevator.Elevator;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.FlipRotation;
import frc.robot.swerve.commands.SnapRotation;

public class RobotContainer {
    private final SwerveDrive swerveDrive = new SwerveDrive();
    private final SnapRotation snap90 = new SnapRotation(90, swerveDrive);
    private final FlipRotation flip180 = new FlipRotation(swerveDrive);

    @SuppressWarnings("unused")
    private final Elevator elevator = new Elevator();

    @SuppressWarnings("unused")
    private final Intake intake = new Intake();

    @SuppressWarnings("unused")
    private final Shooter shooter = new Shooter();

    Controller controller = new Controller(OperatorConstants.CONTROLLER_PORT);

    SendableChooser<Command> autoSelector = new SendableChooser<>();

    public RobotContainer() {
        configureButtonBindings();

        // NamedCommands.registerCommand("PickupNote", intakeCommand);
        // NamedCommands.registerCommand("FireNote", shooterCommand);

        autoSelector.addOption("Left Auto", new PathPlannerAuto("TestLeftAuto"));
        autoSelector.addOption("Right Auto", new PathPlannerAuto("TestRightAuto"));
        autoSelector.setDefaultOption("Straight Auto", new PathPlannerAuto("TestStraightAuto"));

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

        controller.getLeftTrigger().onTrue(snap90);
        controller.getRightTrigger().whileTrue(flip180);
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }
}
