package frc.robot.auto.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveDrive;
import java.util.function.Supplier;

public class AmpAlign extends Command {
    private final SwerveDrive swerveDrive;
    private final Supplier<Pose2d> poseProvider;

    public AmpAlign(SwerveDrive swerveDrive, Supplier<Pose2d> poseProvider) {
        this.swerveDrive = swerveDrive;
        this.poseProvider = poseProvider;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
