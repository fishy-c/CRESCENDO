package frc.robot.autons.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class NAMEDTWOPieceMid extends SequentialCommandGroup{
    private final PathPlannerPath MidPath = PathPlannerPath.fromChoreoTrajectory("midcloseNAMED");
    public NAMEDTWOPieceMid(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d(1.3642183542251587, 5.5551371574401855), new Rotation2d(0)))),
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_MID)),
            new WaitCommand(1.25),
            AutoBuilder.followPath(MidPath)
            );

    }
}
