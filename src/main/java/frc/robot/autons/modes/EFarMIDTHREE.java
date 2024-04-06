package frc.robot.autons.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class EFarMIDTHREE extends SequentialCommandGroup{
    private final PathPlannerPath MidPathForward = PathPlannerPath.fromChoreoTrajectory("midcloseforward");
    private final PathPlannerPath MidPathBackward = PathPlannerPath.fromChoreoTrajectory("midclosebackward");
    private final PathPlannerPath MidPathForward2 = PathPlannerPath.fromChoreoTrajectory("Emidfarforward");
    private final PathPlannerPath MidPathBackward2 = PathPlannerPath.fromChoreoTrajectory("Emidfarbackward");
    public EFarMIDTHREE(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d(1.3642183542251587, 5.5551371574401855), new Rotation2d(0)))),
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_MID)),
            new WaitCommand(1.25),
            new ParallelCommandGroup(
                AutoBuilder.followPath(MidPathForward),
                new SequentialCommandGroup(
                    new WaitCommand(0.7),
                    new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE)))
                    ),
                new WaitCommand(0.3),
                AutoBuilder.followPath(MidPathBackward),
                new ParallelCommandGroup(
                    new WaitCommand(1.25),
                    new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID))
                ),
            new ParallelCommandGroup(
            AutoBuilder.followPath(MidPathForward2),
            new SequentialCommandGroup(
                new WaitCommand(1.9),
                new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE)))
                ),
            new WaitCommand(0.3),
            AutoBuilder.followPath(MidPathBackward2),
            new ParallelCommandGroup(
                new WaitCommand(1.25),
                new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID))
            )
            );
                    
    }
}