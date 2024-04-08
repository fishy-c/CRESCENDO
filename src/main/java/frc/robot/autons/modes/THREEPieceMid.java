package frc.robot.autons.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class THREEPieceMid extends SequentialCommandGroup{
    private final PathPlannerPath MidPathForward1 = PathPlannerPath.fromChoreoTrajectory("midcloseforward");
    private final PathPlannerPath MidPathBackward1 = PathPlannerPath.fromChoreoTrajectory("midclosebackward");
    private final PathPlannerPath MidPathForward2 = PathPlannerPath.fromChoreoTrajectory("midcloseforward3");
    private final PathPlannerPath MidPathBackward2 = PathPlannerPath.fromChoreoTrajectory("midclosebackward3");
    public THREEPieceMid(Swerve swerve, Superstructure superstructure){
    final double X_initial = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 1.3642183542251587 : 16.54 - 1.3642183542251587;
    final double Rot_initial = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 3.14159265359;
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d(X_initial, 5.5551371574401855), new Rotation2d(Rot_initial)))),
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_MID)),
            new WaitCommand(1.25),
            new ParallelCommandGroup(
            AutoBuilder.followPath(MidPathForward1),
            new SequentialCommandGroup(
                new WaitCommand(0.7),
                new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE)))
                ),
            new WaitCommand(0.3),
            AutoBuilder.followPath(MidPathBackward1),
            new ParallelCommandGroup(
                new WaitCommand(1.25),
                new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID))
            ),
            new ParallelCommandGroup(
            AutoBuilder.followPath(MidPathForward2),
            new SequentialCommandGroup(
                new WaitCommand(0.6),
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