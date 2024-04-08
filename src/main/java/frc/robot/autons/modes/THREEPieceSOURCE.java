package frc.robot.autons.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class THREEPieceSOURCE extends SequentialCommandGroup{
    private final PathPlannerPath SourcePathForward = PathPlannerPath.fromChoreoTrajectory("sourcecloseforward");
    private final PathPlannerPath SourcePathBackward = PathPlannerPath.fromChoreoTrajectory("sourceclosebackward");
    private final PathPlannerPath SourcePathForward2 = PathPlannerPath.fromChoreoTrajectory("sourcefarforward");
    private final PathPlannerPath SourcePathBackward2 = PathPlannerPath.fromChoreoTrajectory("sourcefarbackward");
    public THREEPieceSOURCE(Swerve swerve, Superstructure superstructure){
    SuperstructureStates shootSide = DriverStation.getAlliance().equals(Alliance.Blue) ? SuperstructureStates.SPIN_UP_RIGHT : SuperstructureStates.SPIN_UP_RIGHT;
    final double X_initial = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0.6709336638450623 : 16.54 - 0.6709336638450623;
    final double Rot_initial = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? -1.0471975512 : -2*1.0471975512;
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d(X_initial,4.377834320068359), new Rotation2d(Rot_initial)))),
            new InstantCommand(() -> superstructure.setState(shootSide)),
            new WaitCommand(1.25),
            new ParallelCommandGroup(
            AutoBuilder.followPath(SourcePathForward),
            new SequentialCommandGroup(
                new WaitCommand(0.7),
                new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE)))
                ),
            new WaitCommand(0.3),
            AutoBuilder.followPath(SourcePathBackward),
            new ParallelCommandGroup(
                new WaitCommand(1.25),
                new InstantCommand(() -> superstructure.setState(shootSide))
            ),
            new ParallelCommandGroup(
            AutoBuilder.followPath(SourcePathForward2),
            new SequentialCommandGroup(
                new WaitCommand(2.2),
                new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE)))
                ),
            new WaitCommand(0.3),
            AutoBuilder.followPath(SourcePathBackward2),
            new ParallelCommandGroup(
                new WaitCommand(1.25),
                new InstantCommand(() -> superstructure.setState(shootSide))
            )
            );
                    
    }
}