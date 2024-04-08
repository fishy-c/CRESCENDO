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

public class THREEPieceAMP extends SequentialCommandGroup{
    private final PathPlannerPath AmpPathForward1 = PathPlannerPath.fromChoreoTrajectory("ampcloseforward");
    private final PathPlannerPath AmpPathBackward1 = PathPlannerPath.fromChoreoTrajectory("ampclosebackward");
    private final PathPlannerPath AmpPathForward2 = PathPlannerPath.fromChoreoTrajectory("ampfarforwardR");
    private final PathPlannerPath AmpPathBackward2 = PathPlannerPath.fromChoreoTrajectory("ampfarbackwardR");
    public THREEPieceAMP(Swerve swerve, Superstructure superstructure){
    SuperstructureStates shootSide = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? SuperstructureStates.SPIN_UP_LEFT : SuperstructureStates.SPIN_UP_LEFT;
    final double X_initial = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0.6815703511238098 : 16.54 - 0.6815703511238098;
    final double Rot_initial = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 1.0471975512 : 2*1.0471975512;
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d(X_initial,6.723508358001709), new Rotation2d(Rot_initial)))),
            new InstantCommand(() -> superstructure.setState(shootSide)),
            new WaitCommand(1.25),
            new ParallelCommandGroup(
            AutoBuilder.followPath(AmpPathForward1),
            new SequentialCommandGroup(
                new WaitCommand(0.7),
                new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE)))
                ),
            new WaitCommand(0.3),
            AutoBuilder.followPath(AmpPathBackward1),
            new ParallelCommandGroup(
                new WaitCommand(1.25),
                new InstantCommand(() -> superstructure.setState(shootSide))
            ),
            new ParallelCommandGroup(
            AutoBuilder.followPath(AmpPathForward2),
            new SequentialCommandGroup(
                new WaitCommand(1.8),
                new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE)))
                ),
            new WaitCommand(0.3),
            AutoBuilder.followPath(AmpPathBackward2),
            new ParallelCommandGroup(
                new WaitCommand(1.25),
                new InstantCommand(() -> superstructure.setState(shootSide))
            )
            );
                    
    }
}