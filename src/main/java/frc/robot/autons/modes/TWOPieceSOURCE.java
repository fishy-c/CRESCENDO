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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class TwoPieceSource extends SequentialCommandGroup{
    private final PathPlannerPath MidPath = PathPlannerPath.fromChoreoTrajectory("sourceclose");
    public TwoPieceSource(Swerve swerve, Superstructure superstructure){
        SuperstructureStates shootSide = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? SuperstructureStates.SPIN_UP_RIGHT : SuperstructureStates.SPIN_UP_RIGHT;
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d(0.6724225282669067, 4.376314640045166), new Rotation2d(-1.0461863225243422)))),
            new InstantCommand(() -> superstructure.setState(shootSide)),
            new WaitCommand(2),
            new ParallelCommandGroup(
            AutoBuilder.followPath(MidPath),
            new SequentialCommandGroup(
                new WaitCommand(0.7),
                new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE)))
                ),
            new ParallelCommandGroup(
                new WaitCommand(2),
                new InstantCommand(() -> superstructure.setState(shootSide))
            )
            );
                    
    }
}