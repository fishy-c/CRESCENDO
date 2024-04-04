package frc.robot.autons.modes;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class TWOPieceSOURCE extends SequentialCommandGroup{
    private final PathPlannerPath SourcePath = PathPlannerPath.fromChoreoTrajectory("source2piece");
    public TWOPieceSOURCE(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
        new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d(0.7458285093307495, 4.372655868530273), new Rotation2d(-1.036673900515809)))),
        AutoBuilder.followPath(SourcePath)
            );
        
    }
}

