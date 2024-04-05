package frc.robot.autons.modes;

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

public class TWOPieceAMPcopy extends SequentialCommandGroup{
    private final PathPlannerPath AmpPath = PathPlannerPath.fromChoreoTrajectory("amp2piece");
    public TWOPieceAMPcopy(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d(0.708526670, 6.740869522094727), new Rotation2d(1.0460006482793265)))),
            AutoBuilder.followPath(AmpPath)
            );
        
    }
}

