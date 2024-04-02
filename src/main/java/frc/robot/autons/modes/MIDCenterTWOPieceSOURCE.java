package frc.robot.autons.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class MIDCenterTWOPieceSOURCE extends SequentialCommandGroup{
    private final PathPlannerPath Path = PathPlannerPath.fromChoreoTrajectory("midcenter2piecesource");
    public MIDCenterTWOPieceSOURCE(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            AutoBuilder.followPath(Path)
            );
        
    }
}

