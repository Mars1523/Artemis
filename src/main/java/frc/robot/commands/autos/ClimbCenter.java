package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoTo;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class ClimbCenter extends SequentialCommandGroup {
    final Climb climb;
    final SwerveDriveSubsystem swerveSubsystem;

    public ClimbCenter(Climb climb, SwerveDriveSubsystem swerveSubsystem) {
        this.climb = climb;
        this.swerveSubsystem = swerveSubsystem;

        addCommands(GoTo.climbLineUp());
    }
}
