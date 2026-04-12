package frc.robot.commands.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AimingSub;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class NeutralRight extends SequentialCommandGroup {
    final SwerveDriveSubsystem swerveSubsystem;
    final IntakeSubsystem intakeSubsystem;
    final AimingSub aimingSub;

    public NeutralRight(SwerveDriveSubsystem swerveSubsystem, AimingSub aimingSub, IntakeSubsystem intakeSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.aimingSub = aimingSub;
        this.intakeSubsystem = intakeSubsystem;

        addCommands(
                new ParallelCommandGroup(intakeSubsystem.runIntake(), aimingSub.shootPhotonCommand()).withTimeout(6),
                new PathPlannerAuto("NeutralRightAuto"));
    }
}
