package frc.robot.commands.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AimingSub;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class NeutralLeft extends SequentialCommandGroup {
    final SwerveDriveSubsystem swerveSubsystem;
    final IntakeSubsystem intakeSubsystem;
    final AimingSub aimingSub;

    public NeutralLeft(SwerveDriveSubsystem swerveSubsystem, AimingSub aimingSub, IntakeSubsystem intakeSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.aimingSub = aimingSub;
        this.intakeSubsystem = intakeSubsystem;

        addCommands(
                new ParallelCommandGroup(aimingSub.shootPhotonCommand().withTimeout(8), intakeSubsystem.runIntake()),
                new PathPlannerAuto("NeutralLeftAuto"));
    }
}
