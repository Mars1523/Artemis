package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoTo;
import frc.robot.subsystems.AimingSub;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class NeutralRight extends SequentialCommandGroup {
    final SwerveDriveSubsystem swerveSubsystem;
    final IntakeSubsystem intakeSubsystem;
    final AimingSub aimingSub;

    public NeutralRight(Climb climb, SwerveDriveSubsystem swerveSubsystem, AimingSub aimingSub, IntakeSubsystem intakeSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.aimingSub = aimingSub;
        this.intakeSubsystem = intakeSubsystem;

        addCommands(new ParallelCommandGroup(aimingSub.shootPhotonCommand().withTimeout(8), intakeSubsystem.runIntake()), new PathPlannerAuto("NeutralRightAuto"));
    }
}
