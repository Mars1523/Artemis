package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class climbAuto extends SequentialCommandGroup {
    SwerveDriveSubsystem swerveDriveSubsystem;
    Climb climbSubsystem;

    public climbAuto(Climb climbSubsystem, SwerveDriveSubsystem swerveDriveSubsystem) {
        this.climbSubsystem = climbSubsystem;
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        addCommands(
                new ParallelCommandGroup(climbSubsystem.armDownCommand(), climbSubsystem.armDownCommand2()),
                new WaitCommand(5),
                new AutoDrive(swerveDriveSubsystem, 2, 0.3),
                new ParallelCommandGroup(
                        new AutoDrive(swerveDriveSubsystem, 5, 0.1),
                        climbSubsystem.armDownCommand(),
                        climbSubsystem.armDownCommand2()),
                new WaitCommand(2));
    }
}
