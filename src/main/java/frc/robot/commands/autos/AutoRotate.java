package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Simple example auto that rotates the robot by goalYaw degrees at rate of turnSpeed
 */
public class AutoRotate extends Command {

    private SwerveDriveSubsystem swerveSub;
    private double goalYaw;
    private double turnSpeed;
    private Rotation2d startingYaw;

    public AutoRotate(SwerveDriveSubsystem swerveSub, double goalYaw, double turnSpeed) {

        addRequirements(swerveSub);

        this.swerveSub = swerveSub;
        this.turnSpeed = turnSpeed;
        this.goalYaw = goalYaw;
        if (goalYaw < 0) {
            this.turnSpeed = -turnSpeed;
        }
    }

    @Override
    public void initialize() {
        startingYaw = swerveSub.getOdometryHeading();
        if (goalYaw < 0) {
            turnSpeed = -turnSpeed;
        }
    }

    @Override
    public void execute() {
        swerveSub.drive(0, 0, turnSpeed, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSub.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(goalYaw)
                < Math.abs(swerveSub.getOdometryHeading().minus(startingYaw).getDegrees())) {
            return true;
        } else {
            return false;
        }
    }
}
