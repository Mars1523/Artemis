package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

/**
 * Drives the robot via robot-oriented drive with an xbox controller
 * Left stick controls motion, right stick controls steering
 */
public class RobotOrientedControllerSwerve extends Command {

    private XboxController controller;
    private SwerveDriveSubsystem swerveSub;
    boolean slow = false;

    public RobotOrientedControllerSwerve(XboxController controller, SwerveDriveSubsystem swerveSub) {
        addRequirements(swerveSub);
        this.swerveSub = swerveSub;
        this.controller = controller;
    }

    private double signedPow(double a, double pow) {
        return Math.copySign(Math.pow(a, pow), a);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // adding deadbands

        double xSpeed = (MathUtil.applyDeadband(-controller.getLeftY(), 0.1));
        double ySpeed = (MathUtil.applyDeadband(-controller.getLeftX(), 0.1));
        double rot = (MathUtil.applyDeadband(-controller.getRightX(), 0.1));

        double totalSpeed = Math.sqrt((xSpeed * xSpeed) + (ySpeed * ySpeed));
        double speedReduction = signedPow(totalSpeed, 2) * 0.8;

        xSpeed *= speedReduction;
        ySpeed *= speedReduction;

        rot = signedPow(rot * .7, 3);

        swerveSub.drive(xSpeed, ySpeed, rot, DriveMode.ROBOT);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
