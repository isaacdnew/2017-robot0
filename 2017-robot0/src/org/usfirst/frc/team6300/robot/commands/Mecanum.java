 package org.usfirst.frc.team6300.robot.commands;

import org.usfirst.frc.team6300.robot.OI;
import org.usfirst.frc.team6300.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Mecanum extends Command {
	public Mecanum() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.driveTrain.calibrateGyro();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.driveTrain.drive(OI.driveJoy, 1, 0, 4);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
