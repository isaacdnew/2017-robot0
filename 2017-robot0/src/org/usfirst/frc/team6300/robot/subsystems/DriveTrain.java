package org.usfirst.frc.team6300.robot.subsystems;

import org.usfirst.frc.team6300.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 *
 */
public class DriveTrain extends PIDSubsystem {
	private static SpeedController lfMotor, rfMotor, lbMotor, rbMotor;
	private double lfSpeed = 0;
	private double rfSpeed = 0;
	private double lbSpeed = 0;
	private double rbSpeed = 0;
	
	private static Gyro gyro;
	
	public DriveTrain() {
		super("DriveTrain", 0, 0, 0);
		lfMotor = new VictorSP(RobotMap.lfMotor);
		rfMotor = new VictorSP(RobotMap.rfMotor);
		lbMotor = new VictorSP(RobotMap.lbMotor);
		rbMotor = new VictorSP(RobotMap.rbMotor);
		
		gyro = new AnalogGyro(RobotMap.gyro);
		
		// Use these to get going:
		// setSetpoint() -  Sets where the PID controller should move the system
		//                  to
		// enable() - Enables the PID controller.
		getPIDController().setContinuous(true);
		setInputRange(0, 360);
		setAbsoluteTolerance(0.05);
	}
	
	public void initDefaultCommand() {
	}
	
	protected double returnPIDInput() {
		return gyro.getAngle();
	}
	
	protected void usePIDOutput(double output) {
		lfSpeed = -output;
		rfSpeed = output;
		lbSpeed = -output;
		rbSpeed = output;
	}
	
	public void calibrateHeading() {
		disable();
		brake();
		gyro.calibrate();
	}
	
	public void updateMotors() {
		lfMotor.set(lfSpeed);
		rfMotor.set(rfSpeed);
		lbMotor.set(lbSpeed);
		rbMotor.set(rbSpeed);
	}
	
	public void brake() {
		lfMotor.stopMotor();
		rfMotor.stopMotor();
		lbMotor.stopMotor();
		rbMotor.stopMotor();
	}
	
	public void coast() {
		lfMotor.set(0);
		rfMotor.set(0);
		lbMotor.set(0);
		rbMotor.set(0);
	}
	
	public void teleDrive(Joystick joy, int throttleAxis, int slideAxis, int turnAxis) {
		double targetHeading;
		
	}
	
	public void autoDrive(double targetHeading, double power, double seconds) {
		enable();
		setSetpoint(targetHeading);
		while (!onTarget()) {
			updateMotors();
			Timer.delay(0.05);
		}
		
		
		
	}
}