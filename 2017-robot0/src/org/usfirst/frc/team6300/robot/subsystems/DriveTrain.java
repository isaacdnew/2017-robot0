package org.usfirst.frc.team6300.robot.subsystems;

import org.usfirst.frc.team6300.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends PIDSubsystem {
	double loopTime = 0.005;
	
	static SpeedController lfMotor, rfMotor, lbMotor, rbMotor;
	double lfSpeed = 0;
	double rfSpeed = 0;
	double lbSpeed = 0;
	double rbSpeed = 0;
	
	static Gyro gyro;
	
	double targetHeading = 0;
	double pidOutput = 0;
	
	public DriveTrain() {
		super("DriveTrain", 0.025, 0, 0);
		lfMotor = new VictorSP(RobotMap.lfMotor);
		rfMotor = new VictorSP(RobotMap.rfMotor);
		lbMotor = new VictorSP(RobotMap.lbMotor);
		rbMotor = new VictorSP(RobotMap.rbMotor);
		
		gyro = new AnalogGyro(RobotMap.gyro);
		
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
		pidOutput = output;
	}
	
	@Override
	public void disable() {
		super.disable();
		pidOutput = 0;
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
	/**
	 * @param joy
	 * @param throttleAxis
	 * @param slideAxis
	 * @param turnAxis
	 */
	public void teleDrive(Joystick joy, int throttleAxis, int slideAxis, int turnAxis) {
		//set speed to throttle axis
		lfSpeed = joy.getRawAxis(throttleAxis);
		rfSpeed = joy.getRawAxis(throttleAxis);
		lbSpeed = joy.getRawAxis(throttleAxis);
		rbSpeed = joy.getRawAxis(throttleAxis);
		
		//add the slide axis
		lfSpeed += joy.getRawAxis(slideAxis);
		rfSpeed -= joy.getRawAxis(slideAxis);
		lbSpeed -= joy.getRawAxis(slideAxis);
		rbSpeed += joy.getRawAxis(slideAxis);
		
		//add the turn axis
		enable();
		setSetpoint(targetHeading + turnAxis);
		lfSpeed += pidOutput;
		rfSpeed -= pidOutput;
		lbSpeed += pidOutput;
		rbSpeed -= pidOutput;
		
		updateMotors();
	}
	/**
	 * @param targetHeading The heading to turn to
	 * @param power The power to drive at after turning
	 * @param seconds The time to drive after turning
	 * @param coast If true, the robot coasts whenever it needs the motors to stop. If false, it brakes.
	 */
	public void autoDrive(double targetHeading, double power, double seconds, boolean coast) {
		int loopCount = (int) Math.round(seconds / loopTime);
		
		enable();
		setSetpoint(targetHeading);
		while (!onTarget()) {
			lfSpeed = pidOutput;
			rfSpeed = -pidOutput;
			lbSpeed = pidOutput;
			rbSpeed = -pidOutput;
			updateMotors();
			
			Timer.delay(loopTime);
		}
		if (coast) coast();
		else brake();
		
		for(int i = 0; i == loopCount; i++) {
			lfSpeed = pidOutput;
			rfSpeed = -pidOutput;
			lbSpeed = pidOutput;
			rbSpeed = -pidOutput;
			
			lfSpeed += power;
			rfSpeed += power;
			lbSpeed += power;
			rbSpeed += power;
			
			updateMotors();
			
			Timer.delay(loopTime);
		}
		
		if (coast) coast();
		else brake();
	}
	
	/**
	 * @param targetHeading The heading to turn to
	 */
	public void testDrive(double targetHeading) {
		enable();
		setSetpoint(targetHeading);
		while (!onTarget()) {
			lfSpeed = pidOutput;
			rfSpeed = -pidOutput;
			lbSpeed = pidOutput;
			rbSpeed = -pidOutput;
			updateMotors();
			SmartDashboard.putNumber("Heading:", getPosition());
			
			Timer.delay(loopTime);
		}
		brake();
	}
}