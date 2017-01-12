package org.usfirst.frc.team6300.robot.subsystems;

import org.usfirst.frc.team6300.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {
	private static SpeedController leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
	private double lfSpeed = 0;
	private double rfSpeed = 0;
	private double lbSpeed = 0;
	private double rbSpeed = 0;
	
	private static Gyro gyro;
	private double targetHeading = 0;
    	
	public DriveTrain() {
		leftFrontMotor = new VictorSP(RobotMap.leftFrontMotor);
		rightFrontMotor = new VictorSP(RobotMap.leftFrontMotor);
		leftBackMotor = new VictorSP(RobotMap.leftFrontMotor);
		rightBackMotor = new VictorSP(RobotMap.leftFrontMotor);
		
		leftFrontMotor.setInverted(RobotMap.lfInverted);
		rightFrontMotor.setInverted(RobotMap.rfInverted);
		leftBackMotor.setInverted(RobotMap.lbInverted);
		rightBackMotor.setInverted(RobotMap.rbInverted);
		
		gyro = new AnalogGyro(RobotMap.gyro);
	}
	
	public void initDefaultCommand() {
    }
	
	public void brake() {;
		leftFrontMotor.stopMotor();
		rightFrontMotor.stopMotor();
		leftBackMotor.stopMotor();
		rightBackMotor.stopMotor();
	}
	
	public void coast() {
		leftFrontMotor.set(0);
		rightFrontMotor.set(0);
		leftBackMotor.set(0);
		rightBackMotor.set(0);
	}
	
	public void calibrateHeading() {
		brake();
		gyro.calibrate();
		targetHeading = 0;
	}
	
	
	public void teleDrive(Joystick joy, int forwardAxis, int slideAxis, int rotationAxis, double power) {
		if (power > 1) 
			power = 1;
		
		else if (power <= 0)
			power = 0;
		
		double forwardSpeed = joy.getRawAxis(forwardAxis) * power;
		double slideSpeed = joy.getRawAxis(slideAxis) * power;
		double rotationSpeed = joy.getRawAxis(rotationAxis) * power;
		
		/**
		 * yaw correction:
		 */
		targetHeading = targetHeading + rotationSpeed;
		SmartDashboard.putNumber("Target Heading", targetHeading);
		
		if (targetHeading > gyro.getAngle()) {
			//add speed to right side
			rfSpeed = rfSpeed + (power / 10);
			rbSpeed = rbSpeed + (power / 10);
			
			//decrease speed on left side
			lfSpeed = lfSpeed - (power / 10);
			lbSpeed = lbSpeed - (power / 10);
		}
		else if (targetHeading < gyro.getAngle()) {
			//decrease speed on right side
			rfSpeed = rfSpeed - (power / 10);
			rbSpeed = rbSpeed - (power / 10);
			
			//increase speed on left side
			lfSpeed = lfSpeed + (power / 10);
			lbSpeed = lbSpeed + (power / 10);
		}
		
		/**
		 * slide axis:
		 */
		lfSpeed = lfSpeed - slideSpeed;
		lbSpeed = lbSpeed + slideSpeed;
		
		rfSpeed = rfSpeed + slideSpeed;
		rbSpeed = rbSpeed - slideSpeed;
		
		/**
		 * throttle axis:
		 */
		lfSpeed = lfSpeed + forwardSpeed;
		rfSpeed = rfSpeed + forwardSpeed;
		lbSpeed = lbSpeed + forwardSpeed;
		rbSpeed = rbSpeed + forwardSpeed;
		
		leftFrontMotor.set(lfSpeed);
		rightFrontMotor.set(rfSpeed);
		leftBackMotor.set(lbSpeed);
		rightBackMotor.set(rbSpeed);
		Timer.delay(0.01);
	}
	
	public void autoDrive(double targetHeading, double power, double seconds) {
		/**
		 * turn to heading:
		 */
		while (targetHeading != gyro.getAngle()) {
			if (targetHeading > gyro.getAngle()) {
				leftFrontMotor.set(power);
				rightFrontMotor.set(-power);
				leftBackMotor.set(power);
				rightBackMotor.set(-power);
			}
			else {
				leftFrontMotor.set(-power);
				rightFrontMotor.set(power);
				leftBackMotor.set(-power);
				rightBackMotor.set(power);
			}
		}
		brake();
		
		/**
		 * go forward at "power" for "seconds" seconds
		 */
		leftFrontMotor.set(power);
		rightFrontMotor.set(power);
		leftBackMotor.set(power);
		rightBackMotor.set(power);
		Timer.delay(seconds);
		brake();
	}
}

