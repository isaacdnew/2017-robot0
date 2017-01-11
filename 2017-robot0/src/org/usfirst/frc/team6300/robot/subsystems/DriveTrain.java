package org.usfirst.frc.team6300.robot.subsystems;

import org.usfirst.frc.team6300.robot.RobotMap;
import org.usfirst.frc.team6300.robot.commands.Mecanum;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
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
		leftFrontMotor = new Spark(RobotMap.leftFrontMotor);
		rightFrontMotor = new Spark(RobotMap.leftFrontMotor);
		leftBackMotor = new Spark(RobotMap.leftFrontMotor);
		rightBackMotor = new Spark(RobotMap.leftFrontMotor);
		
		leftFrontMotor.setInverted(RobotMap.lfInverted);
		rightFrontMotor.setInverted(RobotMap.rfInverted);
		leftBackMotor.setInverted(RobotMap.lbInverted);
		rightBackMotor.setInverted(RobotMap.rbInverted);
		
		gyro = new AnalogGyro(RobotMap.gyro);
	}
	
	public void initDefaultCommand() {
        setDefaultCommand(new Mecanum());
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
	
	public void calibrateGyro() {
		brake();
		gyro.calibrate();
		targetHeading = 0;
	}
	
	
	public void drive(Joystick joy, int forwardAxis, int slideAxis, int rotationAxis) {
		double forwardSpeed = joy.getRawAxis(forwardAxis);
		double slideSpeed = joy.getRawAxis(slideAxis);
		double rotationSpeed = joy.getRawAxis(rotationAxis);
		
		/**
		 * yaw correction:
		 */
		targetHeading = gyro.getAngle() + rotationSpeed;
		SmartDashboard.putNumber("Target Heading", targetHeading);
		
		if (targetHeading > gyro.getAngle()) {
			//add speed to right side
			rfSpeed = rfSpeed + 0.1;
			rbSpeed = rbSpeed + 0.1;
			
			//decrease speed on left side
			lfSpeed = lfSpeed - 0.1;
			lbSpeed = lbSpeed - 0.1;
		}
		else if (targetHeading < gyro.getAngle()) {
			//decrease speed on right side
			rfSpeed = rfSpeed - 0.1;
			rbSpeed = rbSpeed - 0.1;
			
			//increase speed on left side
			lfSpeed = lfSpeed + 0.1;
			lbSpeed = lbSpeed + 0.1;
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
	
}

