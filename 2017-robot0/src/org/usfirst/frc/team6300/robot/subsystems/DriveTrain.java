package org.usfirst.frc.team6300.robot.subsystems;

import org.usfirst.frc.team6300.robot.RobotMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {
	private static SpeedController leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
	private double lfSpeed = 0;
	private double rfSpeed = 0;
	private double lbSpeed = 0;
	private double rbSpeed = 0;
    	
	public DriveTrain() {
		leftFrontMotor = new VictorSP(RobotMap.leftFrontMotor);
		rightFrontMotor = new VictorSP(RobotMap.leftFrontMotor);
		leftBackMotor = new VictorSP(RobotMap.leftFrontMotor);
		rightBackMotor = new VictorSP(RobotMap.leftFrontMotor);
		
		leftFrontMotor.setInverted(RobotMap.lfInverted);
		rightFrontMotor.setInverted(RobotMap.rfInverted);
		leftBackMotor.setInverted(RobotMap.lbInverted);
		rightBackMotor.setInverted(RobotMap.rbInverted);
	}
	
	public void initDefaultCommand() {
    }
	
	private void updateMotors() {
		leftFrontMotor.set(lfSpeed);
		rightFrontMotor.set(rfSpeed);
		leftBackMotor.set(lbSpeed);
		rightBackMotor.set(rbSpeed);
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
	
	public void teleDrive(Joystick joy, int forwardAxis, int slideAxis, int rotationAxis, double power) {
		if (power > 1) {
			power = 1;
		}
		else if (power < 0) {
			power = 0;
		}
		double forwardSpeed = joy.getRawAxis(forwardAxis) * power;
		double slideSpeed = joy.getRawAxis(slideAxis) * power;
		double rotationSpeed = joy.getRawAxis(rotationAxis) * Math.abs(power);
		
		//rotation axis:
		lfSpeed = rotationSpeed;
		rfSpeed = -rotationSpeed;
		lbSpeed = rotationSpeed;
		rbSpeed = -rotationSpeed;
		
		//slide axis:
		lfSpeed -= slideSpeed;
		lbSpeed += slideSpeed;
		
		rfSpeed += slideSpeed;
		rbSpeed -= slideSpeed;
		
		//throttle axis:
		lfSpeed += forwardSpeed;
		rfSpeed += forwardSpeed;
		lbSpeed += forwardSpeed;
		rbSpeed += forwardSpeed;
		
		updateMotors();
		Timer.delay(0.005);
	}
	
	public void goForward(double power, double seconds) {
		
		//go forward at "power" for "seconds" seconds:
		leftFrontMotor.set(power);
		rightFrontMotor.set(power);
		leftBackMotor.set(power);
		rightBackMotor.set(power);
		Timer.delay(seconds);
		brake();
	}
	
	public void slide(double power, boolean toRight, double seconds) {
		
	}
}

