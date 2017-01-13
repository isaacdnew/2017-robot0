package org.usfirst.frc.team6300.robot.subsystems;

import org.usfirst.frc.team6300.robot.RobotMap;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class GearDropper extends Subsystem {
	private static Servo servo;
	
	public GearDropper() {
		servo = new Servo(RobotMap.servo);
	}
	
	public void dropGear() {
		servo.setAngle(90);
	}
	
	public void close() {
		servo.setAngle(-90);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

