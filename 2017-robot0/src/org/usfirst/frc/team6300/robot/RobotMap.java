package org.usfirst.frc.team6300.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	/**
	 * drivetrain:
	 */
	public static int leftFrontMotor = 0;
	public static int rightFrontMotor = 1;
	public static int leftBackMotor = 2;
	public static int rightBackMotor = 3;
	
	public static boolean lfInverted = true;
	public static boolean rfInverted = false;
	public static boolean lbInverted = true;
	public static boolean rbInverted = true;
	
	public static int gyro = 0;
	
	/**
	 * gear dropper:
	 */
	public static int servo = 4;
	
	/**
	 * joysticks:
	 */
	public static int driveJoy = 0;
	public static int deliverJoy = 1;
}
