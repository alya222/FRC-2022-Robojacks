// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    // Port Numbers

	// Xbox controller port
	public static final int kXboxPort = 0;

	// Drive Ports
	public static final int kLeftFrontPort = 1;
	public static final int kLeftRearPort = 2;
	public static final int kRightFrontPort = 3;
	public static final int kRightRearPort = 4;

	public static final double percentDeadband = 0.08;

    // assign port number to lift motor
    public static final int liftPort = 5;

    // add a compressor
    public Compressor airow = new Compressor(0, PneumaticsModuleType.CTREPCM);

    // assign port number to compressor
    public static final PneumaticsModuleType compressorModule = PneumaticsModuleType.CTREPCM;
    
    // assign port number to arm piston
    public static final int rightArmMoverPort = 1;
    public static final int leftArmMoverPort = 2;

    // assign port number to shooter piston
    public static final int shooterPistonPort = 0;

    public static final int shooterMotorPort = 6;

    // creates variables for the lift speed moving up and down
    public static final double liftUpSpeed = 0.4;
    public static final double liftDownSpeed = -0.4;

    // shooter wait time in seconds
    public static final double shooterWaitTime = 0.5;

    //speed of shooter
    public static final double shooterSpeed = 0.8;

    // speed of autonomous backing up in percent
    public static final double autoDriveSpeed = -0.4;
    
}
