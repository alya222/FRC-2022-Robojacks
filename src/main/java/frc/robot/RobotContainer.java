/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// import Xbox Controller and related buttons and axes
import edu.wpi.first.wpilibj.XboxController;
import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static edu.wpi.first.wpilibj.XboxController.Button.*;

// import commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbAuto;

// import RevDrivetrain subsystem
import frc.robot.subsystems.RevDrivetrain;

// import constants
import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  // drive controller
  private XboxController xbox = new XboxController(kXboxPort);

  // drive subsystem
  private final RevDrivetrain rDrive = new RevDrivetrain();

  // climbAuto subsystem
  private final ClimbAuto climbAuto = new ClimbAuto();

  // Drive with Controller 
  private Command manualDrive = new RunCommand(
    
    // drives the robot using joysticks
    () -> rDrive.getDifferentialDrive().
    tankDrive(rDrive.deadband(xbox.getRawAxis(kLeftY.value), percentDeadband), 
    rDrive.deadband(xbox.getRawAxis(kRightY.value), percentDeadband),
    false
    ),
    rDrive
    );

    private Command moveArm = new RunCommand(
  
  // move the lift up and down with right and left triggers, respectively
    () -> climbAuto.move(xbox.getRawAxis(kRightTrigger.value) - xbox.getRawAxis(kLeftTrigger.value)), climbAuto);

  public RobotContainer() {

    // configure the button bindings
    configureButtonBindings();

    // default to running moveArm and manualDrive
    climbAuto.setDefaultCommand(moveArm);
    rDrive.setDefaultCommand(manualDrive);

  }

  private void configureButtonBindings() {
    
    // moves arm in when A is pressed
    new JoystickButton(xbox, kA.value)
    .whenPressed(new InstantCommand (() -> climbAuto.reaching(false)));

    // moves arm out when Y is pressed
    new JoystickButton(xbox, kY.value)
    .whenPressed(new InstantCommand (() -> climbAuto.reaching(true)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
