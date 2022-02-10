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
import frc.robot.subsystems.Shooter;

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

  private final Shooter shooter = new Shooter();

  private SequentialCommandGroup climb = new SequentialCommandGroup(
    
    // sets arm piston to true, reaching arm out fully
    new InstantCommand(()-> climbAuto.reaching(true)),
    
    /*could use encoders and a PID loop for smoother movements on lift*/
    
    // moves lift up at 40% speed until lift limit switch is hit
    new RunCommand(() -> climbAuto.move(liftUpSpeed)).withInterrupt(climbAuto::isLiftExtended),

    // sets piston to false, moving arm back fully until it is vertical
    new InstantCommand(()-> climbAuto.reaching(false)),

    // moves lift down at 40% speed until lift limit switch is pressed
    new RunCommand(() -> climbAuto.move(liftDownSpeed)).withInterrupt(climbAuto::isHookEngaged),
    
  /** sequential command is repeated **/

    // sets arm piston to true, reaching arm out fully
    new InstantCommand(()-> climbAuto.reaching(true)),
        
    // moves lift up at 40% speed until lift limit switch is hit
    new RunCommand(() -> climbAuto.move(liftUpSpeed)).withInterrupt(climbAuto::isLiftExtended),

    // sets piston to false, moving arm back fully until it is vertical
    new InstantCommand(()-> climbAuto.reaching(false)),

    // moves lift down at 40% speed until lift limit switch is pressed
    new RunCommand(() -> climbAuto.move(liftDownSpeed)).withInterrupt(climbAuto::isHookEngaged)

  );

  private SequentialCommandGroup shootThenGo = new SequentialCommandGroup(
    
    // shoot the cargo into the goal
    new InstantCommand (() -> shooter.setShoot(true))
    .andThen(new InstantCommand (() -> shooter.setShoot(false)))

    // drive backwards at 50% speed for 5 seconds
    .andThen(new RunCommand(() -> rDrive.getDifferentialDrive().tankDrive(-0.5, -0.5),rDrive).withTimeout(5))

  );

  // drives the robot using joysticks
  private Command manualDrive = new RunCommand(
    
    () -> rDrive.getDifferentialDrive().
    tankDrive(rDrive.deadband(xbox.getRawAxis(kLeftY.value), percentDeadband), 
    rDrive.deadband(xbox.getRawAxis(kRightY.value), percentDeadband),
    false
    ),
    rDrive
    );

  // move the lift up and down with right and left triggers, respectively
  private Command moveArm = new RunCommand(
  
    () -> climbAuto.move(xbox.getRawAxis(kRightTrigger.value) - xbox.getRawAxis(kLeftTrigger.value)), climbAuto);

  public RobotContainer() {

    // configure the button bindings
    configureButtonBindings();

    // default to running moveArm and manualDrive
    climbAuto.setDefaultCommand(moveArm);
    rDrive.setDefaultCommand(manualDrive);

  }

  private void configureButtonBindings() {
    
    // moves arm in when left bumper is pressed
    new JoystickButton(xbox, kLeftBumper.value)
    .whenPressed(new InstantCommand (() -> climbAuto.reaching(false)));

    // moves arm out when right bumper is pressed
    new JoystickButton(xbox, kRightBumper.value)
    .whenPressed(new InstantCommand (() -> climbAuto.reaching(true)));

    // shoots when Y is pressed
    new JoystickButton(xbox, kY.value)
    .whenPressed(new InstantCommand (() -> shooter.setShoot(true))
    .andThen(new InstantCommand (() -> shooter.setShoot(false))));

    // runs auto limit switch code when A is pressed
    new JoystickButton(xbox, kA.value)
    .whenPressed(climb);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   public Command getAutonomousCommand() {
     return shootThenGo;
   }
}
