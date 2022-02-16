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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Shooter;

/**
 * CPA Import the Commands Directory
 * or jus the command
 */
import frc.robot.Commands.*;
//or
//import frc.robot.Commands.manual_Drive;



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
  private final Climb climb = new Climb();

  private final Shooter shooter = new Shooter();

  private SequentialCommandGroup climbAuto = new SequentialCommandGroup(
  
    // sets arm piston to true, reaching arm out fully
    new InstantCommand(()-> climb.reaching(true)),
    
    /*could use encoders and a PID loop for smoother movements on lift*/
    
    // moves lift up at 40% speed until lift limit switch is hit
    new RunCommand(() -> climb.move(liftUpSpeed)).withInterrupt(climb::isLiftExtended),

    // sets piston to false, moving arm back fully until it is vertical
    new InstantCommand(()-> climb.reaching(false)),

    // moves lift down at 40% speed until lift limit switch is pressed
    new RunCommand(() -> climb.move(liftDownSpeed)).withInterrupt(climb::isHookEngaged),
    
  /** sequential command is repeated **/

    // sets arm piston to true, reaching arm out fully
    new InstantCommand(()-> climb.reaching(true)),
        
    // moves lift up at 40% speed until lift limit switch is hit
    new RunCommand(() -> climb.move(liftUpSpeed)).withInterrupt(climb::isLiftExtended),

    // sets piston to false, moving arm back fully until it is vertical
    new InstantCommand(()-> climb.reaching(false)),

    // moves lift down at 40% speed until lift limit switch is pressed
    new RunCommand(() -> climb.move(liftDownSpeed)).withInterrupt(climb::isHookEngaged)
  
  );

  private SequentialCommandGroup shootThenGo = new SequentialCommandGroup(
    
    // shoot the cargo into the goal
    new InstantCommand (() -> shooter.setShoot(true)),
    new WaitCommand (shooterWaitTime),
    new InstantCommand (() -> shooter.setShoot(false))

    // drive backwards at 50% speed for 5 seconds
    .andThen(new RunCommand(() -> rDrive.getDifferentialDrive()
    .tankDrive(autoDriveSpeed, autoDriveSpeed),rDrive).withTimeout(5))

  );

  private SequentialCommandGroup resetClimb = new SequentialCommandGroup(
    
    // set the piston back to original position
    new InstantCommand(() -> climb.reaching(false)),

    // move the lift down until it is at the farthest down position using PID loop
    // -- this code causes the robot to stop teleop lift motor from functioning
    // new RunCommand(() -> climb.liftPID(-5))

    // move the lift down for a number of seconds 
    /*-- used as place holder until functioning PID loop 
    that does not interefere with teleop climb is implemented*/
    new RunCommand(() -> climb.move(-0.1)).withTimeout(2)
  );

  private SequentialCommandGroup shoot = new SequentialCommandGroup(
    new InstantCommand(()-> shooter.setShoot(true)),
    new WaitCommand(shooterWaitTime),
    new InstantCommand(()-> shooter.setShoot(false))
  );

  // drives the robot using joysticks
  /**
   * CPA Now uses the manual_Drive Command
   */
  /* private Command manualDrive = new RunCommand(
    
    () -> rDrive.getDifferentialDrive().
    tankDrive(rDrive.deadband(xbox.getRawAxis(kLeftY.value), percentDeadband), 
    rDrive.deadband(xbox.getRawAxis(kRightY.value), percentDeadband),
    false
    ),
    rDrive
    ); */

  // move the lift up and down with right and left triggers, respectively
  private Command moveArm = new RunCommand(
  
    () -> climb.move(xbox.getRawAxis(kRightTrigger.value) - xbox.getRawAxis(kLeftTrigger.value)), climb);

  public RobotContainer() {

    // configure the button bindings
    configureButtonBindings();

    // default to running moveArm and manualDrive
    climb.setDefaultCommand(moveArm);

    /**
     * CPA
     * 
     * Inject the Controlers using the Lamda () -> "Function"
     */
    rDrive.setDefaultCommand(new manual_Drive(() -> xbox.getRawAxis(kLeftY.value),() -> xbox.getRawAxis(kRightY.value), rDrive));

  }

  private void configureButtonBindings() {
    
    // moves arm in when left bumper is pressed
    new JoystickButton(xbox, kLeftBumper.value)
    .whenPressed(new InstantCommand (() -> climb.reaching(false)));

    // moves arm out when right bumper is pressed
    new JoystickButton(xbox, kRightBumper.value)
    .whenPressed(new InstantCommand (() -> climb.reaching(true)));

    // shoots when Y is pressed
    new JoystickButton(xbox, kY.value)
    .whenPressed(shoot);

    // runs auto limit switch climb code when A is pressed
    new JoystickButton(xbox, kA.value)
    .whenPressed(climbAuto);

    //cancels auto limit switch climb code when B is pressed
    new JoystickButton(xbox, kX.value)
    .whenPressed(resetClimb)
    .cancelWhenPressed(climbAuto);

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
