/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Joystick gamepad;

  private WPI_VictorSPX leftMotor;
  private WPI_VictorSPX rightMotor;

  private DifferentialDrive chassis;

  private WPI_TalonSRX intakeOne;
  private WPI_TalonSRX hood;
  private 
  private WPI_TalonSRX shooter;

  private Spark intakeTwo;
  private Spark intakeThree;
  private Spark intakeGround;
  private Spark drawBridge;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    gamepad = new Joystick(0);

    leftMotor = new WPI_VictorSPX(0);
    rightMotor = new WPI_VictorSPX(1);

    chassis = new DifferentialDrive(leftMotor, rightMotor);

    intakeOne = new WPI_TalonSRX(2);
    hood = new WPI_TalonSRX(3);
    shooter = new WPI_TalonSRX(4);

    intakeTwo = new Spark(0);
    intakeThree = new Spark(1);
    intakeGround = new Spark(2);
    drawBridge = new Spark(3);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
      // lower drawbridge until down
      do {
        drawBridge.set(-1);
      } while (!drawBridgeDown.get());
      // STOP WHEN DOWN
      drawBridge.set(0);

      break;
    }
  }

    /**
     * Set based on what direction you want forward/positive to be. This does not
     * affect sensor phase.
     */
  @Override
  public void teleopPeriodic() {
    double forw = gamePad.getRawAxis(1) * .85;
    double angle = gamePad.getRawAxis(0);
    if (forw < .1 && forw > -.1)
      forw = 0;
    if (angle < .1 && angle > -.1)
      angle = 0;
    chassis.arcadeDrive(forw, angle);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
