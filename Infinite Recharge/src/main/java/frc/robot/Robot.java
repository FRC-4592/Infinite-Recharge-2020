/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                                                */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Lib.Loop.MultiLooper;
import frc.robot.Subsystems.CannonPolycord;
import frc.robot.Subsystems.CannonShooter;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.CannonPivot;
import frc.robot.Subsystems.ColorSensor;
import frc.robot.Subsystems.ColorWheel;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.IntakeWheels;
import frc.robot.Subsystems.Auto.AutoTest;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	//private static final String kDefaultAuto = "Default";
	//private static final String kCustomAuto = "My Auto";
    //private String m_autoSelected;
    
    // Loopers
    private MultiLooper DriveLooper = new MultiLooper("DriveLooper", 1 / 200.0, false);
    private MultiLooper SSLooper = new MultiLooper("SSLooper", 1 / 200.0, false);
    private MultiLooper AutoLooper = new MultiLooper("AutoLooper", 1 / 200.0, false);

    // Subsystems - Finish Auto Later
    private Drivetrain myDrive = new Drivetrain(Hardware.rightMasterMotor, Hardware.rightSlaveMotor, 
    Hardware.leftMasterMotor, Hardware.leftSlaveMotor, Hardware.MXP, Hardware.shifter);
    private ColorSensor colorSensor = new ColorSensor(Constants.colorSensor);
    private ColorWheel colorWheel = new ColorWheel(Hardware.colorWheelMotor);
    private IntakeWheels intakeWheels = new IntakeWheels(Hardware.intakeMotor, Hardware.shifter2);
    private CannonPivot cannonPivot = new CannonPivot(Hardware.pivotMasterMotor);
    private CannonPolycord cannonPolycord = new CannonPolycord(Hardware.polycordMasterMotor, Hardware.polycordSlaveMotor);
    private CannonShooter cannonShooter = new CannonShooter(Hardware.shooterMasterMotor, Hardware.shooterSlaveMotor);
    private Climber climber = new Climber(Hardware.wenchMasterMotor, Hardware.wenchSlaveMotor, Hardware.shifter3);
    // Autos
    private AutoTest Auto = new AutoTest(myDrive);
    // Selectables
    //private final SendableChooser<String> m_chooser = new SendableChooser<>();
    //private final SendableChooser<String> sideSelect = new SendableChooser<>();
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        //m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        //m_chooser.addOption("My Auto", kCustomAuto);
        //SmartDashboard.putData("Auto choices", m_chooser);
        SSLooper.addLoopable(colorSensor);
        SSLooper.addLoopable(colorWheel);
        SSLooper.addLoopable(intakeWheels);
        SSLooper.addLoopable(cannonPivot);
        SSLooper.addLoopable(cannonPolycord);
        SSLooper.addLoopable(cannonShooter);
        SSLooper.addLoopable(climber);

        colorSensor.setupSensors();
        colorWheel.setupSensors();
        intakeWheels.setupSensors();
        cannonPivot.setupSensors();
        cannonPolycord.setupSensors();
        cannonShooter.setupSensors();
        climber.setupSensors();

        myDrive.setupSensors();
        DriveLooper.addLoopable(myDrive);
        AutoLooper.addLoopable(Auto);
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
        // CommandScheduler.getInstance().run();
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
        AutoLooper.start();
        AutoLooper.update();

        //m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        //System.out.println("Auto selected: " + m_autoSelected);

    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        /* switch (m_autoSelected) {
            case kCustomAuto:
                // Put custom auto code here
                break;
            case kDefaultAuto:
            default:
                // Put default auto code here
                break; 
        }*/
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void teleopInit() {
        DriveLooper.start();
        DriveLooper.update();
        SSLooper.start();
        SSLooper.update();
    }
}
