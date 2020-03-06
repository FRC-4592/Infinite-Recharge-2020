package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.Hardware;
import frc.robot.Lib.SubsystemFramework;
import frc.robot.Subsystems.CannonPolycord.CannonpolycordStates;
import frc.robot.Subsystems.CannonPolycord;

public class CannonShooter implements SubsystemFramework {
    private WPI_TalonSRX masterShooter;
    private WPI_TalonSRX slaveShooter;

    public CannonShooterStates state = CannonShooterStates.Off;
    public CannonShooter(WPI_TalonSRX masterShooter, WPI_TalonSRX slaveShooter) {
        this.masterShooter = masterShooter;
        this.slaveShooter = slaveShooter;
    }
    
    public enum CannonShooterStates {
        Off, Shoot
    }    
    
    public boolean Shoot() {
        // It's going to be a trigger button on the controller
        return(Hardware.driverPad.getRawAxis(Constants.SHOOT) > 0.1);
    }
    public void update() {
        CannonShooterStates newState = state;

        switch(state) {
            case Off:
                masterShooter.set(0);
                if(Shoot()) {
                    newState = CannonShooterStates.Shoot;
                }
                break;
            case Shoot:
                masterShooter.set(1); //Not gonna be like this, instead going to be a closed loop velocity PID
                CannonPolycord.state = CannonpolycordStates.Shooting;
                
                if(!Shoot()) {
                    newState = CannonShooterStates.Off;
                }
                break;
            default:
                newState = CannonShooterStates.Off;
                break;
        }
        if (newState != state) {
            state = newState;
        }
        outputToSmartDashboard();
    }

    public void setupSensors() {
        slaveShooter.follow(masterShooter);

        slaveShooter.setInverted(true);

        masterShooter.configNominalOutputForward(0, 10);
        masterShooter.configNominalOutputReverse(0, 10);
        masterShooter.configPeakOutputForward(1, 10);
        masterShooter.configPeakOutputReverse(-1, 10);
    }

    public void outputToSmartDashboard() {
        
    }
}