package frc.robot.Subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Hardware;
import frc.robot.Lib.SubsystemFramework;
import frc.robot.Subsystems.CannonShooter;
import frc.robot.Subsystems.CannonShooter.CannonShooterStates;
import frc.robot.Util.doubleSolenoid;



public class Limelight implements SubsystemFramework{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double limelightLED = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getDouble(0);
    private boolean LimelightHasValidTarget;
    private double LimelightDriveCommand;
    private double LimelightSteerCommand;
    private double camtran[];
    int var = 3;

    public Limelight(){ //151 inches from the wall
    }
    public LimelightStates state = LimelightStates.LEDon;
    public enum LimelightStates {
        LEDon, LEDoff
    }

    @Override
    public void update(){
        getLimelightDistance();
        Update_Limelight_Tracking();
        LimelightStates newState = state;
        switch(state) {
            case LEDon:
                newState = LimelightStates.LEDon;
                var = 3;
                if(!Hardware.driverPad.getRawButton(8)){
                    newState = LimelightStates.LEDoff;
                }
                break;
            case LEDoff:
                //newState = LimelightStates.LEDoff;
                //var = 1;
                //if(Hardware.driverPad.getRawButton(8)){
                //    newState = LimelightStates.LEDoff;
                //}
                break;
            default:
                newState = LimelightStates.LEDon;
                break;
        }
        if (newState != state) {
            state = newState;
        }
        table.getEntry("ledMode").setNumber(var);//229 in(about 19ft) 9.79 ty //139 in 22.1 ty 90 change -12.00 change 7.5 inch per ty tick 
        outputToSmartDashboard();
    }

    @Override
    public void outputToSmartDashboard(){
        SmartDashboard.putNumber("Ty", getTy());
        SmartDashboard.putNumber("distance", getLimelightDistance());
    }

    @Override 
    public void setupSensors(){
    
    }
    /* To change Limelight LED from on to off use this.
    table.getEntry("ledMode").forceSetNumber(limelightWantedLED);
    */

    public void Update_Limelight_Tracking() 
    {
        final double STEER_K = -0.05;                    // how hard to turn toward the target
        final double DRIVE_K = 0.1;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 0.0;        // Area of the target when the robot reaches the wall
        this.tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        this.tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        this.ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        this.camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{});
        
        if (tv < 1.0)
    {
        LimelightHasValidTarget = false;
        LimelightDriveCommand = 0.0;
        LimelightSteerCommand = 0.0;
        return;
    }
    LimelightHasValidTarget = true;
    // Start with proportional steering
    double steer_cmd = (tx) * STEER_K;
    LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (DESIRED_TARGET_AREA + (ty)) * DRIVE_K;

    // don't let the robot drive too fast into the goal
    //if (drive_cmd > MAX_DRIVE)
    //{
    //    drive_cmd = MAX_DRIVE;
    //}
        LimelightDriveCommand = drive_cmd;
    
    if (ty <= 2 && ty >= -2){
        LimelightDriveCommand = 0;
    }
	if (tx <= 1 && tx >= -1){
        LimelightSteerCommand = 0;
		}
	}
    public double getLimelightDriveCommand() {
        return this.LimelightDriveCommand;
    }
    public double getLimelightSteerCommand() {
        return this.LimelightSteerCommand;
    }
    public boolean getLimelightHasValidTarget() {
        return this.LimelightHasValidTarget;
    }  
    public int getDistance() {
        return (int)this.camtran[2];
    }
    public void EnableLimelight() {
        table.getEntry("ledMode").setNumber(3);
    }
    public void DisableLimelight() {
        table.getEntry("ledMode").setNumber(1);
    }
    public double getLimelightCamtran(int x) {
        return this.camtran[x];
    }
    public double getTy() { //convert to int later
        return this.ty;
    }
    public double getTx() { //convert to int later
        return this.tx;
    }
    public int getLimelightDistance() {
        SmartDashboard.putNumber("dist", (int)(139 + (getTy() * 7.5)));
        return (int)(139 + (getTy() * 7.5));
    }
    public double findAngle() {
        SmartDashboard.putNumber("Angle", Math.atan(Math.toRadians(getLimelightDistance()/98.25)));
        if(Math.atan(Math.toRadians(getLimelightDistance()/98.25)) >= 45) {
            return Math.atan(Math.toRadians(getLimelightDistance()/98.25));
        }
        return 45; //Adjust distance a little for cannon purposes
    }
    public double findVelocity() {
        double height = 15 * Math.sin(findAngle());
        double gravity = 386.09;
        double actualDistance = getLimelightDistance() + 30;
        double numerator = Math.pow(actualDistance, 2) * gravity;
        double denominator = getLimelightDistance() * Math.sin(2 * findAngle())-2 * height * Math.pow(Math.cos(findAngle()), 2);
        double inchPerSecond = Math.sqrt(numerator/denominator);
        double unitConversion = 819.2/(6.0*Math.PI);
        double velocity = ((inchPerSecond * unitConversion) * 2.451 + 8231.1);
        SmartDashboard.putNumber("Velocity", velocity);
        return velocity;
    }
}