// package frc.robot.subsystems;

// import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.wpilibj.I2C.Port;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.SPI;
// public class NavX extends SubsystemBase {
//   /** Creates a new NavX. */
//   private final AHRS navX;

//   public NavX() {
//     navX = new AHRS(SPI.Port.kMXP, (byte) 50);
//   }

//   public double getAngle() {
//     return navX.getAngle();
//   }

//   public double getXAngle(){
//     return navX.getRoll();
//   }

//   public double getYAngle(){
//     return navX.getPitch();
//   }
  
//   public void reset() {
//     navX.reset();
//   }

//   @Override
//   public void periodic() {
//     SmartDashboard.putNumber("Robot Heading", getAngle());
//     SmartDashboard.putBoolean("IS CONNCETED: ", navX.isConnected());
//   }
// }