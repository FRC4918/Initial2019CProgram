/*----------------------------------------------------------------------------*/
/* Team 4918 Robot                                                            */
/*----------------------------------------------------------------------------*/

#include <frc/WPILib.h>
#include "ctre/Phoenix.h"
#include <iostream>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

// all the following are already included by frc/WPILib.h above
// #include <frc/TimedRobot.h>
// #include <frc/Joystick.h>
// #include <frc/Spark.h>
// #include <frc/Compressor.h>
// #include <frc/DoubleSolenoid.h>
// #include <frc/DriverStation.h>
// #include <frc/drive/DifferentialDrive.h>
// #include <frc/AnalogInput.h>

using std::cout;
using std::endl;

/**
 * This sample program shows how to control several motors using a joystick.
 * In the operator control part of the program, the joystick is read and the
 * value is used to determine how to drive the motor.
 *
 * Joystick analog values range from -1 to 1, and speed controller inputs of
 * m_motor*.Set() also range from -1 to 1, so those 2 work together easily;
 * other motor drive methods require some scaling.
 */
class Robot : public frc::TimedRobot {
   public:
      void AutonomousInit() override {
         iAutoCount = 0;
         m_motorLSSlave1.Follow(m_motorLSMaster);
         m_motorLSSlave2.Follow(m_motorLSMaster);
         m_motorRSSlave1.Follow(m_motorRSMaster);
         m_motorRSSlave2.Follow(m_motorRSMaster);
#ifdef JAG_NOTDEFINED
         m_compressor.SetClosedLoopControl(true);  // turn compresser on
#else
         m_compressor.SetClosedLoopControl(false); // turn compressor off
#endif
         m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kForward); // low gear
         m_motorArmMaster.ConfigNominalOutputForward(0, 10);
         m_motorArmMaster.ConfigNominalOutputReverse(0, 10);
         m_motorArmMaster.ConfigPeakOutputForward(1, 10);
         m_motorArmMaster.ConfigPeakOutputReverse(-1,10);
      }

      void AutonomousPeriodic() override {
         if ( iAutoCount < 100 ) {
           m_drive.CurvatureDrive( 0.5, -0.5, 0 );
         } else if ( iAutoCount < 200 ) {
           m_drive.CurvatureDrive( 0.5, 0.5, 0 );
         } else if ( iAutoCount < 201 ) {
           m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kReverse);  // high gear
         } else if ( iAutoCount < 600 ) {
           m_drive.CurvatureDrive( 0.5, -0.5, 0 );
         } else if ( iAutoCount < 700 ) {
           m_drive.CurvatureDrive( 0.5, 0.5, 0 );
      // } else if ( iAutoCount < 700 ) {
      //   m_drive.CurvatureDrive( 0.5, 0.5, 0 );
         } else {
           m_drive.CurvatureDrive( 0.0, 0.0, 0 );
         }
         iAutoCount++;
         if ( 750 < iAutoCount ) iAutoCount = 0;
      }
      void TeleopInit() override {
         m_motorLSSlave1.Follow(m_motorLSMaster);
         m_motorLSSlave2.Follow(m_motorLSMaster);
         m_motorRSSlave1.Follow(m_motorRSMaster);
         m_motorRSSlave2.Follow(m_motorRSMaster);
         m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kForward); // low gear
         m_motorArmMaster.ConfigNominalOutputForward(0, 10);
         m_motorArmMaster.ConfigNominalOutputReverse(0, 10);
         m_motorArmMaster.ConfigPeakOutputForward(1, 10);
         m_motorArmMaster.ConfigPeakOutputReverse(-1,10);
                   // set ramp speed to 1.0 seconds from Neutral to Full power
         m_motorArmMaster.ConfigClosedloopRamp( 1.0, 10 );

         m_motorArmMaster.ConfigPeakCurrentLimit( 50, 10 );  // 50 Amps
         m_motorArmMaster.ConfigPeakCurrentDuration( 200, 10 );  // 200 msecs
         m_motorArmMaster.ConfigContinuousCurrentLimit( 40, 10 );  // 40 Amps
         m_motorArmMaster.EnableCurrentLimit( true );

         m_motorArmMaster.ConfigSelectedFeedbackSensor( FeedbackDevice::CTRE_MagEncoder_Relative,
                                                        0, 0 );
         m_motorArmMaster.SetSensorPhase( true );
         m_motorArmMaster.Config_kF( 0, 0.40000, 0 );   // was 0.456
         m_motorArmMaster.Config_kP( 0, 0.8, 0 );
         m_motorArmMaster.Config_kI( 0, 0.0, 0 ); // if used, make this
                                                  // very small, like 0.006
         m_motorArmMaster.Config_kD( 0, 1.0, 0 );

         DistSensor1.SetOversampleBits( 4 );
         DistSensor1.SetAverageBits( 2 );
         DistSensor1.SetSampleRate( 3200 );
      }

      void TeleopPeriodic() override {

         static int iCallCount = 0;
         double limex = limenttable->GetNumber("tx",0.0);
         double limey = limenttable->GetNumber("ty",0.0);
         double limev = limenttable->GetNumber("tv",0.0);

         iCallCount++;
              
         // jagorigwas: m_motor.Set(m_stick.GetY());
         // m_motorRSMaster.Set(ControlMode::PercentOutput, m_stick.GetY());
         // m_motorLSMaster.Set(ControlMode::PercentOutput, m_stick.GetY());
         if ( m_stick.GetRawButton(3) ) {
            m_drive.ArcadeDrive( m_stick.GetY(), -m_stick.GetX() );
         } else {
            if ( m_stick.GetX() < 0 ) {
               m_drive.CurvatureDrive( m_stick.GetY(),
                                       powl( fabs(m_stick.GetX()), m_stick.GetThrottle()+2.0 ),
                                       m_stick.GetTop() );
            } else {
               m_drive.CurvatureDrive( m_stick.GetY(),
                                       -powl( fabs(m_stick.GetX()), m_stick.GetThrottle()+2.0 ),
                                       m_stick.GetTop() );m_compressor.SetClosedLoopControl(false);
            }
         }
#ifdef JAG_NOTDEFINED
         m_compressor.SetClosedLoopControl(true); // turn compressor on
#else
         m_compressor.SetClosedLoopControl(false); // turn compressor off
#endif
         // Compressor *c = new Compressor(0);
         // m_doublesolenoid.Set(DoubleSolenoid.Value.kOff);
         if ( m_stick.GetTrigger() ) {
            m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kReverse); // high gear
         } else {
            m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kForward); // low gear
         }
         // m_doublesolenoid.Set(DoubleSolenoid::Value::kReverse); // high gear
         if ( !m_stick.GetRawButton(4) ) {
            /* Speed mode */
            /*
             * 4096 Units/Rev * 328 RPM / 600 100ms/min in either direction:
             * velocity setpoint is in units/100ms
             * We found the max speed of this motor/gearbox combo to be
             * about 2240 ticks/100ms, or about 328 RPM.
             */
            double targetVelocity_UnitsPer100ms = 0.0; 

            if ( m_stick.GetRawButton(5) ) {
               // cout << "button5 " << endl;
               // exactly 146 RPM to the left (-1000 ticks/100ms)
               targetVelocity_UnitsPer100ms = -1000;
            } else if ( m_stick.GetRawButton(6) ) {
               // cout << "button6 " << endl;
               // exactly 146 RPM to the right (1000 ticks/100ms)
               targetVelocity_UnitsPer100ms = 1000.0;
            } else {
                     /* up to 328 RPM (2240 ticks/100ms) in either direction */
               targetVelocity_UnitsPer100ms = m_stick.GetZ() * 4096 * 328.0 / 600;
            }
            if (limev == 1){
               if (limex >= 0.0){
                  targetVelocity_UnitsPer100ms = 750;
               } else {
                  targetVelocity_UnitsPer100ms = -750;
               }
               if (limey >= 0.0){
                  targetVelocity_UnitsPer100ms = targetVelocity_UnitsPer100ms * 2;
               } else {
                  targetVelocity_UnitsPer100ms = targetVelocity_UnitsPer100ms * 0.5;
               } 
            }



               // Display actual/desired speed every 10 iterations
            if ( 0 == (iCallCount%10) )
            {
               m_motorArmMaster.Set( ControlMode::Velocity, targetVelocity_UnitsPer100ms);

               //cout << "speed=";
// #ifdef JAG_NOTDEFINED
               cout << cout.fill(' ');
               cout << cout.width(6) << (int)m_motorArmMaster.GetSelectedSensorVelocity(0);
               cout << "/";
               cout << cout.width(6) << (int)targetVelocity_UnitsPer100ms;
               cout << ", (";
               cout << cout.width(6) << (int)targetVelocity_UnitsPer100ms
                                        - (int)m_motorArmMaster.GetSelectedSensorVelocity(0);
               cout << ", ";
               cout << cout.width(6) << m_motorArmMaster.GetMotorOutputPercent();
               cout << "% )";
               cout << "|";
// #endif
            }
         } else {
            m_motorArmMaster.Set(ControlMode::PercentOutput, m_stick.GetZ());
         }

         if ( 1 == (iCallCount%10) ) {
            //cout << " Dist=";
            //cout << DistSensor1.GetValue();
            // cout << "/";
            // cout << DistSensor1.GetAverageValue();
            // cout << " ( ";
            // cout << DistSensor1.GetVoltage();
            // cout << "/";
            // cout << DistSensor1.GetAverageVoltage();
            // cout << " )";
            cout << limenttable->GetNumber("tx",0.0);
            cout << endl;
	      }

      }

   private:
      frc::Joystick m_stick{0};
      // jagorigwas: frc::Spark m_motor{0};
      WPI_TalonSRX m_motorRSMaster{1}; // Right side drive motor
      WPI_TalonSRX m_motorLSMaster{2}; // Left  side drive motor
      WPI_TalonSRX m_motorArmMaster{7}; // Arm motor

      WPI_VictorSPX m_motorRSSlave1{3};
      WPI_VictorSPX m_motorLSSlave1{4};
      WPI_VictorSPX m_motorLSSlave2{5};
      WPI_VictorSPX m_motorRSSlave2{6};
      int iAutoCount;
      frc::DifferentialDrive m_drive{ m_motorLSMaster, m_motorRSMaster };
      frc::Compressor m_compressor{0};
      frc::DoubleSolenoid m_doublesolenoid{0,1};
      frc::AnalogInput DistSensor1{0};
      std::shared_ptr<NetworkTable> limenttable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
};

// START_ROBOT_CLASS(Robot)

int main() { return frc::StartRobot<Robot>(); }

