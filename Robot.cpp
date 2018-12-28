/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <IterativeRobot.h>
#include <Joystick.h>
#include <Spark.h>
#include "ctre/Phoenix.h"
#include <Compressor.h>
#include <DoubleSolenoid.h>
#include <DriverStation.h>
#include <Drive/DifferentialDrive.h>
#include <iostream>

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * Joystick analog values range from -1 to 1 and speed controller inputs as
 * range from -1 to 1 making it easy to work together.
 */
class Robot : public frc::IterativeRobot {
   public:
      void AutonomousInit() override {
         iAutoCount = 0;
         m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kForward);
      }

      void AutonomousPeriodic() override {
         if ( iAutoCount < 100 ) {
           m_drive.CurvatureDrive( 0.5, -0.5, 0 );
         } else if ( iAutoCount < 200 ) {
           m_drive.CurvatureDrive( 0.5, 0.5, 0 );
         } else if ( iAutoCount < 201 ) {
           m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
         } else if ( iAutoCount < 700 ) {
           m_drive.CurvatureDrive( 0.5, -0.5, 0 );
      // } else if ( iAutoCount < 700 ) {
      //   m_drive.CurvatureDrive( 0.5, 0.5, 0 );                                       }
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
         m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kForward);
      }

      void TeleopPeriodic() override {
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
                                       m_stick.GetTop() );
            }
         }
          m_compressor.SetClosedLoopControl(true);
          // Compressor *c = new Compressor(0);
          // m_doublesolenoid.Set(DoubleSolenoid.Value.kOff);
          if ( m_stick.GetTrigger() ) {
             m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
          } else {
             m_doublesolenoid.Set(frc::DoubleSolenoid::Value::kForward);
          }
          // m_doublesolenoid.Set(DoubleSolenoid::Value::kReverse);
      }

   private:
      frc::Joystick m_stick{0};
      // jagorigwas: frc::Spark m_motor{0};
      WPI_TalonSRX m_motorRSMaster{1};
      WPI_TalonSRX m_motorLSMaster{2};
      WPI_VictorSPX m_motorRSSlave1{3};
      WPI_VictorSPX m_motorLSSlave1{4};
      WPI_VictorSPX m_motorLSSlave2{5};
      WPI_VictorSPX m_motorRSSlave2{6};
      int iAutoCount;
      DifferentialDrive m_drive{ m_motorLSMaster, m_motorRSMaster };
      Compressor m_compressor{0};
      DoubleSolenoid m_doublesolenoid{0,1};
};

START_ROBOT_CLASS(Robot)
