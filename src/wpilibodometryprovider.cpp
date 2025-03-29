#include <exception>

#include "yeastcppwpilibodometryprovider/wpilibodometryprovider.hpp"

#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveDriveOdometry.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Pose2d.h"
#include "wpilibc/frc/Timer.h"
#include "wpimath/wpimath/MathShared.h"

using namespace yeast_motion;

WPILibOdometryProvider::WPILibOdometryProvider(nlohmann::json characterization)
{
    for (auto motor_config : characterization["MotorConfig"])
    {
        SwerveModuleConfig config;
        config.translation.x = motor_config["x"];
        config.translation.y = motor_config["y"];
        module_configs.push_back(config);
    }

    if (module_configs.size() != 4)
    {
        throw std::runtime_error("Only configurations with 4 swerve modules are supported by the Yeast WPI Lib Odometry Provider");
    }

    frc::Translation2d m_frontLeftLocation  {units::meter_t(module_configs[0].translation.x), units::meter_t(module_configs[0].translation.y)};
    frc::Translation2d m_frontRightLocation {units::meter_t(module_configs[1].translation.x), units::meter_t(module_configs[1].translation.y)};
    frc::Translation2d m_backLeftLocation   {units::meter_t(module_configs[2].translation.x), units::meter_t(module_configs[2].translation.y)};
    frc::Translation2d m_backRightLocation  {units::meter_t(module_configs[3].translation.x), units::meter_t(module_configs[3].translation.y)};

    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
    kinematics.reset(new frc::SwerveDriveKinematics<4>
        (m_frontLeftLocation,
        m_frontRightLocation,
        m_backLeftLocation,
        m_backRightLocation));

    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html#creating-the-odometry-object
    frc::SwerveModulePosition zero_position;
    zero_position.angle = 0_rad;
    zero_position.distance = 0_m;

    // Creating my estimator object from the kinematics object. Here,
    // our starting pose is 5 meters along the long end of the field and in the
    // center of the field along the short end, facing forward.
    estimator.reset(new frc::SwerveDrivePoseEstimator<4>
        (*(kinematics.get()),
        frc::Rotation2d(0_rad),
        {
            zero_position,
            zero_position,
            zero_position,
            zero_position
        },
        frc::Pose2d (0_m, 0_m, 0_rad)));

    if (characterization.contains("std_devs") &&
        characterization["std_devs"].is_array() &&
        characterization["std_devs"].size() == 3)
    {
        estimator->SetVisionMeasurementStdDevs (
            { characterization["std_devs"][0],
              characterization["std_devs"][1],
              characterization["std_devs"][2] } );
        std::cout << "Using custom etimator std devs. " <<
            characterization["std_devs"][0] << ", " <<
            characterization["std_devs"][1] << ", " <<
            characterization["std_devs"][2] << std::endl;
    }
    else
    {
        std::cout << "Using default Limelight std devs. .5, .5, 9999999" << std::endl;
        // https://github.com/LimelightVision/limelight-examples/blob/main/java-wpilib/swerve-megatag-odometry/src/main/java/frc/robot/Drivetrain.java#L121
        estimator->SetVisionMeasurementStdDevs( {0.75, 0.75, 9999999} );
    }
}

OdometrySample WPILibOdometryProvider::update (std::vector<SwerveModuleStatus> status, Rotation2D gyro_angle)
{
    frc::SwerveModulePosition fl;
    fl.angle = frc::Rotation2d(units::radian_t(status[0].theta));
    fl.distance = units::meter_t(status[0].position);

    frc::SwerveModulePosition fr;
    fr.angle = frc::Rotation2d(units::radian_t(status[1].theta));
    fr.distance = units::meter_t(status[1].position);

    frc::SwerveModulePosition bl;
    bl.angle = frc::Rotation2d(units::radian_t(status[2].theta));
    bl.distance = units::meter_t(status[2].position);

    frc::SwerveModulePosition br;
    br.angle = frc::Rotation2d(units::radian_t(status[3].theta));
    br.distance = units::meter_t(status[3].position);

    wpi::array<frc::SwerveModulePosition, 4> wheel_positions =
    {
        fl,
        fr,
        bl,
        br
    };

    // https://github.com/wpilibsuite/allwpilib/blob/638d265b339435c7f7af530f84a3e242500f75ce/wpilibcExamples/src/main/cpp/examples/SwerveDrivePoseEstimator/cpp/Drivetrain.cpp#L34
    estimator->Update(frc::Rotation2d
        (units::radian_t (gyro_angle.theta)),
        wheel_positions);

    OdometrySample result;
    frc::Pose2d pose = estimator->GetEstimatedPosition();
    result.pose_valid = true;
    result.pose.translation.x = pose.Translation().X().value();
    result.pose.translation.y = pose.Translation().Y().value();
    result.pose.rotation.theta = pose.Rotation().Radians().value();

    wpi::array < frc::SwerveModuleState, 4 > module_states {
            frc::SwerveModuleState { units::velocity::meters_per_second_t(status[0].speed), units::angle::radian_t(status[0].theta) },
            frc::SwerveModuleState { units::velocity::meters_per_second_t(status[1].speed), units::angle::radian_t(status[1].theta) },
            frc::SwerveModuleState { units::velocity::meters_per_second_t(status[2].speed), units::angle::radian_t(status[2].theta) },
            frc::SwerveModuleState { units::velocity::meters_per_second_t(status[3].speed), units::angle::radian_t(status[3].theta) }
    };

    auto chassis_speeds = kinematics->ToChassisSpeeds(module_states);
    result.velocity.x = chassis_speeds.vx.value();
    result.velocity.y = chassis_speeds.vy.value();
    result.velocity.omega = chassis_speeds.omega.value();
    result.velocity_valid = true;

    last_sample = result;

    return result;
}

void WPILibOdometryProvider::provide_absolute_position_estimate (AbsolutePoseEstimate estimate)
{
    // https://github.com/wpilibsuite/allwpilib/blob/638d265b339435c7f7af530f84a3e242500f75ce/wpilibcExamples/src/main/cpp/examples/SwerveDrivePoseEstimator/cpp/Drivetrain.cpp#L41

    estimator->AddVisionMeasurement(
        frc::Pose2d(units::meter_t(estimate.pose.translation.x),
                units::meter_t(estimate.pose.translation.y),
                frc::Rotation2d(units::radian_t(estimate.pose.rotation.theta))),
                frc::Timer::GetTimestamp() - units::time::millisecond_t(estimate.timestamp));
}

OdometrySample WPILibOdometryProvider::get()
{
    return last_sample;
}

OdometrySample WPILibOdometryProvider::reset (OdometrySample reset_sample)
{
    frc::Pose2d reset_pose;
    reset_pose = reset_pose.TransformBy(
        frc::Transform2d(
            units::meter_t(reset_sample.pose.translation.x),
            units::meter_t(reset_sample.pose.translation.y),
            frc::Rotation2d(units::radian_t(reset_sample.pose.rotation.theta))));

    estimator->ResetPose(reset_pose);
    last_sample = reset_sample;
    return last_sample;
}
