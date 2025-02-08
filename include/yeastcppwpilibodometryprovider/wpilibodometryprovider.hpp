#pragma once

#include <iostream>
#include <vector>

#include "yeastcpp/data_structures/swerve_module_config.hpp"

#include "yeastcpp/components/odometry_provider.hpp"

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

namespace yeast_motion
{
    class WPILibOdometryProvider : public OdometryProvider
    {
        public:
            OdometrySample update (std::vector<SwerveModuleStatus> status, Rotation2D gyro_angle) override;
            OdometrySample reset (OdometrySample reset_sample) override;
            void provide_absolute_position_estimate (AbsolutePoseEstimate estimate) override;
            WPILibOdometryProvider(nlohmann::json characterization);

        private:
            std::vector<SwerveModuleConfig> module_configs;
            std::unique_ptr<frc::SwerveDriveKinematics<4>> kinematics;
            std::unique_ptr<frc::SwerveDrivePoseEstimator<4>> estimator;
            wpi::array<frc::SwerveModulePosition, 4> last_wheel_positions;
    };
}