#pragma once

#include <iostream>
#include <vector>

#include "yeastcpp/data_structures/swerve_module_config.hpp"

#include "yeastcpp/components/odometry_provider.hpp"

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

namespace yeast_motion
{
    class WPILibOdometryProvider : public OdometryProvider
    {
        public:
            OdometrySample update (std::vector<SwerveModuleStatus> status, Rotation2D gyro_angle) override;
            OdometrySample reset (OdometrySample reset_sample) override;
            WPILibOdometryProvider(nlohmann::json characterization);

        private:
            std::vector<SwerveModuleConfig> module_configs;
            std::unique_ptr<frc::SwerveDriveKinematics<4>> kinematics;
            std::unique_ptr<frc::SwerveDriveOdometry<4>> odometry;
    };
}