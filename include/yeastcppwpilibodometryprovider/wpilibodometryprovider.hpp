#pragma once

#include <iostream>
#include <vector>

#include "yeastcpp/data_structures/swerve_module_config.hpp"

#include "yeastcpp/components/odometry_provider.hpp"

namespace wpi
{
    template <typename T, size_t N>
    class array;
}

namespace frc
{
    template <size_t NumModules>
    class SwerveDriveKinematics;

    template <size_t NumModules>
    class SwerveDrivePoseEstimator;

    struct SwerveModulePosition;
}

namespace yeast_motion
{
    class WPILibOdometryProvider : public OdometryProvider
    {
        public:
            OdometrySample update (std::vector<SwerveModuleStatus> status, Rotation2D gyro_angle) override;
            OdometrySample get() override;
            OdometrySample reset (OdometrySample reset_sample) override;
            void provide_absolute_position_estimate (AbsolutePoseEstimate estimate) override;
            WPILibOdometryProvider(nlohmann::json characterization);

        private:
            std::vector<SwerveModuleConfig> module_configs;
            std::unique_ptr<frc::SwerveDriveKinematics<4>> kinematics;
            std::unique_ptr<frc::SwerveDrivePoseEstimator<4>> estimator;
            yeast_motion::OdometrySample last_sample;
    };
}