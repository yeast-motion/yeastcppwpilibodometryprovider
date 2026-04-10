# yeastcppwpilibodometryprovider - Issues

## Bugs / Correctness Problems

### 1. `update()` does not validate `status` vector size
In `src/wpilibodometryprovider.cpp:87`, the method takes `std::vector<SwerveModuleStatus> status` and immediately indexes `status[0]` through `status[3]` (lines 90-103) without checking that the vector has at least 4 elements. A short vector causes out-of-bounds access.

### 2. Typo in log output: "etimator" instead of "estimator"
`src/wpilibodometryprovider.cpp:75`: `"Using custom etimator std devs."` -- minor but visible to users.

### 3. `provide_absolute_position_estimate` timestamp semantics are ambiguous
In `src/wpilibodometryprovider.cpp:153`, the code computes `frc::Timer::GetTimestamp() - units::time::millisecond_t(estimate.timestamp)`. This assumes `estimate.timestamp` is a latency/age in milliseconds. But `AbsolutePoseEstimate.timestamp` is documented in the schema as "A timestamp of the sample" which suggests an absolute time, not a relative offset. The interpretation is inconsistent. If `timestamp` is an age-in-ms, the field name is misleading. If it is an absolute timestamp, subtracting it from `GetTimestamp()` is incorrect if they use different epochs.

### 4. `acceleration_valid` is never set in `update()`
In `src/wpilibodometryprovider.cpp:118-142`, the `result` `OdometrySample` sets `pose_valid = true` and `velocity_valid = true`, but `acceleration_valid` is left at its default (`false`), and `acceleration` is left at default zeros. However, the acceleration field is populated with default `Twist2D` values, which may confuse consumers who check `acceleration_valid`.

### 5. `last_wheel_positions` is stored but never used for acceleration computation
`src/wpilibodometryprovider.cpp:138` stores `last_wheel_positions` each update cycle, but these previous positions are never read back to compute acceleration or validate position deltas.

### 6. Hardcoded 4-module assumption (same as drive controller)
`wpilibodometryprovider.hpp:27-28` hardcodes `SwerveDriveKinematics<4>` and `SwerveDrivePoseEstimator<4>`. Constructor validates size at runtime (line 31) but the template parameter cannot change.

### 7. Shadowed variable `zero_position`
`src/wpilibodometryprovider.cpp:18` declares a file-static `frc::SwerveModulePosition zero_position`, and then `src/wpilibodometryprovider.cpp:49` declares a local `frc::SwerveModulePosition zero_position` inside the constructor that shadows the static one. The static one is used in the initializer list (line 21) while the local one is used in the estimator constructor (line 60-64). The static one is default-constructed (not explicitly zeroed), which may or may not default to zero depending on the WPILib implementation.

## Code Smells

### 8. `stdout` logging instead of a proper logging framework
Lines 75-78 and 82-83 use `std::cout` for logging. In a ROS2 context, this should use `RCLCPP_INFO` or similar.

### 9. Hardcoded default std_devs magic numbers
`src/wpilibodometryprovider.cpp:83` hardcodes `{0.75, 0.75, 0.75}` with no explanation of why these values were chosen.

### 10. CMakeLists.txt uses `GLOB_RECURSE` and relative include paths
Same issues as the drive controller: `GLOB_RECURSE` picks up build dir, and `../yeastcpp/include` is fragile.
