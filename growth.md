# yeastcppwpilibodometryprovider - Growth Opportunities

## Improvement Opportunities

### 1. Compute acceleration from successive velocity measurements
The `last_wheel_positions` field is stored but never used to derive acceleration. Given that `update()` returns `acceleration_valid = false`, computing numerical acceleration from successive velocity readings would fill this gap.

### 2. Add thread safety
`update()`, `get()`, `reset()`, and `provide_absolute_position_estimate()` can be called from different threads (e.g., sensor callbacks vs. control loop). There is no mutex or atomic protection around `last_sample`, `estimator`, or `last_wheel_positions`.

### 3. Parameterize the default vision standard deviations
The hardcoded `{0.75, 0.75, 0.75}` at line 83 should be a named constant with documentation explaining the choice, or better yet, always required through the config.

### 4. Support non-swerve odometry
The `OdometryProvider` interface takes `vector<SwerveModuleStatus>`, coupling it to swerve drives. A more general interface could support differential, mecanum, or other drivetrain types.

## Architectural Enhancements

### 5. Make `reset()` also reset `last_wheel_positions`
`src/wpilibodometryprovider.cpp:171-183`: `reset()` calls `estimator->ResetPose()` and updates `last_sample`, but does not reset `last_wheel_positions`. On the next `update()`, the stale wheel positions could cause a discontinuity.

### 6. Clarify and unify timestamp semantics
Document whether `AbsolutePoseEstimate.timestamp` represents an absolute epoch time, a ROS time, or a latency offset. The current subtraction `GetTimestamp() - millisecond_t(timestamp)` is error-prone.

## Testing Gaps

### 7. No unit tests
Critical paths with no test coverage:
- `update()` with known module states, verify pose output
- `reset()` followed by `get()`, verify position matches reset value
- `provide_absolute_position_estimate()` with various timestamps
- Constructor with missing or malformed `std_devs` config
- Empty or short `status` vector passed to `update()`
