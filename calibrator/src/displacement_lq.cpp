/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
#include "displacement_lq.h"
#include "sphere_parameterization.h"
#include <Accumulate.h>
typedef LIR::Accumulate<double> Accumulate;
typedef std::list<Accumulate> AccumulateRun;

#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#else
#include <lapacke/lapacke.h>
#endif

#include "packed_triangular_matrix.h"

#include <ceres/rotation.h>
#include <map>

#include <Eigen/Cholesky>

struct HingeConstraint1D
{
    const double stddev;

    HingeConstraint1D(const double stddev) : stddev(stddev) {}

    template<typename T>
    bool operator()(const T *q_predInWorld, const T *q_succInWorld,
                    const T *axis_inPred, const T *axis_inSucc,
                    T *residual) const {
        const T q_worldInSucc[4] = { q_succInWorld[0], -q_succInWorld[1], -q_succInWorld[2], -q_succInWorld[3] };
        T q_predInSucc[4];
        ceres::QuaternionProduct(q_worldInSucc, q_predInWorld, q_predInSucc);
        T predAxis_inSucc[3];
        ceres::QuaternionRotatePoint(q_predInSucc, axis_inPred, predAxis_inSucc);

        residual[0] = ceres::DotProduct(predAxis_inSucc, axis_inSucc);
        residual[0] = acos(residual[0]);
        residual[0] /= T(stddev);

        return true;
    }

    static ceres::CostFunction *Create(const double stddev) {
        return new ceres::AutoDiffCostFunction<HingeConstraint1D, 1, 4, 4, 3, 3>(new HingeConstraint1D(stddev));
    }
};

struct RectangularJointAxisJointConstraint
{
    template<typename T>
    bool operator()(const T *r_jointInSensor, const T *axis_inSensor, T *residual) const {
        residual[0] = ceres::DotProduct(r_jointInSensor, axis_inSensor);
        return true;
    }

    static ceres::CostFunction *Create() {
        return new ceres::AutoDiffCostFunction<RectangularJointAxisJointConstraint, 1, 3, 3>(new RectangularJointAxisJointConstraint());
    }
};

struct BodyLengthEqualityConstraint
{
    double distanceSquaredStddev;
    BodyLengthEqualityConstraint(const double distanceStddev)
    : distanceSquaredStddev(sqrt(2) * distanceStddev * distanceStddev)
    {}

    template <typename T>
    bool operator()(const T *r_predInSensorA, const T *r_succInSensorA,
                    const T *r_predInSensorB, const T *r_succInSensorB,
                    T *residual) const {
        T deltaA[3];
        T deltaB[3];
        for (unsigned d = 0; d < 3; ++d) {
            deltaA[d] = r_succInSensorA[d] - r_predInSensorA[d];
            deltaB[d] = r_succInSensorB[d] - r_predInSensorB[d];
        }
        const T distanceASq = ceres::DotProduct(deltaA, deltaA);
        const T distanceBSq = ceres::DotProduct(deltaB, deltaB);
        residual[0] = (distanceBSq - distanceASq) / T(distanceSquaredStddev);
        return true;
    }

    static ceres::CostFunction *Create(double distanceStddev) {
        return new ceres::AutoDiffCostFunction<BodyLengthEqualityConstraint, 1, 3, 3, 3, 3>(new BodyLengthEqualityConstraint(distanceStddev));
    }
};

struct JointConstraintPrior
{
    const Eigen::Vector3d& angular_velocity_pred;
    const Eigen::Vector3d& angular_velocity_succ;
    const double stddev;

    JointConstraintPrior(const Eigen::Vector3d& angular_velocity_pred,
                         const Eigen::Vector3d& angular_velocity_succ,
                         const double stddev)
    : angular_velocity_pred(angular_velocity_pred), angular_velocity_succ(angular_velocity_succ), stddev(stddev) {}

    template<typename T>
    bool operator()(const T *q_pred, const T *v_pred, const T *q_succ, const T *v_succ,
                    const T *r_jointInPredecessor, const T *r_jointInSuccessor,
                    T *residual) const {
        const T omega_pred[3] = {
            T(angular_velocity_pred.x()),
            T(angular_velocity_pred.y()),
            T(angular_velocity_pred.z())
        };
        const T omega_succ[3] = {
            T(angular_velocity_succ.x()),
            T(angular_velocity_succ.y()),
            T(angular_velocity_succ.z())
        };
        T psi_pred[3];
        ceres::CrossProduct(omega_pred, r_jointInPredecessor, psi_pred);
        T psi_pred_world[3];
        ceres::QuaternionRotatePoint(q_pred, psi_pred, psi_pred_world);
        T psi_succ[3];
        ceres::CrossProduct(omega_succ, r_jointInSuccessor, psi_succ);
        T psi_succ_world[3];
        ceres::QuaternionRotatePoint(q_succ, psi_succ, psi_succ_world);

        const T v_joint_pred[3] = {
            v_pred[0] + psi_pred_world[0],
            v_pred[1] + psi_pred_world[1],
            v_pred[2] + psi_pred_world[2],
        };
        const T v_joint_succ[3] = {
            v_succ[0] + psi_succ_world[0],
            v_succ[1] + psi_succ_world[1],
            v_succ[2] + psi_succ_world[2],
        };

        const T stddev_jet(stddev);
        residual[0] = (v_joint_succ[0] - v_joint_pred[0]) / stddev_jet;
        residual[1] = (v_joint_succ[1] - v_joint_pred[1]) / stddev_jet;
        residual[2] = (v_joint_succ[2] - v_joint_pred[2]) / stddev_jet;

        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d& angular_velocity_pred,
                                       const Eigen::Vector3d& angular_velocity_succ,
                                       const double stddev) {
        return new ceres::AutoDiffCostFunction<JointConstraintPrior, 3, 4, 3, 4, 3, 3, 3>(
                new JointConstraintPrior(angular_velocity_pred, angular_velocity_succ, stddev));
    }
};

struct VelocityPrior {
    const double stddev;

    VelocityPrior(const double stddev) : stddev(stddev) {}

    template<typename T>
    bool operator()(const T *velocity, T *residual) const {
        const T stddev_jet(stddev);
        residual[0] = velocity[0] / stddev_jet;
        residual[1] = velocity[1] / stddev_jet;
        residual[2] = velocity[2] / stddev_jet;
        return true;
    }

    static ceres::CostFunction *Create(const double stddev) {
        return new ceres::AutoDiffCostFunction<VelocityPrior, 3, 3>(new VelocityPrior(stddev));
    }
};

struct DynamicAccumulateMeasurement {
    const Accumulate& accumulateDelta;
    const PackedTriangularMatrix<6> inverse_of_cov_sqrt;

    DynamicAccumulateMeasurement(const Accumulate& accumulateDelta,
                                 const PackedTriangularMatrix<6>& inverse_of_cov_sqrt)
    : accumulateDelta(accumulateDelta), inverse_of_cov_sqrt(inverse_of_cov_sqrt) {}

    template<typename T>
    bool operator()(const T *q_pastState, const T *v_pastState, const T *q_futureState, const T *v_futureState, T *residual) const {
        T pastState_Q_worldInImu[4] = { q_pastState[0], -q_pastState[1], -q_pastState[2], -q_pastState[3] };
        T Q_futureInPast[4];
        ceres::QuaternionProduct(pastState_Q_worldInImu, q_futureState, Q_futureInPast);
        T accumulateDelta_Q_inv[4] = { T(accumulateDelta.Q.w()), T(-accumulateDelta.Q.x()), T(-accumulateDelta.Q.y()), T(-accumulateDelta.Q.z()) };
        T deltaQ_quat[4];
        ceres::QuaternionProduct(accumulateDelta_Q_inv, Q_futureInPast, deltaQ_quat);
        T residual_notnormalized[6];
        ceres::QuaternionToAngleAxis(deltaQ_quat, residual_notnormalized);

        const T v_futureInPast_inWorld[3] = {
            v_futureState[0] - v_pastState[0],
            v_futureState[1] - v_pastState[1],
            v_futureState[2] - v_pastState[2] + T(9.81*accumulateDelta.durationSeconds())
        };
        ceres::QuaternionRotatePoint(pastState_Q_worldInImu, v_futureInPast_inWorld, residual_notnormalized+3);
        residual_notnormalized[3] -= T(accumulateDelta.v.x());
        residual_notnormalized[4] -= T(accumulateDelta.v.y());
        residual_notnormalized[5] -= T(accumulateDelta.v.z());
        lower_triangular_packed_matrix_mult<T, 6>(inverse_of_cov_sqrt.elems, residual_notnormalized, residual);

        return true;
    }

    static ceres::CostFunction *Create(const Accumulate& accumulateDelta,
                                       const Eigen::Matrix<double, 6, 6>& covariance) {
        PackedTriangularMatrix<6> inv_of_cov_sqrt;
        inv_of_cov_sqrt.setFromEigenMatrix(covariance);
        char uplo = 'L';
        int n = 6;
        int info = 0;
        dpptrf_(&uplo, &n, inv_of_cov_sqrt.elems, &info);
        if (info != 0) {
            std::cerr << "Could not compute cholesky decomposition of accumulate covariance matrix." << std::endl;
            exit(1);
        }
        char diag = 'N';
        dtptri_(&uplo, &diag, &n, inv_of_cov_sqrt.elems, &info);
        if (info != 0) {
            std::cerr << "Could not compute inverse of lower-triangular Cholesky matrix of accumulate covariance." << std::endl;
            exit(1);
        }
        return new ceres::AutoDiffCostFunction<DynamicAccumulateMeasurement, 6, 4, 3, 4, 3>(new DynamicAccumulateMeasurement(accumulateDelta, inv_of_cov_sqrt));
    }
};

struct HingeError {
    const LIR::IMUAccumulateEntry& entry_succ;
    const LIR::IMUAccumulateEntry& entry_pred;

    HingeError(const LIR::IMUAccumulateEntry& entry_succ, const LIR::IMUAccumulateEntry& entry_pred)
    : entry_succ(entry_succ), entry_pred(entry_pred) {}

    template<typename T>
    bool operator()(const T *axis_inSucc, const T *axis_inPred, T *residual) const {
        /* Two bodies connected over a hinge must have the same angular velocities,
           except for an angular velocity difference along the joint axis. Thus,
           the angular velocity component perpendicular to the joint axis must
           be the same for both bodies. This is only true up to a constant rotation
           matrix, because the orientation of the sensors relative to the bodies are
           unknown. However, rotations do not affect the norms, so the norms of the
           angular velocities perpendicular to the joint axis must be the same
           for both bodies. */

        const T omega_succ[3] = {
            T(entry_succ.angular_velocity.x()),
            T(entry_succ.angular_velocity.y()),
            T(entry_succ.angular_velocity.z())
        };
        const T omega_pred[3] = {
            T(entry_pred.angular_velocity.x()),
            T(entry_pred.angular_velocity.y()),
            T(entry_pred.angular_velocity.z())
        };

        T omega_perp_succ[3];
        ceres::CrossProduct(omega_succ, axis_inSucc, omega_perp_succ);
        T omega_perp_succ_norm2 = ceres::DotProduct(omega_perp_succ, omega_perp_succ);
        T omega_perp_pred[3];
        ceres::CrossProduct(omega_pred, axis_inPred, omega_perp_pred);
        T omega_perp_pred_norm2 = ceres::DotProduct(omega_perp_pred, omega_perp_pred);
        residual[0] = omega_perp_succ_norm2 - omega_perp_pred_norm2;

        return true;
    }

    static ceres::CostFunction *Create(const LIR::IMUAccumulateEntry& entry_succ,
                                       const LIR::IMUAccumulateEntry& entry_pred)
    {
        return new ceres::AutoDiffCostFunction<HingeError, 1, 3, 3>(new HingeError(entry_succ, entry_pred));
    }
};

#if 0
void debug_sphere_parameterization()
{
    Eigen::Vector3d v;
    v.setRandom();
    v.normalize();
    Eigen::Vector2d delta;
    delta.setRandom();
    delta.normalize();
    delta *= M_PI_2;

    Eigen::Vector3d v_plus_delta;
    boxplus<double>(v.data(), delta.data(), v_plus_delta.data());
    Eigen::Vector2d v_plus_delta_minus_v;
    boxminus<double>(v_plus_delta.data(), v.data(), v_plus_delta_minus_v.data());
    std::cout << delta << std::endl << v_plus_delta_minus_v << std::endl;
}
#endif

static void calibrate_hinge(JointSensorMap::SensorLocation& jsmEntrySucc,
                            JointSensorMap::SensorLocation& jsmEntryPred,
                            const LIR::IMUAccumulates& readingsSucc,
                            const LIR::IMUAccumulates& readingsPred,
                            const StandardDeviations& stddevs);

void compute_initial_hinge_axes_guess(
        std::vector<std::shared_ptr<LIR::IMUAccumulates>> readings,
        JointSensorMap& jsm, const StandardDeviations& stddevs)
{
#if 0
    debug_sphere_parameterization();
#endif
    for (unsigned j = 0; j < jsm.numJoints; ++j) {
        JointSensorMap::SensorLocation& predecessor = jsm.sensors[j].predecessor();
        JointSensorMap::SensorLocation& successor = jsm.sensors[j].successor();
        assert (successor.type == predecessor.type);
        if (successor.type != LIR::JointType::hinge)
            continue;
        const LIR::IMUAccumulates& readingsSucc = *readings[successor.sensorId];
        const LIR::IMUAccumulates& readingsPred = *readings[predecessor.sensorId];
        calibrate_hinge(successor, predecessor, readingsSucc, readingsPred, stddevs);
    }

    std::cout << "Listing Hinges:" << std::endl;
    for (int j = 0; j < jsm.numJoints; ++j) {
        const JointSensorMap::SensorLocation& predecessor = jsm.sensors[j].front();
        const JointSensorMap::SensorLocation& successor = jsm.sensors[j].back();
        if (successor.type != LIR::JointType::hinge)
            continue;

        std::cout << "======" << std::endl << "Joint " << j << ':' << std::endl
            << "Axis in sensor-" << predecessor.sensorId << "-coordinates: "
            << predecessor.hingeAxisInSensor.transpose() << std::endl
            << "Axis in sensor-" << successor.sensorId << "-coordinates: "
            << successor.hingeAxisInSensor.transpose() << std::endl;
    }

}

static void calibrate_hinge(JointSensorMap::SensorLocation& jsmEntrySucc,
                            JointSensorMap::SensorLocation& jsmEntryPred,
                            const LIR::IMUAccumulates& readingsSucc,
                            const LIR::IMUAccumulates& readingsPred,
                            const StandardDeviations& stddevs)
{
    ceres::LocalParameterization *sphere_parameterization = new SphereParameterization;
    ceres::Problem problem;

    const std::list<LIR::IMUAccumulateEntry>::const_iterator end_succ = readingsSucc.entries.end();
    std::list<LIR::IMUAccumulateEntry>::const_iterator it_succ = readingsSucc.entries.begin();
    const std::list<LIR::IMUAccumulateEntry>::const_iterator end_pred = readingsPred.entries.end();
    std::list<LIR::IMUAccumulateEntry>::const_iterator it_pred = readingsPred.entries.begin();
    double axis_inSucc[3] = { 0.0, 1.0, 0.0 };
    double axis_inPred[3] = { 0.0, 1.0, 0.0 };

    for (; it_succ != end_succ && it_pred != end_pred; ++it_succ, ++it_pred) {
        problem.AddResidualBlock(HingeError::Create(*it_succ, *it_pred), NULL, axis_inSucc, axis_inPred);
    }
    problem.SetParameterization(axis_inSucc, sphere_parameterization);
    problem.SetParameterization(axis_inPred, sphere_parameterization);
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    jsmEntryPred.hingeAxisInSensor.x() = axis_inPred[0];
    jsmEntryPred.hingeAxisInSensor.y() = axis_inPred[1];
    jsmEntryPred.hingeAxisInSensor.z() = axis_inPred[2];
    jsmEntrySucc.hingeAxisInSensor.x() = axis_inSucc[0];
    jsmEntrySucc.hingeAxisInSensor.y() = axis_inSucc[1];
    jsmEntrySucc.hingeAxisInSensor.z() = axis_inSucc[2];
}

struct CeresJointLocation {
    unsigned joint_id;
    unsigned pred_sensor_id;
    unsigned succ_sensor_id;
    double r_jointInPredecessor[3];
    double r_jointInSuccessor[3];
};
typedef std::vector<CeresJointLocation> JointLocations;

struct CeresJointAxis {
    unsigned pred_sensor_id;
    unsigned succ_sensor_id;
    double axis_inPredecessor[3];
    double axis_inSuccessor[3];
};

struct SensorState {
    double q_imuInWorld[4];
    double v_inWorld[3];
    const Accumulate& accumulateDelta; /*< Relative accumulate that led to this state. */

    SensorState(const Accumulate& accumulateDelta)
        : accumulateDelta(accumulateDelta) {}
};

struct SensorStateRun {
    unsigned sensorId;
    std::list<SensorState> states;
};

/* Mapping of a hinge's joint id to the hinge axis
   both in predecessor and successor coordinates. */
typedef std::map<unsigned, CeresJointAxis> HingeMap;
typedef std::list<Eigen::Vector3d> AngularVelocityRun;

void test_helper_functions()
{
    /* For fun and testing, compute the cholesky factorization with LAPACK of a symmetric matrix. */
    Eigen::Matrix<double, 6, 6> X;
    X.setRandom();
    Eigen::Matrix<double, 6, 6> A = X.transpose() * X;

    PackedTriangularMatrix<6> A_packed;
    A_packed.setFromEigenMatrix(A);
    for (unsigned r = 0; r < 6; ++r)
        for (unsigned c = 0; c < 6; ++c)
            assert (std::abs(A(r, c) - A_packed(r, c)) < 1e-10);

    char uplo = 'L';
    int n = 6;
    int info = 0;
    dpptrf_(&uplo, &n, A_packed.elems, &info);

    Eigen::Matrix<double, 6, 6> L = A.llt().matrixL();
    std::cout << "EIGEN RESULT" << std::endl << L << std::endl << std::endl;
    std::cout << "LAPACK(PACKED) RESULT" << std::endl << A_packed.toEigenMatrix() << std::endl << std::endl;

    char diag = 'N';
    dtptri_(&uplo, &diag, &n, A_packed.elems, &info);
    std::cout << "EIGEN RESULT" << std::endl << L.inverse() << std::endl << std::endl;
    std::cout << "LAPACK(PACKED) RESULT" << std::endl << A_packed.toEigenMatrix() << std::endl;

    /* Test vector multiplication with packed matrix (without LAPACK, because it also has to work with the Jet type. */
    Eigen::Matrix<double, 6, 1> v(Eigen::Matrix<double, 6, 1>::Random());
    Eigen::Matrix<double, 6, 1> Av(Eigen::Matrix<double, 6, 1>::Zero());
    Eigen::Matrix<double, 6, 1> Av_fast(Av);

    const Eigen::Matrix<double, 6, 1> Av_test = A_packed.toEigenMatrix() * v;
    lower_triangular_packed_matrix_mult<double, 6>(A_packed.elems, v.data(), Av_fast.data());
    assert ((Av_test - Av_fast).norm() < 1e-10);
}

static Eigen::Matrix3d crossx(const Eigen::Vector3d& v)
{
    return (Eigen::Matrix3d() <<
            0.0,    -v(2),  v(1),
            v(2),   0.0,    -v(0),
            -v(1),  v(0),   0.0).finished();
}

static void covariance_for_accumulate(Eigen::Matrix<double, 6,6>& covariance, const Accumulate& a,
                                      const struct StandardDeviations& stddevs)
{
    // Compute variance densities.
    const double a_duration_sec = a.durationSeconds();
    const double varomega_density = stddevs.angular_velocity_density * stddevs.angular_velocity_density;
    const double varaccel_density = stddevs.acceleration_density * stddevs.acceleration_density;
    const Eigen::Matrix3d vx = crossx(a.v);
    covariance << Eigen::Matrix3d::Identity() * a_duration_sec, vx * a_duration_sec / 2.0f,
    vx.transpose() * a_duration_sec / 2.0f, vx.transpose() * vx * a_duration_sec / 3.0f;
    covariance *= varomega_density;
    covariance.bottomRightCorner<3, 3>() +=
    Eigen::Matrix3d::Identity() * varaccel_density * a_duration_sec;
}

void run_calibrator(const std::vector<AccumulateRun>& accumulateDeltaRuns,
                    const std::vector<AngularVelocityRun>& angularVelocityRuns,
                    std::vector<TimedSensorStateRun>& imuTrajectories,
                    JointSensorMap& jsm,
                    const struct StandardDeviations& stddevs,
                    const std::list<Symmetry>& symmetries,
                    bool calibrate_with_hinges)
{
#if 0
    test_helper_functions();
#endif

    const unsigned numSensors = accumulateDeltaRuns.size();

    ceres::LocalParameterization *sphere_parameterization = new SphereParameterization;
    ceres::LocalParameterization *quaternion_parameterization = new ceres::QuaternionParameterization;
    ceres::Problem problem;

    std::vector<SensorStateRun> sensorStateRuns; // sensorStateRuns[i] is the evolution of the state of sensor i.
    sensorStateRuns.reserve(numSensors);
    const Eigen::Vector3d gravity(0.0, 0.0, -9.81);

    // Add initial states. Because timestamps are needed for this, this wastes the first measurements of each IMU.
    std::vector<AccumulateRun::const_iterator> imus, ends;
    for (unsigned i = 0; i < numSensors; ++i) {
        imus.push_back(accumulateDeltaRuns[i].begin());
        ends.push_back(accumulateDeltaRuns[i].end());

        SensorState s(*imus[i]);
        if (calibrate_with_hinges) {
            const TimedSensorState& initial_tss = *imuTrajectories[i].front();
            s.q_imuInWorld[0] = initial_tss.q_imuInWorld.w();
            s.q_imuInWorld[1] = initial_tss.q_imuInWorld.x();
            s.q_imuInWorld[2] = initial_tss.q_imuInWorld.y();
            s.q_imuInWorld[3] = initial_tss.q_imuInWorld.z();
            s.v_inWorld[0] = initial_tss.v_inWorld.x();
            s.v_inWorld[1] = initial_tss.v_inWorld.y();
            s.v_inWorld[2] = initial_tss.v_inWorld.z();
        } else {
            /* Find initial orientation quaternion from the first measurement.
               Since it uses gravity only, only the inclination is determined,
               while the heading stays undetermined. If the sensor isn't moved
               during the first accumulation period, v integrates -gravity, i.e.
               it points upwards, the direction of -gravity. */
            const Eigen::Vector3d& v_inImu = imus[i]->v; /**< Velocity of the first accumulate interval. */
            const Eigen::Vector3d v_inWorld = -gravity * imus[i]->durationSeconds();
            Eigen::Quaterniond q_initial;
            q_initial.setFromTwoVectors(v_inImu, v_inWorld);
            s.q_imuInWorld[0] = q_initial.w();
            s.q_imuInWorld[1] = q_initial.x();
            s.q_imuInWorld[2] = q_initial.y();
            s.q_imuInWorld[3] = q_initial.z();
            s.v_inWorld[0] = 0.0;
            s.v_inWorld[1] = 0.0;
            s.v_inWorld[2] = 0.0;

            { // Assert that construction of initial state worked.
                Eigen::Vector3d v_inWorldTest;
                ceres::QuaternionRotatePoint(s.q_imuInWorld, v_inImu.data(), v_inWorldTest.data());
                assert((v_inWorldTest.normalized() - v_inWorld.normalized()).norm() < 1e-6);
            }
        }

        SensorStateRun run;
        run.sensorId = i;
        run.states.push_back(s);
        ++imus[i];
        sensorStateRuns.push_back(run);
    }

    for (unsigned i = 0; i < numSensors; ++i) {
        SensorStateRun& run = sensorStateRuns[i];
        assert (run.states.size() == 1);
        SensorState& initial_state = run.states.back();
        problem.AddParameterBlock(initial_state.q_imuInWorld, 4);
        problem.SetParameterization(initial_state.q_imuInWorld, quaternion_parameterization);
        problem.AddParameterBlock(initial_state.v_inWorld, 3);
    }

    // Add runs for each sensor and relate them by relative measurements.
    for (unsigned i = 0; i < numSensors; ++i) {
        TimedSensorStateRun::const_iterator trajectory_it;
        if (calibrate_with_hinges) {
            trajectory_it = imuTrajectories[i].begin();
            ++trajectory_it; // First entry was used to generate the initial state.
        }

        SensorStateRun& run = sensorStateRuns[i];
        AccumulateRun::const_iterator& it = imus[i];
        AccumulateRun::const_iterator& end = ends[i];
        for (; it != end; ++it) {
            const Accumulate& accumulateDelta = *it;
            Eigen::Matrix<double, 6, 6> accumulateDeltaCov;
            covariance_for_accumulate(accumulateDeltaCov, accumulateDelta, stddevs);
            SensorState& paststate = run.states.back();
            run.states.push_back(SensorState(accumulateDelta));
            SensorState& futurestate = run.states.back();

            if (calibrate_with_hinges) {
                const TimedSensorState& tss = **trajectory_it;
                futurestate.q_imuInWorld[0] = tss.q_imuInWorld.w();
                futurestate.q_imuInWorld[1] = tss.q_imuInWorld.x();
                futurestate.q_imuInWorld[2] = tss.q_imuInWorld.y();
                futurestate.q_imuInWorld[3] = tss.q_imuInWorld.z();
                futurestate.v_inWorld[0] = tss.v_inWorld[0];
                futurestate.v_inWorld[1] = tss.v_inWorld[1];
                futurestate.v_inWorld[2] = tss.v_inWorld[2];
                ++trajectory_it;
            } else {
                const double deltaT = accumulateDelta.durationSeconds();
                assert (deltaT > 0.0);
                const double accumulateDelta_q[4] = {
                    accumulateDelta.Q.w(),
                    accumulateDelta.Q.x(),
                    accumulateDelta.Q.y(),
                    accumulateDelta.Q.z()
                };
                ceres::QuaternionProduct(paststate.q_imuInWorld,
                                         accumulateDelta_q,
                                         futurestate.q_imuInWorld);
                double delta_v_inWorld[3];
                ceres::QuaternionRotatePoint(paststate.q_imuInWorld, accumulateDelta.v.data(), delta_v_inWorld);
                futurestate.v_inWorld[0] = paststate.v_inWorld[0] + delta_v_inWorld[0];
                futurestate.v_inWorld[1] = paststate.v_inWorld[1] + delta_v_inWorld[1];
                futurestate.v_inWorld[2] = paststate.v_inWorld[2] + delta_v_inWorld[2] + gravity.z() * deltaT;
            }
            problem.AddResidualBlock(DynamicAccumulateMeasurement::Create(accumulateDelta, accumulateDeltaCov),
                                     NULL, paststate.q_imuInWorld, paststate.v_inWorld, futurestate.q_imuInWorld, futurestate.v_inWorld);
            problem.SetParameterization(futurestate.q_imuInWorld, quaternion_parameterization);

        }
    }

    // Add the zero-velocity prior
    for (SensorStateRun& run : sensorStateRuns) {
        std::list<SensorState>::iterator it = run.states.begin();
        const std::list<SensorState>::const_iterator end = run.states.end();
        problem.AddResidualBlock(VelocityPrior::Create(stddevs.velocity),
                                 NULL, (it++)->v_inWorld);
        for (; it != end; ++it) {
            const double deltaT = it->accumulateDelta.durationSeconds();
            const double zerovel_stddev = stddevs.velocity * sqrt(1.0 + 2.0 * stddevs.velocity_decorrelation_time/deltaT);
            problem.AddResidualBlock(VelocityPrior::Create(zerovel_stddev),
                                     NULL, it->v_inWorld);
        }
    }

    // Add pseudo-measurements for all sensor coupled by joints.
    const long syncusecs = 10000; // Microseconds states may be apart in time are still considered 'synchronous'.
    HingeMap hingeMap;
    JointLocations jointLocations;
    jointLocations.reserve(jsm.numJoints);
    for (const JointSensorMap::Joint& jointSensors : jsm.sensors) {
        const JointSensorMap::SensorLocation& predecessor = jointSensors.predecessor();
        const JointSensorMap::SensorLocation& successor = jointSensors.successor();
        const bool is_hinge = predecessor.type == LIR::JointType::hinge;
        assert (!is_hinge || successor.type == JointType::hinge);

        // Initialize joint location for joint j.
        jointLocations.push_back(CeresJointLocation());
        CeresJointLocation& jloc = jointLocations.back();
        jloc.joint_id = jointSensors.getJointId();
        jloc.pred_sensor_id = predecessor.sensorId;
        jloc.succ_sensor_id = successor.sensorId;
        for (unsigned d = 0; d < 3; ++d) {
            jloc.r_jointInPredecessor[d] = predecessor.jointInSensor[d];
            jloc.r_jointInSuccessor[d] = successor.jointInSensor[d];
        }

        HingeMap::iterator hinge;
        if (is_hinge && calibrate_with_hinges) {
            hingeMap.emplace(jloc.joint_id, CeresJointAxis());
            hinge = hingeMap.find(jloc.joint_id);
            assert (hinge != hingeMap.end());
            hinge->second.pred_sensor_id = jloc.pred_sensor_id;
            hinge->second.succ_sensor_id = jloc.succ_sensor_id;
            for (unsigned d = 0; d < 3; ++d) {
                hinge->second.axis_inPredecessor[d] = predecessor.hingeAxisInSensor[d];
                hinge->second.axis_inSuccessor[d] = successor.hingeAxisInSensor[d];
            }

            problem.AddResidualBlock(RectangularJointAxisJointConstraint::Create(), NULL,
                                     jloc.r_jointInPredecessor, hinge->second.axis_inPredecessor);
            problem.AddResidualBlock(RectangularJointAxisJointConstraint::Create(), NULL,
                                     jloc.r_jointInSuccessor, hinge->second.axis_inSuccessor);
            problem.SetParameterization(hinge->second.axis_inPredecessor, sphere_parameterization);
            problem.SetParameterization(hinge->second.axis_inSuccessor, sphere_parameterization);

        }

        const AngularVelocityRun& angvel_pred = angularVelocityRuns[jloc.pred_sensor_id];
        const AngularVelocityRun& angvel_succ = angularVelocityRuns[jloc.succ_sensor_id];
        AngularVelocityRun::const_iterator angvel_itpred = angvel_pred.begin();
        AngularVelocityRun::const_iterator angvel_itsucc = angvel_succ.begin();

        SensorStateRun& staterun_pred = sensorStateRuns[jloc.pred_sensor_id];
        SensorStateRun& staterun_succ = sensorStateRuns[jloc.succ_sensor_id];
        std::list<SensorState>::iterator state_itpred = staterun_pred.states.begin();
        const std::list<SensorState>::const_iterator state_endpred = staterun_pred.states.end();
        std::list<SensorState>::iterator state_itsucc = staterun_succ.states.begin();
        const std::list<SensorState>::const_iterator state_endsucc = staterun_succ.states.end();
        for (; state_itpred != state_endpred && state_itsucc != state_endsucc;
             ++state_itpred, ++state_itsucc, ++angvel_itpred, ++angvel_itsucc) {
            assert (angvel_itsucc != angvel_succ.end());
            assert (angvel_itpred != angvel_pred.end());

            const unsigned deviation = std::abs(static_cast<long>(state_itpred->accumulateDelta.duration)
                                                - static_cast<long>(state_itsucc->accumulateDelta.duration));
            if (deviation > syncusecs) {
                std::cerr << "Two delta of sensors " << jloc.pred_sensor_id
                          << " and " << jloc.succ_sensor_id
                          << " accumulates assumed to be of equal lengths differ in duration by "
                          << deviation << " usecs." << std::endl;
                if (state_itpred->accumulateDelta.duration < state_itsucc->accumulateDelta.duration - 50000) {
                    std::cerr << "Looks like " << jloc.succ_sensor_id << " skipped an accumulate " << jloc.pred_sensor_id << " got." << std::endl;
                    ++state_itpred;
                } else if(state_itsucc->accumulateDelta.duration < state_itpred->accumulateDelta.duration - 50000) {
                    std::cerr << "Looks like " << jloc.pred_sensor_id << " skipped an accumulate " << jloc.succ_sensor_id << " got." << std::endl;
                    ++state_itsucc;
                }
            } else {
                const double deltaT = 0.5 * (state_itpred->accumulateDelta.durationSeconds()
                                             + state_itsucc->accumulateDelta.durationSeconds());
                const double jointconstraint_stddev = stddevs.joint_velocity_difference * sqrt(1.0 + 2.0 * stddevs.joint_velocity_difference_decorrelation_time / deltaT);
                problem.AddResidualBlock(JointConstraintPrior::Create(*angvel_itpred, *angvel_itsucc, jointconstraint_stddev), NULL,
                                         state_itpred->q_imuInWorld, state_itpred->v_inWorld,
                                         state_itsucc->q_imuInWorld, state_itsucc->v_inWorld,
                                         jloc.r_jointInPredecessor, jloc.r_jointInSuccessor);

                if (is_hinge && calibrate_with_hinges) {
                    assert (hinge != hingeMap.end());
                    const double hingeaxis_stddev = stddevs.joint_axis_difference
                        * sqrt(1.0 + 2.0 * stddevs.joint_axis_difference_decorrelation_time / deltaT);
                    problem.AddResidualBlock(HingeConstraint1D::Create(hingeaxis_stddev), NULL,
                                             state_itpred->q_imuInWorld,
                                             state_itsucc->q_imuInWorld,
                                             hinge->second.axis_inPredecessor,
                                             hinge->second.axis_inSuccessor);
                    problem.SetParameterization(hinge->second.axis_inPredecessor, sphere_parameterization);
                    problem.SetParameterization(hinge->second.axis_inSuccessor, sphere_parameterization);
                }
            }
        }
    }

    // Add pseudo-measurements of bodies which should have equal lengths.
    for (const Symmetry& s : symmetries) {
        assert(jointLocations[s.a.preceeding_joint_id].succ_sensor_id == s.a.body_id);
        assert(jointLocations[s.a.succeeding_joint_id].pred_sensor_id == s.a.body_id);
        assert(jointLocations[s.b.preceeding_joint_id].succ_sensor_id == s.b.body_id);
        assert(jointLocations[s.b.succeeding_joint_id].pred_sensor_id == s.b.body_id);
        problem.AddResidualBlock(BodyLengthEqualityConstraint::Create(stddevs.symmetric_body_length_difference), NULL,
            jointLocations[s.a.preceeding_joint_id].r_jointInSuccessor, jointLocations[s.a.succeeding_joint_id].r_jointInPredecessor,
            jointLocations[s.b.preceeding_joint_id].r_jointInSuccessor, jointLocations[s.b.succeeding_joint_id].r_jointInPredecessor);
    }

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    // Get calibration resultls
    // Joint-Sensor-Map
    JointLocations::const_iterator jloc_it = jointLocations.begin();
    for (unsigned j = 0; j < jsm.numJoints; ++j, ++jloc_it) {
        assert (jloc_it != jointLocations.end());
        JointSensorMap::SensorLocation& succ = jsm.sensors[j].successor();
        JointSensorMap::SensorLocation& pred = jsm.sensors[j].predecessor();
        const CeresJointLocation& jloc = *jloc_it;
        assert (j == jloc.joint_id);
        assert (succ.sensorId == jloc.succ_sensor_id);
        assert (pred.sensorId == jloc.pred_sensor_id);

        for (unsigned d = 0; d < 3; ++d) {
            pred.jointInSensor[d] = jloc.r_jointInPredecessor[d];
            succ.jointInSensor[d] = jloc.r_jointInSuccessor[d];
        }

        if (pred.type == LIR::JointType::hinge && calibrate_with_hinges) {
            assert(succ.type == JointType::hinge);
            HingeMap::const_iterator hinge = hingeMap.find(j);
            assert (hinge != hingeMap.end());
            assert(pred.sensorId == hinge->second.pred_sensor_id);
            assert(succ.sensorId == hinge->second.succ_sensor_id);
            for (unsigned d = 0; d < 3; ++d) {
                pred.hingeAxisInSensor[d] = hinge->second.axis_inPredecessor[d];
                succ.hingeAxisInSensor[d] = hinge->second.axis_inSuccessor[d];
            }
        }
    }

    // Save the calibrated sensors runs.
    imuTrajectories.clear();
    imuTrajectories.reserve(numSensors);
    for (unsigned i = 0; i < numSensors; ++i) {
        const SensorStateRun& staterun = sensorStateRuns[i];
        TimedSensorStateRun run;
        unsigned usecs = 0;

        for (const SensorState& s : staterun.states) {
            usecs += s.accumulateDelta.duration;
            const double seconds = usecs * 1e-6;
            Eigen::Quaterniond q_imuInWorld;
            q_imuInWorld.w() = s.q_imuInWorld[0];
            q_imuInWorld.x() = s.q_imuInWorld[1];
            q_imuInWorld.y() = s.q_imuInWorld[2];
            q_imuInWorld.z() = s.q_imuInWorld[3];
            Eigen::Vector3d v_inWorld;
            v_inWorld[0] = s.v_inWorld[0];
            v_inWorld[1] = s.v_inWorld[1];
            v_inWorld[2] = s.v_inWorld[2];
            TimedSensorState *tss = new TimedSensorState(q_imuInWorld, v_inWorld, seconds);
            run.push_back(std::shared_ptr<TimedSensorState>(tss));
        }
        imuTrajectories.push_back(run);
    }
}

void calibrate_displacements(std::vector<std::shared_ptr<LIR::IMUAccumulates>> readings,
                             std::vector<TimedSensorStateRun>& imuTrajectories,
                             JointSensorMap& jsm,
                             const struct StandardDeviations& stddevs,
                             const std::list<Symmetry>& symmetries)
{
    const unsigned numSensors = readings.size();
    /* Compute the set of difference accumulates for each sensor.
       To do so, first turn the IMUAccumulates into AccumulateRuns, then
       compute the deltas.
       In addition, the angular velocities at the end of each delta are extracted.
       These are needed to apply joint constraints at time t after integrating
       the deltas up to time t. (Practically this means that the first angular
       velocity measurements are dropped). */
    std::vector<AccumulateRun> accumulateDeltaRuns;
    std::vector<AngularVelocityRun> angularVelocityRuns;
    accumulateDeltaRuns.resize(numSensors);
    angularVelocityRuns.resize(numSensors);
    for (unsigned i = 0; i < numSensors; ++i) {
        AngularVelocityRun& avelrun = angularVelocityRuns[i];
        AccumulateRun run;
        for (const LIR::IMUAccumulateEntry& e : readings[i]->entries) {
            run.push_back(Accumulate(e.orientation, e.velocity, e.usecs));
            avelrun.push_back(e.angular_velocity);
        }
        AccumulateRun& deltas = accumulateDeltaRuns[i];
        compute_delta_accumulate_run(deltas, run);
        avelrun.pop_front();
        assert (deltas.size() == readings[i]->entries.size()-1);
        assert (deltas.size() == avelrun.size());
    }

    run_calibrator(accumulateDeltaRuns, angularVelocityRuns, imuTrajectories, jsm, stddevs, symmetries, false);
    // Check joint axes.
    for (JointSensorMap::Joint& js : jsm.sensors) {
        JointSensorMap::SensorLocation& pred = js.predecessor();
        const JointSensorMap::SensorLocation& succ = js.successor();
        if (pred.type != LIR::JointType::hinge)
            continue;

        const TimedSensorStateRun& run_pred = imuTrajectories[pred.sensorId];
        const TimedSensorStateRun& run_succ = imuTrajectories[succ.sensorId];
        const TimedSensorState& first_pred = *run_pred.front();
        const TimedSensorState& first_succ = *run_succ.front();

        double angle;
        do {
            const Eigen::Quaterniond q_succInPred = first_pred.q_imuInWorld.inverse() * first_succ.q_imuInWorld;
            Eigen::Vector3d succaxis_inPred = q_succInPred * succ.hingeAxisInSensor;
            std::cout << "=== JOINT " << js.getJointId() << " ===" << std::endl;
            std::cout << "Pred Axis In Pred: " << pred.hingeAxisInSensor.transpose() << std::endl;
            std::cout << "Succ Axis In Pred: " << succaxis_inPred.transpose() << std::endl;
            Eigen::Vector2d delta;
            boxminus<double>(succaxis_inPred.data(), pred.hingeAxisInSensor.data(), delta.data());
            angle = delta.norm();
            std::cout << "Succ Axis In Pred BOXMINUS Pred Axis In Pred: " << delta.transpose() << " (" << (angle/M_PI*180.0) << ')' << std::endl;
            if (angle > M_PI_2) {
                pred.hingeAxisInSensor = -pred.hingeAxisInSensor;
            }
        } while (angle > M_PI_2);
    }
    run_calibrator(accumulateDeltaRuns, angularVelocityRuns, imuTrajectories, jsm, stddevs, symmetries, true);
}
