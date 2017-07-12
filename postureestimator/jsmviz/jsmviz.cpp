/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
/**
 * Copyright (c) 2017 Felix Wenk <felixwenk@googlemail.com>
 */

#include "jsmviz.h"

#include <OrientationFromMotion/joint_sensor_map.h>

#include <osg/ShapeDrawable>
#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osgViewer/Viewer>
#include <osg/PositionAttitudeTransform>
#include <osgGA/TrackballManipulator>

#include <Eigen/Dense>
#include <array>

osg::Geode *create_joint_geode()
{
    osg::TessellationHints *hints = new osg::TessellationHints;
    hints->setDetailRatio(1.0f);
    osg::ShapeDrawable *joint_shape = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0, 0.0, 0.0), 0.02), hints);
    joint_shape->setDataVariance(osg::Object::STATIC);
    joint_shape->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    osg::Geode *joint_geode = new osg::Geode;
    joint_geode->addDrawable(joint_shape);
    joint_geode->setDataVariance(osg::Object::STATIC);
    return joint_geode;
}

osg::Geode *create_sensor_geode()
{
    osg::ShapeDrawable *sensor_shape = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0, 0.0, 0.0), 0.02));
    sensor_shape->setDataVariance(osg::Object::STATIC);
    sensor_shape->setColor(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    osg::Geode *sensor_geode = new osg::Geode;
    sensor_geode->addDrawable(sensor_shape);
    sensor_geode->setDataVariance(osg::Object::STATIC);

    return sensor_geode;
}

osg::Geode *create_connector(float length, float thickness, const osg::Vec4& color)
{
    osg::Shape *cylinder_shape = new osg::Cylinder(osg::Vec3f(0.0f, 0.0f, 0.0f), thickness, length);
    cylinder_shape->setDataVariance(osg::Object::STATIC);
    osg::ShapeDrawable *cylinder = new osg::ShapeDrawable(cylinder_shape);
    cylinder->setColor(color);
    cylinder->setDataVariance(osg::Object::STATIC);

    osg::Geode *geode = new osg::Geode;
    geode->addDrawable(cylinder);
    geode->setDataVariance(osg::Object::STATIC);
    return geode;
}

osg::Geode *create_joint_sensor_connector(const JointSensorMap& jsm, const unsigned joint_idx, const bool tosuccessor)
{
    const Eigen::Vector3f& jointInSensor
        = tosuccessor ? jsm.sensors[joint_idx].back().jointInSensor : jsm.sensors[joint_idx].front().jointInSensor;
    return create_connector(jointInSensor.norm(), 0.0025f, osg::Vec4(0.0, 0.0, 1.0, 0.7));
}

template <unsigned long n>
unsigned index_of_computable_joint(const JointSensorMap& jsm,
       const std::array<bool, n>& computed_sensor,
       const std::array<bool, n-1>& computed_joint)
{
    constexpr unsigned m = n-1;
    for (unsigned j=0; j<m; ++j) {
        if (computed_joint[j])
            continue;
        const std::list<SensorLocation>& joint = jsm.sensors[j];
        const int psid = joint.front().sensorId; /* Index of preceeding sensor. */
        const int ssid = joint.back().sensorId; /* Index of succeeding senosr. */
        if (computed_sensor[psid] || computed_sensor[ssid])
            return j;
    }
    assert(false && "Do not call this function if there are no joints with at least one known sensor position.");
    return -1;
}

template <unsigned long n>
void compute_joint_and_sensor_positions(const JointSensorMap& jsm,
        const std::array<std::shared_ptr<SharedOrientationf>, n>& sso,
        std::array<std::shared_ptr<Eigen::Vector3f>, n>& sensor_pos,
        std::array<std::shared_ptr<Eigen::Vector3f>, n-1>& joint_pos)
{
    static_assert(n > 2, "Body two is the pinned body. Thus this function does not support skeletons with less than 3 bodies.");
    std::array<bool, n> computed_sensor{};
    std::array<bool, n-1> computed_joint{};
    computed_sensor[2] = true;
    if (!sensor_pos[2])
        sensor_pos[2]=std::make_shared<Eigen::Vector3f>(Eigen::Vector3f::Zero());

    for (unsigned i = 0; i < n-1; ++i) {
        const unsigned j = index_of_computable_joint(jsm, computed_sensor, computed_joint);
        const SensorLocation& ploc = jsm.sensors[j].front(); /* Location of the sensor preceeding the joint. */
        const SensorLocation& sloc = jsm.sensors[j].back(); /* Location of the sensor succeeding the joint. */
        /* Location of the sensor whose position in world coordinates is known. */
        const SensorLocation& known_loc = computed_sensor[ploc.sensorId] ? ploc : sloc;
        /* Location of the sensor whose position in world coordinates is not known. */
        const SensorLocation& unknown_loc = computed_sensor[sloc.sensorId] ? ploc : sloc;
        assert(computed_sensor[known_loc.sensorId]);
        assert(!computed_sensor[unknown_loc.sensorId]);

        if (!sensor_pos[unknown_loc.sensorId])
            sensor_pos[unknown_loc.sensorId] = std::make_shared<Eigen::Vector3f>();
        if (!joint_pos[j])
            joint_pos[j] = std::make_shared<Eigen::Vector3f>();
        const Eigen::Matrix3f Q_unknownInWorld = sso[unknown_loc.sensorId]->get_data();
        const Eigen::Matrix3f Q_knownInWorld = sso[known_loc.sensorId]->get_data();
        *(joint_pos[j]) = *(sensor_pos[known_loc.sensorId]) + Q_knownInWorld * known_loc.jointInSensor;
        *(sensor_pos[unknown_loc.sensorId]) = *(joint_pos[j]) - Q_unknownInWorld * unknown_loc.jointInSensor;
        computed_joint[j] = true;
        computed_sensor[unknown_loc.sensorId] = true;
    }

    for (unsigned k=0; k<n-1; ++k) {
        assert(computed_joint[k]);
        assert(computed_sensor[k]);
    }
    assert(computed_sensor[n-1]);
}

class UpdateConnector : public osg::NodeCallback
{
    const std::shared_ptr<Eigen::Vector3f>& position_a;
    const std::shared_ptr<Eigen::Vector3f>& position_b;
public:
    UpdateConnector(const std::shared_ptr<Eigen::Vector3f>& position_a,
                    const std::shared_ptr<Eigen::Vector3f>& position_b)
    : position_a(position_a), position_b(position_b)
    {}

    virtual void operator()(osg::Node *node, osg::NodeVisitor *nv)
    {
        if (!position_a || !position_b)
            return;

        const Eigen::Vector3f& bpos = *position_b;
        const Eigen::Vector3f& apos = *position_a;

        osg::PositionAttitudeTransform *pose = dynamic_cast<osg::PositionAttitudeTransform *>(node);
        const Eigen::Vector3f delta = bpos - apos;
        const float zsign = (delta.z() > 0.0f) - (delta.z() < 0.0f);
        const float dlen = delta.norm();
        const Eigen::Vector3f center = apos + delta / 2.0;
        const float alen = sqrt(delta.x()*delta.x() + delta.y()*delta.y());
        if (fabs(alen) > 0.1f/180.0f*M_PI) {
            osg::Vec3 axis(-delta.y() / alen, delta.x() / alen, 0.0f);
            float angle = std::asin(zsign*alen / dlen);
            pose->setAttitude(osg::Quat(angle, axis));
        } else {
            pose->setAttitude(osg::Quat(0.0f, 0.0f, 0.0f, 1.0f));
        }

        pose->setPosition(osg::Vec3f(center.x(), center.y(), center.z()));

        traverse(node, nv);
    }
};

template <unsigned long n>
class UpdatePositions : public osg::NodeCallback
{
    const JointSensorMap& jsm;
    std::array<std::shared_ptr<SharedOrientationf>, n>& shared_orientations;
    std::array<std::shared_ptr<Eigen::Vector3f>, n>& sensor_positions;
    std::array<std::shared_ptr<Eigen::Vector3f>, n-1>& joint_positions;
public:
    UpdatePositions(const JointSensorMap& jsm,
                    std::array<std::shared_ptr<SharedOrientationf>, n>& sso,
                    std::array<std::shared_ptr<Eigen::Vector3f>, n>& sensor_pos,
                    std::array<std::shared_ptr<Eigen::Vector3f>, n-1>& joint_pos)
    : jsm(jsm), shared_orientations(sso), sensor_positions(sensor_pos), joint_positions(joint_pos)
    {}

    virtual void operator()(osg::Node *node, osg::NodeVisitor *nv)
    {
        if (std::all_of(shared_orientations.begin(), shared_orientations.end(),
                        [](const std::shared_ptr<SharedOrientationf>& so) {
                            if (!so)
                                return false;
                            return so->is_valid();
                        }))
        {
            compute_joint_and_sensor_positions(jsm, shared_orientations, sensor_positions, joint_positions);
        }
        traverse(node, nv);
    }
};

class SinglePositionUpdate : public osg::NodeCallback
{
    const std::shared_ptr<Eigen::Vector3f>& position;
public:
    SinglePositionUpdate(const std::shared_ptr<Eigen::Vector3f>& position) : position(position)
    {}

    virtual void operator()(osg::Node *node, osg::NodeVisitor *nv)
    {
        if (position) {
            osg::PositionAttitudeTransform *pose = dynamic_cast<osg::PositionAttitudeTransform *>(node);
            pose->setPosition(osg::Vec3f(position->x(), position->y(), position->z()));
        }
        traverse(node, nv);
    }
};

template <unsigned long n>
void jsmviz(const JointSensorMap& jsm, std::array<std::shared_ptr<SharedOrientationf>, n>& sso)
{
    /* Create basic geometry. */
    osg::Geode *joint_geode = create_joint_geode();
    osg::Geode *sensor_geode = create_sensor_geode();
    /* Create pointers to sensor positions and joint positions, both in world coordinates. */
    std::array<std::shared_ptr<Eigen::Vector3f>, n> sensor_pos;
    std::array<std::shared_ptr<Eigen::Vector3f>, n-1> joint_pos;

    osg::Group *root = new osg::Group;
    root->addUpdateCallback(new UpdatePositions<n>(jsm, sso, sensor_pos, joint_pos));

    /* Create sensors. */
    for (unsigned k=0; k<n; ++k) {
        osg::PositionAttitudeTransform *sensor = new osg::PositionAttitudeTransform;
        sensor->setUpdateCallback(new SinglePositionUpdate(sensor_pos[k]));
        sensor->addChild(sensor_geode);
        sensor->setDataVariance(osg::Object::DYNAMIC);
        root->addChild(sensor);
    }
    /* Create joints. */
    for (unsigned k=0; k<n-1; ++k) {
        osg::PositionAttitudeTransform *joint = new osg::PositionAttitudeTransform;
        joint->setUpdateCallback(new SinglePositionUpdate(joint_pos[k]));
        joint->addChild(joint_geode);
        joint->setDataVariance(osg::Object::DYNAMIC);
        root->addChild(joint);
    }
    /* Create connections between joints and sensors. */
    for (unsigned k=0; k<n-1; ++k) {
        const std::list<SensorLocation>& locs = jsm.sensors[k];
        osg::PositionAttitudeTransform *connector_with_succ = new osg::PositionAttitudeTransform;
        const unsigned sidx = locs.back().sensorId;
        osg::Geode *succ = create_joint_sensor_connector(jsm, k, true);
        connector_with_succ->addChild(succ);
        connector_with_succ->addUpdateCallback(new UpdateConnector(joint_pos[k], sensor_pos[sidx]));
        root->addChild(connector_with_succ);

        osg::PositionAttitudeTransform *connector_with_pred = new osg::PositionAttitudeTransform;
        const unsigned pidx = locs.front().sensorId;
        osg::Geode *pred = create_joint_sensor_connector(jsm, k, false);
        connector_with_pred->addChild(pred);
        connector_with_pred->addUpdateCallback(new UpdateConnector(joint_pos[k], sensor_pos[pidx]));
        root->addChild(connector_with_pred);
    }
    /* Create connections between adjacent joints. */
    for (unsigned k=0; k<n-1; ++k) {
        const SensorLocation& kpred = jsm.sensors[k].front();
        const SensorLocation& ksucc = jsm.sensors[k].back();
        for (unsigned l=k+1; l<n-1; ++l) {
            const SensorLocation& lpred = jsm.sensors[l].front();
            const SensorLocation& lsucc = jsm.sensors[l].back();
            Eigen::Vector3f delta;
            if (kpred.sensorId == lpred.sensorId)
                delta = kpred.jointInSensor - lpred.jointInSensor;
            else if (kpred.sensorId == lsucc.sensorId)
                delta = kpred.jointInSensor - lsucc.jointInSensor;
            else if (ksucc.sensorId == lpred.sensorId)
                delta = ksucc.jointInSensor - lpred.jointInSensor;
            else if (ksucc.sensorId == lsucc.sensorId)
                delta = ksucc.jointInSensor - lsucc.jointInSensor;
            else
                continue;
            osg::Geode *connector_geode = create_connector(delta.norm(), 0.0035f, osg::Vec4(1.0, 0.0, 0.0, 0.7));
            osg::PositionAttitudeTransform *connector = new osg::PositionAttitudeTransform;
            connector->addChild(connector_geode);
            connector->addUpdateCallback(new UpdateConnector(joint_pos[k], joint_pos[l]));
            root->addChild(connector);
        }
    }


    /* Create osg viewer and render scene graph. */
    osgViewer::Viewer viewer;
    const float h = 2.f;
    osg::Vec3f eye(h, h, h);
    osg::Vec3f center(0.0f, 0.0f, 0.0f);
    osg::Vec3f up(0.0f, 0.0f, 1.0f);
    viewer.setCameraManipulator(new osgGA::TrackballManipulator());
    viewer.getCameraManipulator()->setHomePosition(eye, center, up);
    viewer.setUpViewInWindow(0, 0, 1024, 768);
    viewer.setSceneData( root );
    osg::Camera *camera = viewer.getCamera();
    camera->setClearColor(osg::Vec4(1.0f,1.0f,1.0f,0.0f));

    viewer.setSceneData(root);
    viewer.run();
}

template void jsmviz<15>(const JointSensorMap& jsm, std::array<std::shared_ptr<SharedOrientationf>, 15>& sso);
