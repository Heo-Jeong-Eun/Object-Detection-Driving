// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file LaneKeepingSystem.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief Lane Keeping System Class source file
 * @version 1.1
 * @date 2023-05-02
 */
#include "LaneKeepingSystem/LaneKeepingSystem.hpp"

namespace Xycar {
template <typename PREC>
LaneKeepingSystem<PREC>::LaneKeepingSystem()
{
    std::string configPath;
    mNodeHandler.getParam("config_path", configPath);
    YAML::Node config = YAML::LoadFile(configPath);

    mPID = std::make_unique<PIDController<PREC>>(config["PID"]["P_GAIN"].as<PREC>(), config["PID"]["I_GAIN"].as<PREC>(), config["PID"]["D_GAIN"].as<PREC>());
    mMovingAverage = std::make_unique<MovingAverageFilter<PREC>>(config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<uint32_t>());
    mHoughTransformLaneDetector = std::make_unique<HoughTransformLaneDetector<PREC>>(config);
    mVehicleModel = std::make_unique<VehicleModel<PREC>>(0, 0, 0);
    setParams(config);

    mPublisher = mNodeHandler.advertise<xycar_msgs::xycar_motor>(mPublishingTopicName, mQueueSize);
    mSubscriber = mNodeHandler.subscribe(mSubscribedTopicName, mQueueSize, &LaneKeepingSystem::imageCallback, this);

    mVehicleStatePublisher = mNodeHandler.advertise<std_msgs::Float32MultiArray>("/vehicle_state", 1);
    mLanePositionPublisher = mNodeHandler.advertise<std_msgs::Float32MultiArray>("/lane_position", 1);

    mStanley = std::make_unique<StanleyController<PREC>>(mStanleyGain, mStanleyLookAheadDistance);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::setParams(const YAML::Node& config)
{
    mPublishingTopicName = config["TOPIC"]["PUB_NAME"].as<std::string>();
    mSubscribedTopicName = config["TOPIC"]["SUB_NAME"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    mXycarSpeed = config["XYCAR"]["START_SPEED"].as<PREC>();
    mXycarMaxSpeed = config["XYCAR"]["MAX_SPEED"].as<PREC>();
    mXycarMinSpeed = config["XYCAR"]["MIN_SPEED"].as<PREC>();
    mXycarSpeedControlThreshold = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<PREC>();
    mAccelerationStep = config["XYCAR"]["ACCELERATION_STEP"].as<PREC>();
    mDecelerationStep = config["XYCAR"]["DECELERATION_STEP"].as<PREC>();
    mStanleyGain = config["STANLEY"]["K_GAIN"].as<PREC>();
    mStanleyLookAheadDistance = config["STANLEY"]["LOOK_AHREAD_DISTANCE"].as<PREC>();
    mDebugging = config["DEBUG"].as<bool>();

    mLinearUnit = config["XYCAR"]["LINEAR_UNIT"].as<PREC>();
    mAngleUnit = config["XYCAR"]["ANGLE_UNIT"].as<PREC>();
}

template <typename PREC>
void LaneKeepingSystem<PREC>::run()
{
    ros::Rate rate(kFrameRate);
    ros::Time currentTime, previousTime, pubTime;
    ros::Time now = ros::Time::now();
    PREC previousSteeringAngle = 0.f;
    PREC gradientLeftX = -0.1;
    PREC gradientRightX = 0.1;

    currentTime = now;
    previousTime = now;
    pubTime = now;

    while (ros::ok())
    {
        ros::spinOnce();
        if (mFrame.empty())
            continue;

        auto [leftPositionX, rightPositionX] = mHoughTransformLaneDetector->getLanePosition(mFrame);

        currentTime = ros::Time::now();
        ros::Duration delta_t = currentTime - previousTime;
        mVehicleModel->update(mXycarSpeed / mLinearUnit, previousSteeringAngle * M_PI / 180.f, static_cast<double>(delta_t.toNSec()) / 1000000.f / 1000.f);

        int32_t estimatedPositionX = static_cast<int32_t>((leftPositionX + rightPositionX) / 2);
        int32_t errorFromMid = estimatedPositionX - static_cast<int32_t>(mFrame.cols / 2);

        mStanley->calculateSteeringAngle(errorFromMid, 0, mXycarSpeed);

        PREC stanleyResult = mStanley->getResult();
        PREC steeringAngle = std::max(static_cast<PREC>(-kXycarSteeringAangleLimit), std::min(static_cast<PREC>(stanleyResult), static_cast<PREC>(kXycarSteeringAangleLimit)));
        std::cout << "error: " << errorFromMid << std::endl;
        std::cout << "steeringAngle: " << stanleyResult << std::endl;

        speedControl(steeringAngle);
        drive(steeringAngle);

        if (mDebugging)
        {
            std_msgs::Float32MultiArray vehicleStateMsg;
            std_msgs::Float32MultiArray lanePositionMsg;

            std::tuple<PREC, PREC, PREC> vehicleState = mVehicleModel->getResult();

            vehicleStateMsg.data.push_back(std::get<0>(vehicleState));
            vehicleStateMsg.data.push_back(std::get<1>(vehicleState));
            vehicleStateMsg.data.push_back(std::get<2>(vehicleState));
            vehicleStateMsg.data.push_back(leftPositionX);
            vehicleStateMsg.data.push_back(rightPositionX);

            ros::Duration pubDiff = ros::Time::now() - pubTime;

            if (pubDiff.toSec() > 0.1)
            {
                mVehicleStatePublisher.publish(vehicleStateMsg);
                mLanePositionPublisher.publish(lanePositionMsg);
                pubTime = ros::Time::now();
            }

            // std::cout << "lpos: " << leftPositionX << ", rpos: " << rightPositionX << ", mpos: " << estimatedPositionX << std::endl;
            mHoughTransformLaneDetector->drawRectangles(leftPositionX, rightPositionX, estimatedPositionX);
            cv::imshow("Debug", mHoughTransformLaneDetector->getDebugFrame());
            cv::waitKey(1);
        }

        previousTime = ros::Time::now();
        previousSteeringAngle = steeringAngle;
    }
}

template <typename PREC>
void LaneKeepingSystem<PREC>::imageCallback(const sensor_msgs::Image& message)
{
    cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
    cv::cvtColor(src, mFrame, cv::COLOR_RGB2BGR);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::speedControl(PREC steeringAngle)
{
    if (std::abs(steeringAngle) > mXycarSpeedControlThreshold)
    {
        mXycarSpeed -= mDecelerationStep;
        mXycarSpeed = std::max(mXycarSpeed, mXycarMinSpeed);
        return;
    }

    mXycarSpeed += mAccelerationStep;
    mXycarSpeed = std::min(mXycarSpeed, mXycarMaxSpeed);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::drive(PREC steeringAngle)
{
    xycar_msgs::xycar_motor motorMessage;

    motorMessage.header.stamp = ros::Time::now();
    motorMessage.angle = std::round(steeringAngle);
    motorMessage.speed = std::round(mXycarSpeed);

    // motorMessage.angle = 0;
    // motorMessage.speed = 5;

    mPublisher.publish(motorMessage);
}

template class LaneKeepingSystem<float>;
template class LaneKeepingSystem<double>;
} // namespace Xycar
