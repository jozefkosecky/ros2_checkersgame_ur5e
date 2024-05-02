#include "robot_control/RobotControlNode.hpp"


using namespace std::chrono_literals;

RobotControlNode::RobotControlNode()
        : Node("robot_control_node"),
          tf_buffer(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tf_listener(std::make_shared<tf2_ros::TransformListener>(*tf_buffer)) {


    // publisher
    chessBoardPub = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    robotMovePub = this->create_publisher<checkers_msgs::msg::RobotMove>("robot_move_topic", 10);

    // subcriber
    checkersBoardSub = this->create_subscription<checkers_msgs::msg::Board>(
        "board_topic", 10, std::bind(&RobotControlNode::checkers_board_callback, this, std::placeholders::_1));

    checkersMoveSub = this->create_subscription<checkers_msgs::msg::Move>(
        "move_topic", 10, std::bind(&RobotControlNode::checkers_move_callback, this, std::placeholders::_1));
        
    handDetectedSub = this->create_subscription<checkers_msgs::msg::HandDetected>(
        "hand_detected", 10, std::bind(&RobotControlNode::hand_detected_callback, this, std::placeholders::_1));

    handDetectedLMSub = this->create_subscription<std_msgs::msg::String>(
            "leap_gesture", 10, std::bind(&RobotControlNode::hand_detected_lm_callback, this, std::placeholders::_1));

    rosOutSub = this->create_subscription<rcl_interfaces::msg::Log>(
        "/rosout",10,std::bind(&RobotControlNode::ros_out_callback, this, std::placeholders::_1));

    palmPositionLMSub = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/leap_gesture/palm_position",10,std::bind(&RobotControlNode::palm_position_lm_callback, this, std::placeholders::_1));

    serviceResumeMovement = this->create_service<checkers_msgs::srv::ResumeMovement>(
        "resume_movement", std::bind(&RobotControlNode::handle_resume_movement, this, std::placeholders::_1, std::placeholders::_2));


    // Service
    gripperServiceClient = this->create_client<gripper_srv::srv::GripperService>("gripper_service");
    while (!gripperServiceClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    

    square_size = 0.047;  // Size of each square

    // DON'T FORGET. BOAR IS ROTATED ABOUT 90 degrees
    boardOffsetX = (square_size * 12) - 0.0396;
    boardOffsetY = 0.3 - 0.0438;

    zAttach = 0.15908388793468475;
    zMoving = 19.5;


    // geometry_msgs::msg::Pose target_pose1;
    // // target_pose1.orientation.x = 0.5;
    // // target_pose1.orientation.y = 0.5;
    // // target_pose1.orientation.z = -0.5;
    // // target_pose1.orientation.w = 0.5;

    // target_pose1.orientation.w = 0.0;
    // target_pose1.orientation.x = 1.0;
    // target_pose1.orientation.y = 0.0; //-5.563795639318414e-06
    // target_pose1.orientation.z = 0.0; //-0.000838900392409414
    // target_pose1.position.x = 0.4765383303165436;
    // target_pose1.position.y = -0.16665436327457428;
    // target_pose1.position.z = 0.04123930260539055;

    // // Second Pose
    // geometry_msgs::msg::Pose target_pose2;
    // target_pose2.orientation.x = 0.0;
    // target_pose2.orientation.y = 1.0;
    // target_pose2.orientation.z = 0.0;
    // target_pose2.orientation.w = 0.0;
    // target_pose2.position.x = 0.4765383303165436;
    // target_pose2.position.y = -0.16665436327457428 + (square_size*7);
    // target_pose2.position.z = 0.04123930260539055;

    // pose_list.push_back(target_pose1);
    // pose_list.push_back(target_pose2);
    // target_pose = pose_list[target_pose_index];
    

}

void RobotControlNode::initMoveGroup() {
    using moveit::planning_interface::MoveGroupInterface;
    move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    move_group_interface->setEndEffectorLink("tool0");
    move_group_interface->setPoseReferenceFrame("base_link");
    move_group_interface->setMaxVelocityScalingFactor(0.8);
    move_group_interface->setMaxAccelerationScalingFactor(0.04);
    move_group_interface->startStateMonitor();



//    executor.add_node(shared_from_this());
//
//    std::thread([&]() {
//        executor.spin();
//    }).detach();

    getPose();

//    std::ofstream outfile;
//    outfile.open("/home/collab/collab_ws/src/robot_control/src/" + currentFileName, std::ios_base::app); // Open in append mode
//    if (!outfile.is_open()) {
//        std::cerr << "Failed to open file: " << currentFileName << std::endl;
//    }
//    outfile << currentPosition.position.x << ","
//            << currentPosition.position.y << ","
//            << currentPosition.position.z << "," << std::endl;
//    outfile.close();



    openGripper();
    publishCheckerboard();
    
    // createPiece(0, 0);
    // attachPiece();
    // usleep(2000000);
    // detachPiece();
    // removePiece();


    // targetPositions.push_back(Mission(0, 0, Task::ATTACH));
    // targetPositions.push_back(Mission(0, 5, Task::DETACH));

    // trajectory_list = getPoseList(targetPositions[target_pose_index]);
    // target_pose = trajectory_list[trajectory_pose_index].first;
    // move(target_pose);

    // move(target_pose);



        // HOME POSITION
//    geometry_msgs::msg::Pose pose;
//    pose.orientation.x = -0.0028119066264480352;
//    pose.orientation.y = 0.9999957084655762;
//    pose.orientation.z = -0.0007648332393728197;
//    pose.orientation.w = -0.00023792324645910412;
//    pose.position.x = 0.0 - 0.0438;
//    pose.position.y = 0.3;
//    pose.position.z = 0.35;
//    pose.position.x = 1.186 + 0.0396;
//    pose.position.y = 0.2015 + 0.0438;
//    pose.position.z = 0.3- 0.0107;   // pre zdvich pridaj 0.195. Rozdiel medzi tool0 hodnotou a mojou je -0.0107
//    target_pose = pose;
//    moveInThread(target_pose);

//    openGripper();
//
    // Suradnice dosky ked som prvy krat skusal realnu hraciu plochu
//    geometry_msgs::msg::Pose pose1;
//    pose1.orientation.x = -0.0028119066264480352;
//    pose1.orientation.y = 0.9999957084655762;
//    pose1.orientation.z = -0.0007648332393728197;
//    pose1.orientation.w = -0.00023792324645910412;
//    pose1.position.x = (square_size * 12) - 0.0396;
//    pose1.position.y = 0.3 - 0.0438;
//    pose1.position.z = 0.159 - 0.0107;   // pre zdvich pridaj 0.195. Rozdiel medzi tool0 hodnotou a mojou je -0.0107
//    //    poses.push_back(std::make_pair(pose1, Task::NONE));
//    target_pose = pose1;
//    isRobotMoving = true;
//    moveInThread(target_pose);


    //HOME

//    geometry_msgs::msg::Pose pose;
//    pose.orientation.x = -0.0028119066264480352;
//    pose.orientation.y = 0.9999957084655762;
//    pose.orientation.z = -0.0007648332393728197;
//    pose.orientation.w = -0.00023792324645910412;
//    pose.position.x = 0.0 - 0.0438;
//    pose.position.y = 0.3;
//    pose.position.z = 0.35;
//    target_pose = pose;
//    moveInThread(target_pose);

//    std::this_thread::sleep_for(std::chrono::seconds(5));
//    closeGripper();
//    openGripper();


//    pose1.orientation.x = -0.0028119066264480352;
//    pose1.orientation.y = 0.9999957084655762;
//    pose1.orientation.z = -0.0007648332393728197;
//    pose1.orientation.w = -0.00023792324645910412;
//    pose1.position.x = (square_size * 12) - 0.0396;
//    pose1.position.y = 0.3 - 0.0438;
//    pose1.position.z = 0.19;   // pre zdvich pridaj 0.195. Rozdiel medzi tool0 hodnotou a mojou je -0.0107
////    poses.push_back(std::make_pair(pose1, Task::NONE));
//    moveInThread(pose1);
//
//    std::this_thread::sleep_for(std::chrono::seconds(5));
//    openGripper();
}

void RobotControlNode::openGripper() {
    auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();
    request->position = 0;
    request->speed = 5;
    request->force = 5;
    auto result = gripperServiceClient->async_send_request(request);
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

void RobotControlNode::closeGripper() {
    auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();
    request->position = 117; // ak bude vypadavat tak daj 118
    request->speed = 5;
    request->force = 5;
    auto result = gripperServiceClient->async_send_request(request);
    std::this_thread::sleep_for(std::chrono::seconds(3));
}

void RobotControlNode::mainLoop() {
    getPose();
//    return;
    if(isStop || isRobotMoving) {
        // std::cout << "Robot sa hybe"<< std::endl;
        return;
    }
    
    if(targetPositions.empty() || doingTask) {
        // std::cout << "Nemam ziadnu misiu alebo robim task."<< std::endl;
        // std::cout << "-------------------------------" << std::endl;

        return;
    }

    if(trajectory_pose_index >= 0 && static_cast<std::size_t>(trajectory_pose_index) < trajectory_list.size()) {
        RCLCPP_WARN(shared_from_this()->get_logger(), "\n\nDalsi bod na trajektori.");
        std::cout << "Size: " << trajectory_list.size() << std::endl;
        std::cout << "aktualny index ciastocneho pohybu: " << trajectory_pose_index << std::endl;
    
        // Taking task at the end of the move.
        Task task = trajectory_list[trajectory_pose_index].second;
        if(task != Task::NONE) {
            std::cout << "Robim TASK" << std::endl;
            std::cout << "Robim TASK" << std::endl;
            Mission mission = targetPositions[target_pose_index];
            makeTask(mission);  
            return;
        }
        
        target_pose = trajectory_list[trajectory_pose_index].first;

        std::cout << "posielam robotu informaciu o pohybe" << std::endl;
        isRobotMoving = true;
        moveInThread(target_pose);

        trajectory_pose_index++;
        std::cout << "dalsi index ciastocneho pohybu: " << trajectory_pose_index << std::endl;
        std::cout << "-------------------------------" << std::endl;
        return;
    }

    
    if(static_cast<std::size_t>(target_pose_index + 1) < targetPositions.size()) {
        target_pose_index++;
        RCLCPP_ERROR(shared_from_this()->get_logger(), "\n\nDalsia misia.");
        std::cout << "Size: " << targetPositions.size() << std::endl;
        std::cout << "aktualny index celkoveho pohybu: " << target_pose_index << std::endl;
        std::cout << "-------------------------------" << std::endl;;

        trajectory_list = getPoseList(targetPositions[target_pose_index]);
        
    } 
    
    if(static_cast<std::size_t>(target_pose_index + 1) >= targetPositions.size() && static_cast<std::size_t>(trajectory_pose_index) >= trajectory_list.size()) {
	std::cout << "cistim po celkovej misii" << std::endl;
        target_pose_index = -1;
        trajectory_pose_index = 0;
        targetPositions.clear();
        trajectory_list.clear();
        
        std::cout << "Posielam domov" << std::endl;
        geometry_msgs::msg::Pose pose;
        pose.orientation.x = -0.0028119066264480352;
        pose.orientation.y = 0.9999957084655762;
        pose.orientation.z = -0.0007648332393728197;
        pose.orientation.w = -0.00023792324645910412;
        pose.position.x = 0.0 - 0.0438;
        pose.position.y = 0.3;
        pose.position.z = 0.35;  // pre zdvich pridaj 0.195. Rozdiel medzi tool0 hodnotou a mojou je -0.0107
        
        // move(pose);

        isRobotSendingHome = true;
        isRobotMoving = true;
        target_pose = pose;
        moveInThread(target_pose);


//        move_group_interface->setNamedTarget("test_configuration");
//
//        // Create a plan to that target pose
//        moveit::planning_interface::MoveGroupInterface::Plan plan;
//        bool success = static_cast<bool>(move_group_interface->plan(plan));
//
//        // Execute the plan
//        if (success) {
//            move_group_interface->execute(plan);
//        } else {
//            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
//            move_group_interface->setNamedTarget("test_configuration");
//        }

    }
   
    // usleep(2000000); // Sleep for 2000000 microseconds (2 seconds)
}

void RobotControlNode::makeTask(Mission mission) {
    doingTask = true;

    Task task = mission.task;
    int row = mission.row;
    int col = mission.col;
    std::string color = mission.color;

    std::string pieceID = "piece" + std::to_string(row) + std::to_string(col);

    if(task == Task::ATTACH) {
        removeFakePiece(pieceID);

        createPiece(row, col);
        attachPiece();
        closeGripper();
    }
    else {
        detachPiece();
        removePiece();
        createFakePieceWithColor(pieceID, row, col, color);
        chessBoardPub->publish(marker_array_fake_pieces);
        openGripper();
    }

    targetPositions[target_pose_index].task = Task::NONE;
    trajectory_list[trajectory_pose_index].second = Task::NONE;
    doingTask = false;
}

std::pair<bool, double> RobotControlNode::checkPosition(const geometry_msgs::msg::Pose& current_local_pos, const geometry_msgs::msg::Pose& target_position) {
    double threshold = 0.05;

    double current_x = current_local_pos.position.x;
    double current_y = current_local_pos.position.y;
    double current_z = current_local_pos.position.z;
    if (current_z < 0) {
        current_z = 0.0;
    }

    double target_x = target_position.position.x;
    double target_y = target_position.position.y;
    double target_z = target_position.position.z;

    double distance = euclideanDistance(current_x, current_y, current_z, target_x, target_y, target_z);

    if (distance < threshold) {
        RCLCPP_DEBUG(shared_from_this()->get_logger(), "\n\nWe are close enough to the target!\n\n");
        // std::cout << "We are close enough to the target!" << std::endl;
        return std::make_pair(true, distance);
    } else {
        RCLCPP_DEBUG(shared_from_this()->get_logger(), "\n\nStill on the way to the target.\n\n");
        // std::cout << "Still on the way to the target." << std::endl;
        return std::make_pair(false, distance);
    }
}

double RobotControlNode::euclideanDistance(double x1, double y1, double z1, double x2, double y2, double z2) {
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));
}



geometry_msgs::msg::Pose RobotControlNode::getPose() {
    // Get the current state of the robot
//    move_group_interface->setStartStateToCurrentState();
//    move_group_interface->startStateMonitor(2.0);
    currentPosition = move_group_interface->getCurrentPose("tool0").pose;
    
//    std::cout << "Current position: " << currentPosition.position.x << " " << currentPosition.position.y << " " << currentPosition.position.z << std::endl;

    return currentPosition;
}

void RobotControlNode::move(geometry_msgs::msg::Pose targetPose) {
    isRobotMoving = true;
    
    // Set a target Pose
    move_group_interface->setPoseTarget(targetPose);

    // Create a plan to that target pose
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_interface->plan(plan));

    // Execute the plan
    if (success) {
        move_group_interface->asyncExecute(plan);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
//        move(targetPose);
    }
}

void RobotControlNode::moveInThread(geometry_msgs::msg::Pose targetPose) {
    if (moveThread && moveThread->joinable()) {
        moveThread->join(); // Ensure any previous move operation is completed before starting a new one
    }
    moveThread = std::make_unique<std::thread>(&RobotControlNode::move, this, targetPose);
}

RobotControlNode::~RobotControlNode() {
    if (moveThread && moveThread->joinable()) {
        moveThread->join(); // Ensure the thread is completed before destruction
    }
}

void RobotControlNode::stop() {
    std::cout << "STOOOOPPPPP Zavolany" << std::endl;
    isStop = true;
    move_group_interface->stop();
}

void RobotControlNode::handle_resume_movement(const std::shared_ptr<checkers_msgs::srv::ResumeMovement::Request> request,
                                              std::shared_ptr<checkers_msgs::srv::ResumeMovement::Response> response) {
    // Resume robot movement logic here
    isStop = false;
    RCLCPP_INFO(this->get_logger(), "Resuming robot movement.");
    response->success = true;  // Indicate success
    moveInThread(target_pose);
}

void RobotControlNode::ros_out_callback(const rcl_interfaces::msg::Log::SharedPtr msg)
{
    if(isRobotMoving) {
        // Check if the log message contains any of the specific messages
        if (msg->msg.find("Goal reached, success!") != std::string::npos ||
            msg->msg.find("Controller 'joint_trajectory_controller' successfully finished") != std::string::npos ||
            msg->msg.find("Completed trajectory execution with status SUCCEEDED") != std::string::npos ||
            msg->msg.find("Solution was found and executed.") != std::string::npos ||
            msg->msg.find("Plan and Execute request complete!") != std::string::npos)
        {
            RCLCPP_INFO(this->get_logger(), "Filtered Log: '%s'", msg->msg.c_str());
            std::cout << "Robot je v ciely" << std::endl;
            isRobotMoving = false;
            if(isRobotSendingHome) {
                fileIndex++;  // Increment file counter
                currentFileName = "robot_data_" + std::to_string(fileIndex) + ".csv";  // Generate new file name
                isRobotSendingHome = false;
                auto message = checkers_msgs::msg::RobotMove();
                message.robot_move_done = true;
                robotMovePub->publish(message);
            }
        }

        if (msg->msg.find("Unable to solve the planning problem") != std::string::npos ||
            msg->msg.find("Motion plan could not be computed") != std::string::npos ||
            msg->msg.find("Failed to find a motion plan") != std::string::npos ||
            msg->msg.find("Planning failed!") != std::string::npos ||
            msg->msg.find("Timed out") != std::string::npos ||
            msg->msg.find("TIMED_OUT") != std::string::npos
        ) {
            RCLCPP_INFO(this->get_logger(), "Filtered Log: '%s'", msg->msg.c_str());
            moveInThread(target_pose);
        }
    }
}

void RobotControlNode::palm_position_lm_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{

    geometry_msgs::msg::PointStamped palm_point;
    palm_point.point.x = msg->x;
    palm_point.point.y = msg->y;
    palm_point.point.z = msg->z;

    double distance = calculate_palm_LM_distance_to_tool0(palm_point.point);
    std::cout << "Distance: " << distance << std::endl;

    if(distance < 0.275 && isRobotMoving) {
        std::cout << "Bola zachytena ruka" << std::endl;
        std::cout << "Zastavujem robota" << std::endl;
        std::cout << "Distance: " << distance << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
        stop();
    } else if(isStop){
        std::cout << "Ruka je v poriadku" << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
        isStop = false;
        RCLCPP_INFO(this->get_logger(), "Resuming robot movement.");
        moveInThread(target_pose);
    }
}

double RobotControlNode::calculate_palm_LM_distance_to_tool0(const geometry_msgs::msg::Point& palm_point) {

//    std::cout << "Palm position: " << palm_point.x << " " << palm_point.y << " " << palm_point.z << std::endl;
//    std::cout << "Current position: " << currentPosition.position.x << " " << currentPosition.position.y << " " << currentPosition.position.z << std::endl;

    // Calculate the Euclidean distance
    double distance = std::sqrt(
            std::pow(currentPosition.position.x - palm_point.x, 2) +
            std::pow(currentPosition.position.y - palm_point.y, 2) +
            std::pow((currentPosition.position.z - 0.013) - palm_point.z, 2)
    );

//    std::cout << "Euclidean distance from palm to tool0: " << distance << " meters" << std::endl;
//    std::cout << std::endl;
//    std::cout << std::endl;

    if(isRobotMoving && !isStop) {
        // Write to CSV file
        std::ofstream outfile;
        outfile.open("/home/collab/collab_ws/src/robot_control/src/" + currentFileName, std::ios_base::app); // Open in append mode
        if (!outfile.is_open()) {
            std::cerr << "Failed to open file: " << currentFileName << std::endl;
            return distance;
        }
        outfile << currentPosition.position.x << ","
                << currentPosition.position.y << ","
                << currentPosition.position.z << ","
                << palm_point.x << ","
                << palm_point.y << ","
                << palm_point.z << ","
                << (distance < 0.275 && isRobotMoving ? distance : 0) << std::endl;
        outfile.close();
    }

    return distance;
}

void RobotControlNode::hand_detected_callback(const checkers_msgs::msg::HandDetected::SharedPtr msg) {
    if(isRobotMoving) {
        std::cout << "Bola zachytena ruka" << std::endl;
        std::cout << "Zastavujem robota" << std::endl;
        stop();
    }
}

void RobotControlNode::hand_detected_lm_callback(const std_msgs::msg::String::SharedPtr msg) {
//    if(isRobotMoving) {
//        std::cout << "Bola zachytena ruka" << std::endl;
//        std::cout << "Zastavujem robota" << std::endl;
//        stop();
//    }
//    std::cout << "Bola zachytena ruka" << std::endl;
//    std::cout << "Zastavujem robota" << std::endl;
}

void RobotControlNode::checkers_move_callback(const checkers_msgs::msg::Move::SharedPtr msg) {
    std::cout << "-------------------------------" << std::endl;
    std::cout << "checkers_move_callback zavolany" << std::endl;
    std::cout << "-------------------------------" << std::endl;
    
    targetPositions.clear();
    target_pose_index = -1;
    

    // First Mission: Attach at the start position
    auto [startRow, startCol] = rotate90DegreesCounterClockwise(msg->piece_for_moving.row, msg->piece_for_moving.col);

    targetPositions.push_back(Mission(startRow, startCol, redColorString, Task::ATTACH));

    // Second Mission: Detach at the target position
    auto [targetRow, targetCol] = rotate90DegreesCounterClockwise(msg->target_row, msg->target_col);

    targetPositions.push_back(Mission(targetRow, targetCol, redColorString, Task::DETACH));

    // Additional Missions for removed pieces
    for (const auto& piece : msg->removed_pieces) {
        auto [row, col] = rotate90DegreesCounterClockwise(piece.row, piece.col);
        
        // Place the removed piece next to the board
        int xPos = -9 - (removedPiecesCount / 6); // There are 6 positions in a row
        int yPos = removedPiecesCount % 6; // Switches between -1 and -2 based on the count
        removedPiecesCount++;

        targetPositions.push_back(Mission(row, col, whiteColorString, Task::ATTACH));

        targetPositions.push_back(Mission(xPos, yPos, whiteColorString, Task::DETACH));
    }
}

std::vector<std::pair<geometry_msgs::msg::Pose, Task>> RobotControlNode::getPoseList(Mission mission) {
    trajectory_pose_index = 0;
    std::vector<std::pair<geometry_msgs::msg::Pose, Task>> poses;

    
    // float posX = (mission.row * square_size) + boardOffsetX + (square_size/2);
    // float posY = (mission.col * square_size) + boardOffsetY + (square_size/2);

    std::cout << "-------------------------------" << std::endl;
    std::cout << "mission.row : " << mission.row  << std::endl;
    std::cout << "mission.col : " << mission.col  << std::endl;
    std::cout << "square_size: " << square_size << std::endl;
    std::cout << "boardOffsetX: " << boardOffsetX << std::endl;
    std::cout << "boardOffsetY: " << boardOffsetY << std::endl;
    std::cout << "-------------------------------" << std::endl;
    float posX = (mission.row * square_size) + boardOffsetX;
    float posY = (mission.col * square_size) + boardOffsetY;

    std::cout << "-------------------------------" << std::endl;
    std::cout << "posX: " << posX << std::endl;
    std::cout << "posY: " << posY << std::endl;
    std::cout << "-------------------------------" << std::endl;

//    msg.orientation.x = -0.043154485523700714;
//    msg.orientation.y = 0.9961686134338379;
//    msg.orientation.z = 0.07547704875469208;
//    msg.orientation.w = -0.009561617858707905;
//    msg.position.x =  1.1692674160003662;
//    msg.position.y = 0.24401788413524628;
//    msg.position.z = 0.5697827935218811;


//    geometry_msgs::msg::Pose pose1;
//    pose1.orientation.x = -0.043154485523700714;
//    pose1.orientation.y = 0.9961686134338379;
//    pose1.orientation.z = 0.07547704875469208;
//    pose1.orientation.w = -0.009561617858707905;
//    pose1.position.x = 0.7214473485946655;
//    pose1.position.y = 0.26021891832351685;
//    pose1.position.z = 0.25;
//    poses.push_back(std::make_pair(pose1, Task::NONE));

    geometry_msgs::msg::Pose pose1;
    pose1.orientation.x = -0.0028119066264480352;
    pose1.orientation.y = 0.9999957084655762;
    pose1.orientation.z = -0.0007648332393728197;
    pose1.orientation.w = -0.00023792324645910412;
    pose1.position.x = posX;
    pose1.position.y = posY;
    pose1.position.z = 0.19;
    poses.push_back(std::make_pair(pose1, Task::NONE));

    geometry_msgs::msg::Pose pose2;
    pose2.orientation.x = -0.0028119066264480352;
    pose2.orientation.y = 0.9999957084655762;
    pose2.orientation.z = -0.0007648332393728197;
    pose2.orientation.w = -0.00023792324645910412;
    pose2.position.x = posX;
    pose2.position.y = posY;
    pose2.position.z = 0.159 - 0.0107;
    poses.push_back(std::make_pair(pose2, Task::NONE));

    geometry_msgs::msg::Pose pose3;
    pose3.orientation.x = -0.0028119066264480352;
    pose3.orientation.y = 0.9999957084655762;
    pose3.orientation.z = -0.0007648332393728197;
    pose3.orientation.w = -0.00023792324645910412;
    pose3.position.x = posX;
    pose3.position.y = posY;
    pose3.position.z = 0.19;
    poses.push_back(std::make_pair(pose3, mission.task));


//    geometry_msgs::msg::Pose pose1;
//    pose1.orientation.x = 0.0;
//    pose1.orientation.y = 1.0;
//    pose1.orientation.z = 0.0;
//    pose1.orientation.w = 0.0;
//    pose1.position.x = posX;
//    pose1.position.y = posY;
//    pose1.position.z = 0.04123930260539055 + 0.30;
//    poses.push_back(std::make_pair(pose1, Task::NONE));
//
//
//    geometry_msgs::msg::Pose pose2;
//    pose2.orientation.x = 0.0;
//    pose2.orientation.y = 1.0;
//    pose2.orientation.z = 0.0;
//    pose2.orientation.w = 0.0;
//    pose2.position.x = posX;
//    pose2.position.y = posY;
//    pose2.position.z = 0.04123930260539055;
//    poses.push_back(std::make_pair(pose2, mission.task));
//
//    geometry_msgs::msg::Pose pose3;
//    pose3.orientation.x = 0.0;
//    pose3.orientation.y = 1.0;
//    pose3.orientation.z = 0.0;
//    pose3.orientation.w = 0.0;
//    pose3.position.x = posX;
//    pose3.position.y = posY;
//    pose3.position.z = 0.042 + 0.30;
//    poses.push_back(std::make_pair(pose3, Task::NONE));

    return poses;
}


void RobotControlNode::checkers_board_callback(const checkers_msgs::msg::Board::SharedPtr msg)
{
    removeAllFakePieces();

    for (const auto& piece : msg->pieces) {

        auto [row, col] = rotate90DegreesCounterClockwise(piece.row, piece.col);
        
        std::string color = piece.color;
        // bool isKing = piece.king;

        std::string pieceID = "piece" + std::to_string(row) + std::to_string(col);
        
        createFakePieceWithColor(pieceID, row, col, color);
    }

    chessBoardPub->publish(marker_array_fake_pieces);

    // removeFakePiece("piece01");
    
    if(startProgram) {
        // move(target_pose);
        startProgram = false;
    }
}

std::tuple<float, float, float> RobotControlNode::getColorFromName(const std::string& colorName) {
    if (colorName == "white") {
        return std::make_tuple(1.0f, 1.0f, 1.0f); // RGB for white
    } else if (colorName == "red") {
        return std::make_tuple(1.0f, 0.0f, 0.0f); // RGB for red
    }

    // Default color (black) if no match is found
    return std::make_tuple(0.0f, 0.0f, 0.0f);
}



void RobotControlNode::createFakePieceWithColor(const std::string& object_id, int row, int col, const std::string& colorName) {
    int objectIDLong = convertStringToInt(object_id);

    piecesInRviz[objectIDLong] = true;

    std::tuple<float, float, float> color = getColorFromName(colorName);
    
    visualization_msgs::msg::Marker fakePiece;
    // fakePiece.header.frame_id = move_group_interface->getPlanningFrame();
    fakePiece.header.frame_id = "base_link";
    fakePiece.id = objectIDLong;

    fakePiece.type = visualization_msgs::msg::Marker::CYLINDER;
    fakePiece.action = visualization_msgs::msg::Marker::ADD;

    fakePiece.pose.position.x = (row * square_size) + boardOffsetX;
    fakePiece.pose.position.y = (col * square_size) + boardOffsetY;
    fakePiece.pose.position.z = 0.0075;
    fakePiece.pose.orientation.w = 1.0;

    fakePiece.scale.x = 0.028;
    fakePiece.scale.y = 0.028;
    fakePiece.scale.z = 0.005;

    fakePiece.color.r = std::get<0>(color);
    fakePiece.color.g = std::get<1>(color);
    fakePiece.color.b = std::get<2>(color);
    fakePiece.color.a = 1.0;  // Alpha

    marker_array_fake_pieces.markers.push_back(fakePiece);

    
}

int RobotControlNode::convertStringToInt(const std::string& stringID){
    long long concatenatedNumber = 0;

    for (char c : stringID) {
        concatenatedNumber = concatenatedNumber * 1000 + static_cast<int>(c);
    }

    return concatenatedNumber;
}

void RobotControlNode::removeAllFakePieces() {
    for (auto& marker : marker_array_fake_pieces.markers) {
        int markerID = marker.id;
        if (piecesInRviz.find(markerID) != piecesInRviz.end()) {
            bool isOnTheBoard = piecesInRviz[markerID];
            if(isOnTheBoard) {
                marker.action = visualization_msgs::msg::Marker::DELETE;
            }
        }
    }

    chessBoardPub->publish(marker_array_fake_pieces);
    piecesInRviz.clear();
    marker_array_fake_pieces.markers.clear();
}

void RobotControlNode::removeFakePiece(const std::string& object_id) {
    int objectIDLong = convertStringToInt(object_id);
    // visualization_msgs::msg::MarkerArray marker_array;

    // visualization_msgs::msg::Marker removePiece;
    // removePiece.header.frame_id = move_group_interface->getPlanningFrame();
    // removePiece.id = objectIDLong;
    // removePiece.action = visualization_msgs::msg::Marker::DELETE;

    // marker_array.markers.push_back(removePiece);

    // chessBoardPub->publish(marker_array);

    // visualization_msgs::msg::Marker marker_for_delete;
    // for (auto& marker : marker_array_fake_pieces.markers) {
    //     int markerID = marker.id;
    //     if(objectIDLong == markerID){
    //         marker.action = visualization_msgs::msg::Marker::DELETE;
    //         marker_for_delete = marker;
    //         break;
    //     }
    // }


    visualization_msgs::msg::MarkerArray updated_marker_array;

    for (auto& marker : marker_array_fake_pieces.markers) {
        if (marker.id != objectIDLong) {  // Keep all markers except the one to remove
            updated_marker_array.markers.push_back(marker);
        }
        else {
            marker.action = visualization_msgs::msg::Marker::DELETE;
            if (piecesInRviz.find(objectIDLong) != piecesInRviz.end()) {
                piecesInRviz.erase(objectIDLong);
            }
        }
    }

    chessBoardPub->publish(marker_array_fake_pieces);

    marker_array_fake_pieces = updated_marker_array;
}

void RobotControlNode::createPiece(int row, int col) {
    collision_object = moveit_msgs::msg::CollisionObject();
//    collision_object.header.frame_id = move_group_interface->getPlanningFrame();
    collision_object.header.frame_id = "base_link";
    collision_object.id = "collisionObjectID";
    shape_msgs::msg::SolidPrimitive primitive;


    float posX = (row * square_size) + boardOffsetX;
    float posY = (col * square_size) + boardOffsetY;

    // Define the size of the box in meters
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.005; // Height of the cylinder
    primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.028/2; // Radius of the cylinder

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose cylinder_pose;
    cylinder_pose.orientation.w = 1.0;
    cylinder_pose.position.x = posX;
    cylinder_pose.position.y = posY;
    cylinder_pose.position.z = 0.0075;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
    collision_object.operation = collision_object.ADD;
    

    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);
}

void RobotControlNode::removePiece() {
    collision_object.operation = collision_object.REMOVE;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);
}

void RobotControlNode::attachPiece() {
    move_group_interface->attachObject(collision_object.id, "tool0"); 
}

void RobotControlNode::detachPiece() {
    move_group_interface->detachObject(collision_object.id); 
}



void RobotControlNode::publishCheckerboard()
{
    visualization_msgs::msg::MarkerArray marker_array;
        int rows = 8;
        int cols = 8;
        

        // Create the checkerboard squares and pieces
        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {

                auto [rotatedRow, rotatedCol] = rotate90DegreesCounterClockwise(row, col);

                // Add square
                visualization_msgs::msg::Marker square_marker;
//                square_marker.header.frame_id = move_group_interface->getPlanningFrame();
                square_marker.header.frame_id = "base_link";
                square_marker.type = visualization_msgs::msg::Marker::CUBE;
                square_marker.action = visualization_msgs::msg::Marker::ADD;

                square_marker.pose.position.x = (rotatedRow * square_size) + boardOffsetX;
                square_marker.pose.position.y = (rotatedCol * square_size) + boardOffsetY;
                square_marker.pose.position.z = 0.0;
                square_marker.pose.orientation.w = 1.0;

                square_marker.scale.x = square_size;
                square_marker.scale.y = square_size;
                square_marker.scale.z = 0.01;

                if ((row + col) == 0)
                {
                    square_marker.color.r = 0.0;
                    square_marker.color.g = 1.0;
                    square_marker.color.b = 0.0;
                }
                else if ((rotatedRow + rotatedCol) % 2 == 0)
                {
                    square_marker.color.r = 1.0;
                    square_marker.color.g = 1.0;
                    square_marker.color.b = 1.0;
                }
                else
                {
                    square_marker.color.r = 0.0;
                    square_marker.color.g = 0.0;
                    square_marker.color.b = 0.0;
                }

                square_marker.color.a = 1.0;  // Alpha
                square_marker.id = rotatedRow * cols + rotatedCol;

                marker_array.markers.push_back(square_marker);
            }
        }

        chessBoardPub->publish(marker_array);
}


std::pair<int, int> RobotControlNode::rotate90DegreesCounterClockwise(int x, int y) {
    return {-y, x};
}
