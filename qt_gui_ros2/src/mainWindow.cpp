#include "qt_gui_ros2/mainWindow.hpp"

#include <QGridLayout>
#include <geometry_msgs/msg/point.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_rendering/render_window.hpp>
#include <QVector3D>
#include <QDebug>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>

MainWindow::MainWindow(QApplication *app, rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, QWidget *parent)
    : QMainWindow(parent), app_(app), rviz_ros_node_(rviz_ros_node), mapReceived_(false), globalCostmapReceived_(false), localCostmapReceived_(false) {
    centralWidget_ = new QWidget();
    mainLayout_ = new QHBoxLayout;
    sidebarWidget_ = new QWidget();
    sidebarLayout_ = new QVBoxLayout;

    initializeRViz();

    // Sidebar styling
    sidebarWidget_->setFixedWidth(250);
    sidebarWidget_->setStyleSheet(R"(
        QWidget { 
            background-color: #2E2E2E; 
            color: #FFFFFF; 
            font-family: 'Arial'; 
            font-size: 14px; 
        }
    )");

    // Frame input section
    QLabel *frameLabel = new QLabel("Frame:");
    frameLabel->setStyleSheet("font-weight: bold; color: #E0E0E0; padding: 5px;");
    frameLineEdit_ = new QLineEdit("map");
    frameLineEdit_->setStyleSheet(R"(
        QLineEdit {
            background-color: #3C3C3C;
            color: #FFFFFF;
            border: 1px solid #555555;
            border-radius: 5px;
            padding: 5px;
        }
        QLineEdit:focus {
            border: 1px solid #1E90FF;
        }
    )");
    frameLineEdit_->setToolTip("Enter the reference frame (e.g., 'map')");
    QPushButton *updateFrameButton = new QPushButton("Update");
    updateFrameButton->setStyleSheet(R"(
        QPushButton {
            background-color: #1E90FF;
            color: #FFFFFF;
            border: none;
            border-radius: 5px;
            padding: 8px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #4682B4;
        }
        QPushButton:pressed {
            background-color: #4169E1;
        }
    )");
    updateFrameButton->setToolTip("Apply the reference frame");
    connect(updateFrameButton, &QPushButton::clicked, this, &MainWindow::updateFrame);

    // Status indicators
    mapReceivedIndicator_ = new QLabel("Map Status: Waiting", this);
    mapReceivedIndicator_->setStyleSheet("color: #FF5555; padding: 5px;");
    globalCostmapIndicator_ = new QLabel("Global Costmap: Waiting", this);
    globalCostmapIndicator_->setStyleSheet("color: #FF5555; padding: 5px;");
    localCostmapIndicator_ = new QLabel("Local Costmap: Waiting", this);
    localCostmapIndicator_->setStyleSheet("color: #FF5555; padding: 5px;");

    // Joystick controls
    setupJoystickControls();

    // Sidebar layout
    sidebarLayout_->addWidget(frameLabel);
    sidebarLayout_->addWidget(frameLineEdit_);
    sidebarLayout_->addWidget(updateFrameButton);
    sidebarLayout_->addWidget(mapReceivedIndicator_);
    sidebarLayout_->addWidget(globalCostmapIndicator_);
    sidebarLayout_->addWidget(localCostmapIndicator_);
    sidebarLayout_->addStretch(); // Push controls to top
    sidebarWidget_->setLayout(sidebarLayout_);

    // Main layout
    renderPanel_->setStyleSheet("border: 1px solid #444444;");
    mainLayout_->addWidget(sidebarWidget_);
    mainLayout_->addWidget(renderPanel_, 1); // RViz takes remaining space
    centralWidget_->setLayout(mainLayout_);
    setCentralWidget(centralWidget_);
    setStyleSheet("QMainWindow { background-color: #1C1C1C; }");

    // Initialize /cmd_vel publisher
    cmdVelPublisher_ = rviz_ros_node_.lock()->get_raw_node()->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Set up RViz displays
    QString frame_id = frameLineEdit_->text();
    manager_->getRootDisplayGroup()->setFixedFrame(frame_id);
    manager_->setFixedFrame(frame_id);

    setupGridDisplay();
    setupTFDisplay();
    setupMapDisplay();
    setupRobotModelDisplay();
    setupLaserScanDisplay();
    setupGlobalCostmapDisplay();
    setupLocalCostmapDisplay();
    setupSubscribers();
}

MainWindow::~MainWindow() {
    rclcpp::shutdown();
}

QWidget *MainWindow::getParentWindow() {
    return this;
}

rviz_common::PanelDockWidget *MainWindow::addPane(const QString &name, QWidget *pane, Qt::DockWidgetArea area, bool floating) {
    return nullptr;
}

void MainWindow::setStatus(const QString &message) {
}

void MainWindow::initializeRViz() {
    app_->processEvents();
    renderPanel_ = new rviz_common::RenderPanel(centralWidget_);
    app_->processEvents();
    renderPanel_->getRenderWindow()->initialize();

    auto clock = rviz_ros_node_.lock()->get_raw_node()->get_clock();
    manager_ = new rviz_common::VisualizationManager(renderPanel_, rviz_ros_node_, this, clock);
    renderPanel_->initialize(manager_);

    renderPanel_->setMouseTracking(true);
    renderPanel_->setFocusPolicy(Qt::StrongFocus);

    app_->processEvents();
    manager_->initialize();
    manager_->startUpdate();

    manager_->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/Orbit");
    auto orbit_view_controller = manager_->getViewManager()->getCurrent();
    if (!orbit_view_controller) {
        qDebug() << "Orbit view controller could not be set.";
        return;
    }

    qDebug() << "Orbit view controller initialized successfully.";

    orbit_view_controller->subProp("Distance")->setValue(10.0);
    orbit_view_controller->subProp("Focal Point")->setValue(QVariant::fromValue(QVector3D(0.0, 0.0, 0.0)));
    orbit_view_controller->subProp("Pitch")->setValue(1.5708);
    orbit_view_controller->subProp("Yaw")->setValue(3.14);

    auto tool_manager = manager_->getToolManager();
    tool_manager->setCurrentTool(tool_manager->addTool("rviz_default_plugins/Interact"));
}

void MainWindow::setupGridDisplay() {
    QString frame_id = frameLineEdit_->text();
    grid_ = manager_->createDisplay("rviz_default_plugins/Grid", "Grid", true);
    if (grid_) {
        grid_->subProp("Line Style")->setValue("Lines");
        grid_->subProp("Color")->setValue(QColor(Qt::white));
        grid_->subProp("Reference Frame")->setValue(frame_id);
        qDebug() << "Grid display configured for fixed frame:" << frame_id;
    } else {
        qDebug() << "Failed to create Grid display.";
    }
}

void MainWindow::setupTFDisplay() {
    tf_display_ = manager_->createDisplay("rviz_default_plugins/TF", "TF Display", true);
    if (tf_display_) {
        tf_display_->subProp("Show Axes")->setValue(true);
        qDebug() << "TF display configured with axes and names shown.";
    } else {
        qDebug() << "Failed to create TF display.";
    }
}

void MainWindow::setupMapDisplay() {
    QString frame_id = frameLineEdit_->text();
    map_display_ = manager_->createDisplay("rviz_default_plugins/Map", "Map Display", true);
    if (map_display_) {
        map_display_->subProp("Topic")->setValue("/map");
        map_display_->subProp("Alpha")->setValue(1.0);
        map_display_->subProp("Draw Behind")->setValue(false);
        map_display_->subProp("Color Scheme")->setValue("map");
        map_display_->subProp("Topic")->subProp("Durability Policy")->setValue("Transient Local");
        qDebug() << "Map display configured for /map topic with fixed frame:" << frame_id;
    } else {
        qDebug() << "Failed to create Map display.";
    }
}

void MainWindow::setupRobotModelDisplay() {
    robot_model_display_ = manager_->createDisplay("rviz_default_plugins/RobotModel", "RobotModel Display", true);
    if (robot_model_display_) {
        robot_model_display_->subProp("Description Topic")->setValue("/robot_description");
        robot_model_display_->subProp("TF Prefix")->setValue(""); // Set to "tb3_0" if using namespace
        robot_model_display_->subProp("Alpha")->setValue(1.0);
        robot_model_display_->setEnabled(true);
        qDebug() << "RobotModel display configured for /robot_description topic.";
    } else {
        qDebug() << "Failed to create RobotModel display.";
    }
}

void MainWindow::setupGlobalCostmapDisplay() {
    QString frame_id = frameLineEdit_->text();
    global_costmap_display_ = manager_->createDisplay("rviz_default_plugins/Map", "Global Costmap Display", true);
    if (global_costmap_display_) {
        global_costmap_display_->subProp("Topic")->setValue("/global_costmap/costmap");
        global_costmap_display_->subProp("Alpha")->setValue(0.7);
        global_costmap_display_->subProp("Color Scheme")->setValue("costmap");
        global_costmap_display_->subProp("Reference Frame")->setValue(frame_id);
        global_costmap_display_->subProp("Topic")->subProp("Durability Policy")->setValue("Transient Local");
        global_costmap_display_->setEnabled(true);
        qDebug() << "Global Costmap display configured for /global_costmap/costmap topic with fixed frame:" << frame_id;
    } else {
        qDebug() << "Failed to create Global Costmap display.";
    }
}

void MainWindow::setupLocalCostmapDisplay() {
    QString frame_id = frameLineEdit_->text();
    local_costmap_display_ = manager_->createDisplay("rviz_default_plugins/Map", "Local Costmap Display", true);
    if (local_costmap_display_) {
        local_costmap_display_->subProp("Topic")->setValue("/local_costmap/costmap");
        local_costmap_display_->subProp("Alpha")->setValue(0.5);
        local_costmap_display_->subProp("Color Scheme")->setValue("costmap");
        local_costmap_display_->subProp("Reference Frame")->setValue(frame_id);
        local_costmap_display_->subProp("Topic")->subProp("Durability Policy")->setValue("Transient Local");
        local_costmap_display_->setEnabled(true);
        qDebug() << "Local Costmap display configured for /local_costmap/costmap topic with fixed frame:" << frame_id;
    } else {
        qDebug() << "Failed to create Local Costmap display.";
    }
}

void MainWindow::setupLaserScanDisplay() {
    auto laser_scan_display = manager_->createDisplay("rviz_default_plugins/LaserScan", "LaserScan Display", true);
    if (laser_scan_display) {
        laser_scan_display->subProp("Topic")->setValue("/scan");
        laser_scan_display->subProp("Size (m)")->setValue(0.1);
        laser_scan_display->subProp("Color")->setValue(QColor(Qt::green));
        qDebug() << "LaserScan display configured successfully for /scan.";
    } else {
        qDebug() << "Failed to configure LaserScan display.";
    }
}

void MainWindow::setupJoystickControls() {
    QGridLayout *joystickLayout = new QGridLayout;

    forwardButton_ = new QPushButton("↑ Forward");
    backwardButton_ = new QPushButton("↓ Backward");
    leftButton_ = new QPushButton("← Left");
    rightButton_ = new QPushButton("Right →");
    stopButton_ = new QPushButton("Stop");

    QString buttonStyle = R"(
        QPushButton {
            background-color: #4CAF50;
            color: #FFFFFF;
            border: none;
            border-radius: 8px;
            padding: 10px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #45A049;
        }
        QPushButton:pressed {
            background-color: #3D8B40;
        }
    )";

    forwardButton_->setStyleSheet(buttonStyle);
    backwardButton_->setStyleSheet(buttonStyle);
    leftButton_->setStyleSheet(buttonStyle);
    rightButton_->setStyleSheet(buttonStyle);
    stopButton_->setStyleSheet(R"(
        QPushButton {
            background-color: #FF5555;
            color: #FFFFFF;
            border: none;
            border-radius: 8px;
            padding: 10px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #FF3333;
        }
        QPushButton:pressed {
            background-color: #CC0000;
        }
    )");

    forwardButton_->setToolTip("Move the robot forward");
    backwardButton_->setToolTip("Move the robot backward");
    leftButton_->setToolTip("Turn the robot left");
    rightButton_->setToolTip("Turn the robot right");
    stopButton_->setToolTip("Stop the robot");

    joystickLayout->addWidget(forwardButton_, 0, 1);
    joystickLayout->addWidget(backwardButton_, 2, 1);
    joystickLayout->addWidget(leftButton_, 1, 0);
    joystickLayout->addWidget(rightButton_, 1, 2);
    joystickLayout->addWidget(stopButton_, 1, 1);

    joystickLayout->setSpacing(10);
    sidebarLayout_->addLayout(joystickLayout);

    connect(forwardButton_, &QPushButton::pressed, this, [this]() {
        currentTwist_.linear.x = 0.05; currentTwist_.angular.z = 0.0;
        sendJoystickCommand();
    });
    connect(backwardButton_, &QPushButton::pressed, this, [this]() {
        currentTwist_.linear.x = -0.05; currentTwist_.angular.z = 0.0;
        sendJoystickCommand();
    });
    connect(leftButton_, &QPushButton::pressed, this, [this]() {
        currentTwist_.linear.x = 0.0; currentTwist_.angular.z = 0.2;
        sendJoystickCommand();
    });
    connect(rightButton_, &QPushButton::pressed, this, [this]() {
        currentTwist_.linear.x = 0.0; currentTwist_.angular.z = -0.2;
        sendJoystickCommand();
    });
    connect(stopButton_, &QPushButton::pressed, this, [this]() {
        currentTwist_.linear.x = 0.0; currentTwist_.angular.z = 0.0;
        sendJoystickCommand();
    });
}

void MainWindow::sendJoystickCommand() {
    cmdVelPublisher_->publish(currentTwist_);
}

void MainWindow::updateFrame() {
    QString frame_id = frameLineEdit_->text();
    if (grid_) {
        grid_->subProp("Reference Frame")->setValue(frame_id);
    }
    if (map_display_) {
        map_display_->subProp("Reference Frame")->setValue(frame_id);
    }
    if (global_costmap_display_) {
        global_costmap_display_->subProp("Reference Frame")->setValue(frame_id);
    }
    if (local_costmap_display_) {
        local_costmap_display_->subProp("Reference Frame")->setValue(frame_id);
    }
    if (manager_ && manager_->getFrameManager()) {
        manager_->setFixedFrame(frame_id);
        manager_->getRootDisplayGroup()->setFixedFrame(frame_id);
        qDebug() << "FrameManager fixed frame updated to:" << frame_id;
    }
}

void MainWindow::closeEvent(QCloseEvent *event) {
    rclcpp::shutdown();
    event->accept();
    qDebug() << "Application closed, ROS shutdown complete.";
}

void MainWindow::setupSubscribers() {
    auto node = rviz_ros_node_.lock()->get_raw_node();
    mapSubscriber_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            Q_UNUSED(msg);
            mapReceived_ = true;
            qDebug() << "Map Received";
            updateMapReceivedIndicator(true);
        }
    );
    globalCostmapSubscriber_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap", 10,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            Q_UNUSED(msg);
            globalCostmapReceived_ = true;
            qDebug() << "Global Costmap Received";
            updateGlobalCostmapIndicator(true);
        }
    );
    localCostmapSubscriber_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/local_costmap/costmap", 10,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            Q_UNUSED(msg);
            localCostmapReceived_ = true;
            qDebug() << "Local Costmap Received";
            updateLocalCostmapIndicator(true);
        }
    );
}

void MainWindow::updateMapReceivedIndicator(bool received) {
    if (received) {
        mapReceivedIndicator_->setText("Map Status: Received");
        mapReceivedIndicator_->setStyleSheet("color: #55FF55; padding: 5px;");
    } else {
        mapReceivedIndicator_->setText("Map Status: Waiting");
        mapReceivedIndicator_->setStyleSheet("color: #FF5555; padding: 5px;");
    }
}

void MainWindow::updateGlobalCostmapIndicator(bool received) {
    if (received) {
        globalCostmapIndicator_->setText("Global Costmap: Received");
        globalCostmapIndicator_->setStyleSheet("color: #55FF55; padding: 5px;");
    } else {
        globalCostmapIndicator_->setText("Global Costmap: Waiting");
        globalCostmapIndicator_->setStyleSheet("color: #FF5555; padding: 5px;");
    }
}

void MainWindow::updateLocalCostmapIndicator(bool received) {
    if (received) {
        localCostmapIndicator_->setText("Local Costmap: Received");
        localCostmapIndicator_->setStyleSheet("color: #55FF55; padding: 5px;");
    } else {
        localCostmapIndicator_->setText("Local Costmap: Waiting");
        localCostmapIndicator_->setStyleSheet("color: #FF5555; padding: 5px;");
    }
}