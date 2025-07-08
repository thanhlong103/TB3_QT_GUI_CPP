#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QCloseEvent>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace rviz_common {
class Display;
}

class MainWindow : public QMainWindow, public rviz_common::WindowManagerInterface {
    Q_OBJECT

public:
    MainWindow(QApplication *app, rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, QWidget *parent = nullptr);
    ~MainWindow();

    QWidget *getParentWindow() override;
    rviz_common::PanelDockWidget *addPane(const QString &name, QWidget *pane, Qt::DockWidgetArea area, bool floating) override;
    void setStatus(const QString &message) override;

protected:
    void closeEvent(QCloseEvent *event) override;

private slots:
    void sendJoystickCommand();              // Sends cmd_vel based on button input
    void updateFrame();                      // Slot to update the reference frame
    void updateMapReceivedIndicator(bool received);  // Updates map received indicator
    void updateGlobalCostmapIndicator(bool received); // Updates global costmap indicator
    void updateLocalCostmapIndicator(bool received);  // Updates local costmap indicator

private:
    void initializeRViz();                   // Initializes RViz components
    void setupJoystickControls();            // Initializes joystick buttons
    void setupGridDisplay();
    void setupTFDisplay();
    void setupMapDisplay();
    void setupRobotModelDisplay();
    void setupLaserScanDisplay();
    void setupGlobalCostmapDisplay();
    void setupLocalCostmapDisplay();
    void setupSubscribers();                 // Sets up map and costmap subscribers

    QApplication *app_;
    QWidget *centralWidget_;
    QHBoxLayout *mainLayout_;                // Main layout (sidebar + RViz)
    QWidget *sidebarWidget_;                 // Sidebar for controls
    QVBoxLayout *sidebarLayout_;             // Layout for sidebar controls
    QLineEdit *frameLineEdit_;               // Text box for reference frame
    QLabel *mapReceivedIndicator_;           // Indicator for map reception
    QLabel *globalCostmapIndicator_;         // Indicator for global costmap
    QLabel *localCostmapIndicator_;          // Indicator for local costmap
    
    rviz_common::RenderPanel *renderPanel_;
    rviz_common::Display *grid_;
    rviz_common::Display *tf_display_;
    rviz_common::Display *map_display_;
    rviz_common::Display *robot_model_display_;
    rviz_common::Display *global_costmap_display_;
    rviz_common::Display *local_costmap_display_;
    rviz_common::VisualizationManager *manager_;

    // ROS node and publisher for /cmd_vel
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher_;
    geometry_msgs::msg::Twist currentTwist_;

    // Joystick buttons
    QPushButton *forwardButton_;
    QPushButton *backwardButton_;
    QPushButton *leftButton_;
    QPushButton *rightButton_;
    QPushButton *stopButton_;
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr globalCostmapSubscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr localCostmapSubscriber_;
    bool mapReceived_;
    bool globalCostmapReceived_;
    bool localCostmapReceived_;
};

#endif // MAINWINDOW_HPP