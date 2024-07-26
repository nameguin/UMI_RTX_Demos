/**
 * @file main_gui.cpp
 * @brief Implementation of the MainGUI class and the main function for the UMI-RTX Interface.
 * 
 * This file contains the implementation of the MainGUI class, which represents the main graphical user interface for the UMI-RTX controller. 
 * It includes widget initialization, layout management, slider and button functionalities, and integration with RViz for visualization. 
 * The file also contains the main function which initializes and runs the application.
 * 
 * @note This file requires Qt and RViz libraries and assumes ROS2 integration.
 */

#ifndef __GUI__
#define __GUI__

#include <QApplication>
#include <QSlider>
#include <QLabel>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QObject>
#include <QMainWindow>
#include <QWidget>
#include <QPushButton>
#include <QPalette>
#include <QDockWidget>
#include <QProcess>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QResizeEvent>
#include <QTimer>
#include <QImage>


#include "umi_rtx_controller/node_commands.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"

#include "rviz_common/display.hpp"
#include "rviz_common/window_manager_interface.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/visualization_manager.hpp"
#include <rviz_common/config.hpp>
#include <rviz_common/yaml_config_reader.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include "rviz_rendering/render_window.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

namespace rviz_common
{
class Display;
class RenderPanel;
class VisualizationManager;
}

class MainGUI : public QMainWindow, public rviz_common::WindowManagerInterface {
public:
    /**
     * @brief Construct a new MainGUI object
     * 
     * Initializes the main GUI components, including the widgets, layouts, sliders, buttons, and integrates RViz for 3D visualization.
     *
     * @param app QApplication object that will be used for the GUI
     * @param ros2_node The command node that works in pair with this interface
     * @param rviz_ros_node The RViz ROS node that is necessary to run RViz2 in our interface
     * @param parent 
     */
    MainGUI(QApplication * app,
            const std::shared_ptr<Objective_node>&  ros2_node, 
            rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, 
            QWidget* parent = nullptr);

    /**
     * @brief Destroy the Main GUI object
     */
    ~MainGUI() override;

    double x=0., y=0.6, z=0.6, yaw=0., pitch=0., roll=0., grip=0.02;
    double raw_yaw=0.;

    bool is_started_game = false;
    bool manual_on = true;
    int frame_stream = 0;

    /**
     * @brief Get the Parent Window object, override of the QMainWindow property, necessary for compilation but useless here
     * 
     * @return QWidget* 
     */
    QWidget * getParentWindow() override;
    /**
     * @brief Add a DockWidget to our window, override of the QMainWindow property, necessary for compilation but useless here
     * 
     * @param name Name of the new DockWidget
     * @param pane Type of the desired DOckWidget
     * @param area Size of the Widget
     * @param floating 
     * @return rviz_common::PanelDockWidget* 
     */
    rviz_common::PanelDockWidget * addPane(const QString & name, QWidget * pane, Qt::DockWidgetArea area, bool floating) override;
    /**
     * @brief Set the Status object, override of the QMainWindow property, necessary for compilation but useless here
     * 
     * @param message 
     */
    void setStatus(const QString & message) override;

private:
    const shared_ptr<Objective_node> ros2_node;
    

    QApplication* app_;
    QWidget* main_widget;
    QPushButton* switchButton;
    QImage* image;
    QLabel *Title;
    QLabel* videoLabel;
    QTimer* timer;
    QPushButton* gameButton;
    QDoubleSpinBox *spinBox_x,*spinBox_y,*spinBox_z,*spinBox_yaw,*spinBox_pitch,*spinBox_roll, *spinBox_grip; 
    QHBoxLayout *main_layout;

    QVBoxLayout* game_layout;
    QGridLayout* board_layout;
    QVBoxLayout* info_layout;
    QVBoxLayout* history_layout;
    QVBoxLayout* umi_layout;

    QLabel *turn_label;
    QLabel *player_label;
    QLabel *info_labels[9];

    QLabel *board_labels[3][3];
    int board[3][3];
    std::vector<std::string> msgs;

    QPixmap case0;
    QPixmap case1;
    QPixmap case2;


    rviz_common::RenderPanel * render_panel_;
    rviz_common::Display *TF_, *Model_;
    rviz_common::VisualizationManager * manager_;
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;

    cv::Mat* frame;
    /**
     * @brief Initialise the RViz2 object, in order to integrate in our interface
     */
    void initializeRViz();

private slots:
    /**
     * @brief Event necessary to compile, close the window and shutdown the ros node.
     * @param event 
     */
    void closeEvent(QCloseEvent *event);

    /**
    * @brief Update the frame and interface elements based on new camera data.
    */
    void updateFrameAndInterface();

    /**
    * @brief Add a slider with its associated label and spin box to the layout.
    * 
    * @param layout Layout where the slider will be added.
    * @param labelText Text for the slider label.
    * @param slider Pointer to the QSlider instance.
    * @param spinBox Pointer to the QSpinBox instance.
    * @param min Minimum value for the slider.
    * @param max Maximum value for the slider.
    * @param step Step value for the slider.
    * @param value Initial value for the slider.
    */
    void addSlider(QGridLayout* layout, const QString& label, QSlider*& slider, QDoubleSpinBox*& spinBox, int min, int max, int singleStep, int value);

    /**
    * @brief Connect sliders with spin boxes for synchronized updates.
    */
    void connectSlidersWithSpinBoxes();

protected:
    /**
    * @brief Handles the resizing of the widget and updates the displayed images.
    * 
    * This function is called automatically when the widget is resized. It performs the following actions:
    * - Loads images for different board states from the package directory.
    * - Scales these images to fit the size of the board labels while maintaining their aspect ratio.
    * - Updates the pixmap for each board label based on the current state of the board.
    * - Scales and updates the video feed image to fit the size of the video label while maintaining its aspect ratio.
    * 
    * @param event A pointer to the QResizeEvent object that contains information about the resize event.
    * 
    * @note This function uses `ament_index_cpp::get_package_share_directory` to obtain the directory of the package
    *       and load images from it. It assumes that the images are located in the "images" subdirectory of the package's
    *       share directory.
    */
    void resizeEvent(QResizeEvent *event) override {
        case0 = QPixmap(QString::fromStdString(ament_index_cpp::get_package_share_directory("umi_rtx_controller")+"/images/grid0.png"));
        case1 = QPixmap(QString::fromStdString(ament_index_cpp::get_package_share_directory("umi_rtx_controller")+"/images/grid1.png"));
        case2 = QPixmap(QString::fromStdString(ament_index_cpp::get_package_share_directory("umi_rtx_controller")+"/images/grid2.png"));

        case0 = case0.scaled(board_labels[0][0]->size(), Qt::KeepAspectRatio);
        case1 = case1.scaled(board_labels[0][0]->size(), Qt::KeepAspectRatio);
        case2 = case2.scaled(board_labels[0][0]->size(), Qt::KeepAspectRatio);

        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                if(board[row][col] == 1)
                    board_labels[row][col]->setPixmap(case1);
                else if(board[row][col] == 2)
                    board_labels[row][col]->setPixmap(case2);
                else
                    board_labels[row][col]->setPixmap(case0);
            }
        }
        QImage scaledImage = image->scaled(videoLabel->size(), Qt::KeepAspectRatio);

        videoLabel->setPixmap(QPixmap::fromImage(scaledImage));

        event->accept();
    }

};

#endif