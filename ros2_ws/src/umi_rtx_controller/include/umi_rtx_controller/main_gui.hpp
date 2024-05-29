/**
 * @file main_gui.hpp
 * @author Théo MASSA (theo.massa@ensta-bretagne.org)
 * @brief Main window used for the custom Graphic User Interface (GUI) of our ROS2 interface
 * @version 0.1
 * @date 2023-07-19
 * 
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

/**
 * @brief MainGUI class, defines our custom interface
 */
class MainGUI : public QMainWindow, public rviz_common::WindowManagerInterface {
public:
    /**
     * @brief Construct a new MainGUI object
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
     * @param floating Don't know the utility of this one
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
    QImage* image;
    QLabel *Title;
    QLabel* videoLabel;
    QTimer* timer;
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

    // cv::VideoCapture capture;
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
     * @brief Function to update the video frame     * 
     */
    void updateFrameAndInterface();

    void addSlider(QGridLayout* layout, const QString& label, QSlider*& slider, QDoubleSpinBox*& spinBox, int min, int max, int singleStep, int value);

    void connectSlidersWithSpinBoxes();

protected:
    void resizeEvent(QResizeEvent *event) override {
        // Redimensionner les images pour qu'elles s'adaptent aux labels
        case0 = QPixmap(QString::fromStdString(ament_index_cpp::get_package_share_directory("umi_rtx_controller")+"/images/grid0.png"));
        case1 = QPixmap(QString::fromStdString(ament_index_cpp::get_package_share_directory("umi_rtx_controller")+"/images/grid1.png"));
        case2 = QPixmap(QString::fromStdString(ament_index_cpp::get_package_share_directory("umi_rtx_controller")+"/images/grid2.png"));

        case0 = case0.scaled(board_labels[0][0]->size(), Qt::KeepAspectRatio);
        case1 = case1.scaled(board_labels[0][0]->size(), Qt::KeepAspectRatio);
        case2 = case2.scaled(board_labels[0][0]->size(), Qt::KeepAspectRatio);

                // Définir les images redimensionnées comme contenu des labels
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

        // Définir les images redimensionnées comme contenu des labels
        videoLabel->setPixmap(QPixmap::fromImage(scaledImage));

        event->accept();
    }

};

#endif