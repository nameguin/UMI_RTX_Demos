#include "umi_rtx_controller/main_gui.hpp"

MainGUI::MainGUI(QApplication * app,
                 const std::shared_ptr<Objective_node>& ros2_node, 
                 rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, 
                 QWidget* parent)
  : app_(app), ros2_node(ros2_node), rviz_ros_node_(rviz_ros_node), QMainWindow(parent)
{

    // Widget and layout initialization
    main_widget = new QWidget(this);
    main_layout = new QHBoxLayout(main_widget);
    game_layout = new QVBoxLayout;
    board_layout = new QGridLayout;
    info_layout = new QVBoxLayout;
    history_layout = new QVBoxLayout;
    umi_layout = new QVBoxLayout;

    // Setting spacing and margins
    main_layout->setSpacing(10);
    main_layout->setMargin(10);
    board_layout->setContentsMargins(0, 0, 0, 0);
    board_layout->setSpacing(0);

    // Title initialization
    Title = new QLabel("UMI-RTX Interface");
    Title->setAlignment(Qt::AlignHCenter);
    info_layout->addWidget(Title);

    // Images initialization
    case0 = QPixmap(QString::fromStdString(ament_index_cpp::get_package_share_directory("umi_rtx_controller") + "/images/grid0.png"));
    case1 = QPixmap(QString::fromStdString(ament_index_cpp::get_package_share_directory("umi_rtx_controller") + "/images/grid1.png"));
    case2 = QPixmap(QString::fromStdString(ament_index_cpp::get_package_share_directory("umi_rtx_controller") + "/images/grid2.png"));

    // Loop to create board labels
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            board_labels[i][j] = new QLabel;
            board_labels[i][j]->setScaledContents(true);

            board_labels[i][j]->setPixmap(case0);
            board_labels[i][j]->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

            // Adding QLabel to the grid
            board_layout->addWidget(board_labels[i][j], i, j);
        }
    }

    //////////////////////////////////////////////// Sliders /////////////////////////////////////////////////

    // Sliders layout creation
    QGridLayout* sliders_layout = new QGridLayout;

    // Adding sliders for X, Y, Z, Yaw, Pitch, Roll, Grip
    QSlider* slider_x = new QSlider(Qt::Horizontal);
    QSlider* slider_y = new QSlider(Qt::Horizontal);
    QSlider* slider_z = new QSlider(Qt::Horizontal);
    QSlider* slider_yaw = new QSlider(Qt::Horizontal);
    QSlider* slider_pitch = new QSlider(Qt::Horizontal);
    QSlider* slider_roll = new QSlider(Qt::Horizontal);
    QSlider* slider_grip = new QSlider(Qt::Horizontal);

    addSlider(sliders_layout, "X :", slider_x, spinBox_x, -60, 60, 1, 60);
    addSlider(sliders_layout, "Y :", slider_y, spinBox_y, 20, 70, 1, 70);
    addSlider(sliders_layout, "Z :", slider_z, spinBox_z, 10, 70, 1, 70);
    addSlider(sliders_layout, "Yaw :", slider_yaw, spinBox_yaw, -110, 110, 1, 110);
    addSlider(sliders_layout, "Pitch :", slider_pitch, spinBox_pitch, 0, 90, 1, 90);
    addSlider(sliders_layout, "Roll :", slider_roll, spinBox_roll, 0, 90, 1, 90);
    addSlider(sliders_layout, "Grip :", slider_grip, spinBox_grip, 20, 80, 1, 80);

    // Connecting sliders with spin boxes
    connectSlidersWithSpinBoxes();

    // Initial values for manual mode
    slider_x->setValue(0);
    slider_y->setValue(68);
    slider_z->setValue(60);
    slider_grip->setValue(20);

    //////////////////////////////////////////////// Switch Button /////////////////////////////////////////////////

    // Button to switch between manual and automatic mode
    QPushButton* switchButton = new QPushButton(this);
    switchButton->setCheckable(true);
    switchButton->setChecked(true);
    // Personnalisation de l'apparence du switch
    switchButton->setFixedHeight(50);
    switchButton->setText("Manual mode");
    switchButton->setStyleSheet("QPushButton {"
                                "border: none;"
                                "background-color: #ccc;"
                                "border-radius: 10px"
                                "}"
                                "QPushButton:checked {"
                                    "background-color: #6c6;"
                                "}"
                                "QPushButton:!checked {"
                                    "background-color: #ccc;"
                                "}");

    // Connection of the clicked signal to the corresponding slot to react to clicks on the switch
    connect(switchButton, &QPushButton::clicked, this, [=]() {
        manual_on = switchButton->isChecked();
        if (manual_on){
            ros2_node->mode = "manual";
            switchButton->setText("Manual mode");
        }
        else {
            ros2_node->mode = "grab";
            switchButton->setText("Grab mode");
        }
    });

    //////////////////////////////////////////////// Image Button /////////////////////////////////////////////////

    // Button to switch between depth and processed image
    QPushButton* imageButton = new QPushButton(this);
    imageButton->setCheckable(true);
    imageButton->setChecked(false);
    // Personnalisation de l'apparence du image
    imageButton->setFixedHeight(50);
    imageButton->setText("Image displayed");
    imageButton->setStyleSheet("QPushButton {"
                                "border: none;"
                                "background-color: #ccc;"
                                "border-radius: 10px"
                                "}"
                                "QPushButton:checked {"
                                    "background-color: #6c6;"
                                "}"
                                "QPushButton:!checked {"
                                    "background-color: #ccc;"
                                "}");

    // Connection of the clicked signal to the corresponding slot to react to clicks on the image
    connect(imageButton, &QPushButton::clicked, this, [=]() {
        depth_frame = imageButton->isChecked();
        if (depth_frame){
            imageButton->setText("Depth displayed");
        }
        else {
            imageButton->setText("Image displayed");
        }
    });
    

    //////////////////////////////////////////////// 3D Simulation /////////////////////////////////////////////////

    // Initialize RViz configuration
    initializeRViz();

    // Add RViz widget to the interface
    render_panel_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    //////////////////////////////////////////////// Camera Image /////////////////////////////////////////////////

    videoLabel = new QLabel("");
    frame = new cv::Mat;
    image = new QImage;
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this](){
        updateFrameAndInterface();
        double  up_x = this->ros2_node->x,
                up_y = this->ros2_node->y,
                up_z = this->ros2_node->z,
                up_pitch = this->ros2_node->pitch,
                up_roll = this->ros2_node->roll;
                // up_yaw = this->ros2_node->yaw,
        spinBox_x->setValue(100*up_x);
        spinBox_y->setValue(100*up_y);
        spinBox_z->setValue(100*up_z);
        // spinBox_yaw->setValue(up_yaw);
        spinBox_pitch->setValue(up_pitch);
        spinBox_roll->setValue(up_roll);
    });
    timer->start(40);
    videoLabel->setAlignment(Qt::AlignCenter);

    //////////////////////////////////////////////// Information Layout /////////////////////////////////////////////////

    turn_label = new QLabel("Turn " + QString::number(ros2_node->turn));
    turn_label->setAlignment(Qt::AlignCenter); // Alignement centré
    turn_label->setStyleSheet("font-size: 40px;"); // Taille de police plus grande
    player_label = new QLabel("");
    player_label->setStyleSheet("font-size: 20px;");
    player_label->setAlignment(Qt::AlignCenter); // Alignement centré

    for(int i = 0; i < 9; i++){
        info_labels[i] = new QLabel("");
        info_labels[i]->setAlignment(Qt::AlignCenter);
        info_labels[i]->setContentsMargins(0, 10, 0, 10);
        history_layout->addWidget(info_labels[i]);
    }

    info_layout->addWidget(turn_label);
    info_layout->addWidget(player_label);
    info_layout->addLayout(history_layout);

    turn_label->setContentsMargins(0, 0, 0, 0);
    history_layout->setContentsMargins(0, 50, 0, 150);
    player_label->setContentsMargins(0, 50, 0, 100);


    //////////////////////////////////////////////// Main Layout /////////////////////////////////////////////////

    game_layout->addLayout(board_layout);
    game_layout->addWidget(videoLabel);
    game_layout->addWidget(imageButton);

    umi_layout->addWidget(render_panel_);
    umi_layout->addLayout(sliders_layout);
    umi_layout->addWidget(switchButton);

    main_layout->addLayout(game_layout);
    main_layout->addLayout(info_layout);
    main_layout->addLayout(umi_layout);

    main_layout->setStretch(0, 1);
    main_layout->setStretch(1, 1);
    main_layout->setStretch(2, 1);

    main_widget->setLayout(main_layout);
    setCentralWidget(main_widget);
    setStyleSheet("background-color: #e0e8bd;");

    setWindowIcon(QIcon(QString::fromStdString(ament_index_cpp::get_package_share_directory("umi_rtx_controller")+"/images/icon.png")));
}

MainGUI::~MainGUI()
{
    // capture.release();
}


void MainGUI::initializeRViz()
{
    app_->processEvents();
    render_panel_ = new rviz_common::RenderPanel(main_widget);
    app_->processEvents();
    render_panel_->getRenderWindow()->initialize();
    auto clock = rviz_ros_node_.lock()->get_raw_node()->get_clock();
    manager_ = new rviz_common::VisualizationManager(render_panel_, rviz_ros_node_, this, clock);
    render_panel_->initialize(manager_);
    app_->processEvents();

    // Add TF and model in the integrated window
    TF_ = manager_->createDisplay("rviz_default_plugins/TF","TF",true);
    Model_ = manager_->createDisplay("rviz_default_plugins/RobotModel","RobotModel",true);
    assert(TF_ != NULL);
    assert(Model_ != NULL);

    // Subscribe to the description topic to see the model
    Model_->setTopic(QString::fromStdString("/robot_description"),QString::fromStdString("std_msgs/msg/String"));

    // Necessary to move the camera
    manager_->getToolManager()->addTool(QString::fromStdString("rviz_default_plugins/MoveCamera"));
    // render_panel_->getViewController()->subProp(QString::fromStdString("Distance"))->setValue(2.0);


    manager_->initialize();
    manager_->startUpdate();
}

void MainGUI::addSlider(QGridLayout* layout, const QString& label, QSlider*& slider, QDoubleSpinBox*& spinBox, int min, int max, int singleStep, int value)
{
    QLabel* labelWidget = new QLabel(label);
    labelWidget->setAlignment(Qt::AlignRight);

    slider = new QSlider(Qt::Horizontal);
    slider->setRange(min, max);
    slider->setSingleStep(singleStep);
    slider->setValue(value);

    spinBox = new QDoubleSpinBox;
    spinBox->setMinimum(min);
    spinBox->setMaximum(max);
    spinBox->setSingleStep(singleStep);
    spinBox->setValue(value);

    // Connect slider and spin box
    QObject::connect(slider, &QSlider::valueChanged, spinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::setValue));
    QObject::connect(spinBox, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), slider, &QSlider::setValue);

    // Add widgets to layout
    layout->addWidget(labelWidget, layout->rowCount(), 0);
    layout->addWidget(slider, layout->rowCount() - 1, 1);
    layout->addWidget(spinBox, layout->rowCount() - 1, 2);
}

// Method to connect sliders with spin boxes
void MainGUI::connectSlidersWithSpinBoxes()
{
    QObject::connect(spinBox_x, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [=](double newValue){
        x = newValue / 100;
        yaw = atan2(y, x) * 180 / M_PI + raw_yaw;
    });
    QObject::connect(spinBox_y, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [=](double newValue){
        y = newValue/100;
        yaw = atan2(y,x)*180/M_PI + raw_yaw;
    });
    QObject::connect(spinBox_z, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [=](double newValue){
        z = newValue/100;
    });
    QObject::connect(spinBox_yaw, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [=](double newValue){
        yaw = atan2(y,x)*180/M_PI + newValue;
        raw_yaw = newValue;
    });
    QObject::connect(spinBox_pitch, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [=](double newValue){
        pitch = newValue;
    });
    QObject::connect(spinBox_roll, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [=](double newValue){
        roll = newValue;
    });
    QObject::connect(spinBox_grip, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [=](double newValue){
        grip = newValue/1000;
    });
}

// Method to update the frame and interface elements
void MainGUI::updateFrameAndInterface()
{
    // Choose which frame to display based on the depth_frame flag
    cv::Mat* selectedFrame = (depth_frame) ? &(ros2_node->depth_frame) : &(ros2_node->processed_frame);

    // Resize the frame to reduce calculations
    cv::Size newSize(640, 360);
    if (selectedFrame->cols > 0 && selectedFrame->rows > 0) {
        cv::resize(*selectedFrame, *selectedFrame, newSize);
    }

    // Convert the frame to QImage
    *image = QImage(selectedFrame->data, selectedFrame->cols, selectedFrame->rows, selectedFrame->step, QImage::Format_RGB888).rgbSwapped();

    // Scale the image to fit the QLabel
    if (!image->isNull()){
        *image = image->scaled(videoLabel->size(), Qt::KeepAspectRatio);

        // Set the image as pixmap of the QLabel
        videoLabel->setPixmap(QPixmap::fromImage(*image));
        videoLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    }

    // Update game board and interface elements if needed
    if (ros2_node->need_update) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                board[i][j] = ros2_node->board[i][j];
                QPixmap pixmap;
                if (board[i][j] == 1) {
                    pixmap = case1;
                } else if (board[i][j] == 2) {
                    pixmap = case2;
                } else {
                    pixmap = case0;
                }

                // Set the image on the QLabel
                board_labels[i][j]->setPixmap(pixmap);

                // Add the QLabel to the grid layout
                board_layout->addWidget(board_labels[i][j], i, j);
            }
        }

        // Update game status display
        if (ros2_node->result != -1) {
            turn_label->setText("Game ended");
            if (ros2_node->result == 0)
                player_label->setText("Draw !");
            else if (ros2_node->result == 1)
                player_label->setText("Robot won !");
            else
                player_label->setText("Human won !");
            player_label->setStyleSheet("font-size: 26px;");
        } else {
            turn_label->setText("Turn " + QString::number(ros2_node->turn));

            if (ros2_node->player_turn == 1)
                player_label->setText("Human");
            else
                player_label->setText("Robot");
            player_label->setStyleSheet("font-size: 20px;");
        }

        // Update recent messages display
        for (int i = 0; i < ros2_node->recent_msgs.size(); i++) {
            info_labels[i]->setText(QString::fromStdString(ros2_node->recent_msgs[i]));
        }

        ros2_node->need_update = false;
    }
}


// Necessary to build but useless functions in our case
QWidget *
MainGUI::getParentWindow()
{
  return this;
}

rviz_common::PanelDockWidget *
MainGUI::addPane(const QString & name, QWidget * pane, Qt::DockWidgetArea area, bool floating)
{
  return nullptr;
}

void
MainGUI::setStatus(const QString & message)
{
}

void MainGUI::closeEvent(QCloseEvent * event)
{
  QMainWindow::closeEvent(event);
  rclcpp::shutdown();
}

static void siginthandler(int /*param*/)
{
    QApplication::quit();
}


int main(int argc, char* argv[])
{   
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);

    rclcpp::init(argc, argv);
    auto ros_node_abs = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_render_node");
    auto ros2_node = std::make_shared<Objective_node>();
    auto gui_app = std::make_shared<MainGUI>(&app,ros2_node,ros_node_abs);

    app.processEvents();
    gui_app->showMaximized();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(ros2_node);

    while (rclcpp::ok())
    {   
        if (ros2_node->mode=="manual"){
            ros2_node->update_state(gui_app->x, gui_app->y, gui_app->z, gui_app->yaw, gui_app->pitch, gui_app->roll, gui_app->grip);
        }
        exec.spin_some();
        app.processEvents();
    }
    signal(SIGINT, siginthandler);

    exec.remove_node(ros2_node);
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
