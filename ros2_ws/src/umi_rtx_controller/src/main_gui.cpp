#include "umi_rtx_controller/main_gui.hpp"

MainGUI::MainGUI(QApplication * app,
                 const std::shared_ptr<Objective_node>& ros2_node, 
                 rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, 
                 QWidget* parent)
  : app_(app), ros2_node(ros2_node), rviz_ros_node_(rviz_ros_node), QMainWindow(parent)
{

    main_widget = new QWidget(this);
    main_layout = new QHBoxLayout(main_widget);

    game_layout = new QVBoxLayout;
    board_layout = new QGridLayout;
    info_layout = new QVBoxLayout;
    history_layout = new QVBoxLayout;
    umi_layout = new QVBoxLayout;

    main_layout->setSpacing(10);
    main_layout->setMargin(10);

    board_layout->setContentsMargins(0, 0, 0, 0);
    board_layout->setSpacing(0);


    Title = new QLabel("UMI-RTX Interface");
    Title->setAlignment(Qt::AlignHCenter);
    info_layout->addWidget(Title);

    // Redimensionnement des images si nécessaire
    case0 = QPixmap(QString::fromStdString(ament_index_cpp::get_package_share_directory("umi_rtx_controller")+"/images/grid0.png"));
    case1 = QPixmap(QString::fromStdString(ament_index_cpp::get_package_share_directory("umi_rtx_controller")+"/images/grid1.png"));
    case2 = QPixmap(QString::fromStdString(ament_index_cpp::get_package_share_directory("umi_rtx_controller")+"/images/grid2.png"));

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            board_labels[i][j] = new QLabel;
            board_labels[i][j]->setScaledContents(true); 

            if(board[i][j] == 0)
                board_labels[i][j]->setPixmap(case0);
            else if(board[i][j] == 1)
                board_labels[i][j]->setPixmap(case1);
            else
                board_labels[i][j]->setPixmap(case2);

            board_labels[i][j]->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

            // Ajouter le QLabel à la grille
            board_layout->addWidget(board_labels[i][j], i, j);
        }
    }
    game_layout->addLayout(board_layout);


    ///////////////////////////// Add sliders /////////////////////////////////

    QGridLayout* sliders_layout = new QGridLayout;
    
    QLabel *label_x = new QLabel("X :");
    label_x->setAlignment(Qt::AlignRight);
    sliders_layout->addWidget(label_x,1,0);
    // Slider creation, to control x position
    QSlider* slider_x = new QSlider(Qt::Horizontal);
    slider_x->setRange(-60, 60); // range of values
    slider_x->setSingleStep(1); 
    sliders_layout->addWidget(slider_x,1,1);

    spinBox_x = new QDoubleSpinBox;
    spinBox_x->setMaximum(60);
    spinBox_x->setMinimum(-60);
    spinBox_x->setSingleStep(1);
    sliders_layout->addWidget(spinBox_x,1,2);
    // Link slider and spinBox
    QObject::connect(slider_x, &QSlider::valueChanged, spinBox_x, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::setValue));
    QObject::connect(spinBox_x, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), slider_x, &QSlider::setValue);


    // Same for y
    QLabel *label_y = new QLabel("Y :");
    label_y->setAlignment(Qt::AlignRight);
    sliders_layout->addWidget(label_y,2,0);

    QSlider* slider_y = new QSlider(Qt::Horizontal);
    slider_y->setRange(20, 70); 
    slider_y->setSingleStep(1);
    sliders_layout->addWidget(slider_y,2,1);

    spinBox_y = new QDoubleSpinBox;
    spinBox_y->setMaximum(70);
    spinBox_y->setMinimum(20);
    sliders_layout->addWidget(spinBox_y,2,2);

    QObject::connect(slider_y, &QSlider::valueChanged, spinBox_y, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::setValue));
    QObject::connect(spinBox_y, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), slider_y, &QSlider::setValue);


    QLabel *label_z = new QLabel("Z :");
    label_z->setAlignment(Qt::AlignRight);
    sliders_layout->addWidget(label_z,3,0);

    QSlider* slider_z = new QSlider(Qt::Horizontal);
    slider_z->setRange(10, 70); 
    slider_z->setSingleStep(1);
    sliders_layout->addWidget(slider_z,3,1);

    spinBox_z = new QDoubleSpinBox;
    spinBox_z->setMaximum(70);
    spinBox_z->setMinimum(10);
    sliders_layout->addWidget(spinBox_z,3,2);

    QObject::connect(slider_z, &QSlider::valueChanged, spinBox_z, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::setValue));
    QObject::connect(spinBox_z, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), slider_z, &QSlider::setValue);

    QLabel *label_yaw = new QLabel("Yaw :");
    label_yaw->setAlignment(Qt::AlignRight);
    sliders_layout->addWidget(label_yaw,4,0);
    QSlider* slider_yaw = new QSlider(Qt::Horizontal);
    slider_yaw->setRange(-110, 110);
    slider_yaw->setSingleStep(1);
    sliders_layout->addWidget(slider_yaw,4,1);
    spinBox_yaw = new QDoubleSpinBox;
    spinBox_yaw->setMaximum(110);
    spinBox_yaw->setMinimum(-110);
    spinBox_yaw->setSingleStep(1);
    sliders_layout->addWidget(spinBox_yaw,4,2);
    QObject::connect(slider_yaw, &QSlider::valueChanged, spinBox_yaw, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::setValue));
    QObject::connect(spinBox_yaw, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), slider_yaw, &QSlider::setValue);

    QLabel *label_pitch = new QLabel("Pitch :");
    label_pitch->setAlignment(Qt::AlignRight);
    sliders_layout->addWidget(label_pitch,5,0);
    QSlider* slider_pitch = new QSlider(Qt::Horizontal);
    slider_pitch->setRange(0, 90);
    slider_pitch->setSingleStep(1);
    sliders_layout->addWidget(slider_pitch,5,1);
    spinBox_pitch = new QDoubleSpinBox;
    spinBox_pitch->setMaximum(90);
    spinBox_pitch->setMinimum(0);
    spinBox_pitch->setSingleStep(1);
    sliders_layout->addWidget(spinBox_pitch,5,2);
    QObject::connect(slider_pitch, &QSlider::valueChanged, spinBox_pitch, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::setValue));
    QObject::connect(spinBox_pitch, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), slider_pitch, &QSlider::setValue);

    QLabel *label_roll = new QLabel("Roll :");
    label_roll->setAlignment(Qt::AlignRight);
    sliders_layout->addWidget(label_roll,6,0);
    QSlider* slider_roll = new QSlider(Qt::Horizontal);
    slider_roll->setRange(0, 90);
    slider_roll->setSingleStep(1);
    sliders_layout->addWidget(slider_roll,6,1);
    spinBox_roll = new QDoubleSpinBox;
    spinBox_roll->setMaximum(90);
    spinBox_roll->setMinimum(0);
    spinBox_roll->setSingleStep(1);
    sliders_layout->addWidget(spinBox_roll,6,2);
    QObject::connect(slider_roll, &QSlider::valueChanged, spinBox_roll, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::setValue));
    QObject::connect(spinBox_roll, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), slider_roll, &QSlider::setValue);

    QLabel *label_grip = new QLabel("Grip :");
    label_grip->setAlignment(Qt::AlignRight);
    sliders_layout->addWidget(label_grip,7,0);
    QSlider* slider_grip = new QSlider(Qt::Horizontal);
    slider_grip->setRange(20, 80);
    slider_grip->setSingleStep(1);
    sliders_layout->addWidget(slider_grip,7,1);
    spinBox_grip = new QDoubleSpinBox;
    spinBox_grip->setMaximum(80);
    spinBox_grip->setMinimum(20);
    spinBox_grip->setSingleStep(1);
    sliders_layout->addWidget(spinBox_grip,7,2);
    QObject::connect(slider_grip, &QSlider::valueChanged, spinBox_grip, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::setValue));
    QObject::connect(spinBox_grip, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), slider_grip, &QSlider::setValue);


    QObject::connect(spinBox_x, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [=](double newValue){
        x = newValue/100;
        yaw = atan2(y,x)*180/M_PI + raw_yaw;
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

    // Initial values for manual mode
    slider_x->setValue(0);
    slider_y->setValue(68);
    slider_z->setValue(60);
    slider_grip->setValue(20);

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


 // Initialize RViz configuration
    initializeRViz();

    // Add RViz widget to the interface
    render_panel_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);


    umi_layout->addWidget(render_panel_);
    umi_layout->addLayout(sliders_layout);
    umi_layout->addWidget(switchButton);


    videoLabel = new QLabel("");
    frame = new cv::Mat;
    image = new QImage;
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this](){
        updateFrame();
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
    game_layout->addWidget(videoLabel);
    game_layout->addWidget(imageButton);

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

void MainGUI::updateFrame()
{   
    if (depth_frame){
        *frame = ros2_node->depth_frame;
    }
    else {
        *frame = ros2_node->processed_frame;
    }
    int w = frame->cols,h = frame->rows;

    cv::Size newSize(640,360); // resize frame to do less calculations
    if (w > 0 && h > 0){
        cv::resize(*frame,*frame,newSize);
    }

    *image = QImage(frame->data, frame->cols, frame->rows, frame->step, QImage::Format_RGB888).rgbSwapped();
    if (!image->isNull())
        *image = image->scaled(videoLabel->size(), Qt::KeepAspectRatio);
    videoLabel->setPixmap(QPixmap::fromImage(*image));
    videoLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    
    if(ros2_node->need_update){
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                board[i][j] = ros2_node->board[i][j];
                // Sélectionner l'image en fonction de la valeur dans le tableau
                QPixmap pixmap;
                if (board[i][j] == 1) {
                    pixmap = case1;
                } else if (board[i][j] == 2) {
                    pixmap = case2;
                } else {
                    pixmap = case0;
                }

                // Définir l'image sur le QLabel
                board_labels[i][j]->setPixmap(pixmap);

                // Ajouter le QLabel à la grille
                board_layout->addWidget(board_labels[i][j], i, j);
            }
        }
        
        if(ros2_node->result != -1) {
            turn_label->setText("Game ended");
            if(ros2_node->result == 0)
                player_label->setText("Draw !");
            else if(ros2_node->result == 1)
                player_label->setText("Robot won !");
            else
                player_label->setText("Human won !");
            player_label->setStyleSheet("font-size: 26px;");
        }
        else {
            turn_label->setText("Turn " + QString::number(ros2_node->turn));

            if(ros2_node->player_turn == 1)
                player_label->setText("Human");
            else
                player_label->setText("Robot");
            player_label->setStyleSheet("font-size: 20px;");
        }
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
