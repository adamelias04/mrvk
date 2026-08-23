// ROS2 port of the Gazebo Classic GUI plugin that publishes a 2D navigation
// goal (X/Y/Yaw form) to /goal_pose for Nav2. Uses rclcpp instead of roscpp.
// The click-on-map goal tool is provided by RViz2's built-in "2D Goal Pose".

#include <gazebo/gui/gui.hh>
#include <gazebo/gui/GuiPlugin.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <sdf/sdf.hh>

#include <QWidget>
#include <QFrame>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QFont>
#include <QString>

#include <memory>
#include <string>
#include <thread>

namespace gazebo
{

class GazeboNavGoalGui : public GUIPlugin
{
  Q_OBJECT

public:
  GazeboNavGoalGui()
      : GUIPlugin(),
        topic_("/goal_pose"),
        default_frame_("odom"),
        pos_x_(170),
        pos_y_(20)
  {
    this->setObjectName("gazeboNavGoalGui");
    this->resize(620, 430);
    this->move(pos_x_, pos_y_);

    auto *outerLayout = new QVBoxLayout();
    outerLayout->setContentsMargins(0, 0, 0, 0);
    outerLayout->setSpacing(0);

    auto *panel = new QFrame();
    panel->setObjectName("mainPanel");
    panel->setFixedSize(620, 430);
    panel->setStyleSheet(
        "#mainPanel { background-color: rgba(24, 24, 24, 245); border: 2px solid #3f3f3f; }"
        "QLabel { color: white; }"
        "QLineEdit { background: white; color: black; border: 1px solid #666;"
        "  border-radius: 2px; padding-left: 8px; padding-right: 8px; }"
        "QPushButton { background-color: #5f5f5f; color: white;"
        "  border: 1px solid #8a8a8a; font-weight: bold; }"
        "QPushButton:hover { background-color: #737373; }");

    auto *mainLayout = new QVBoxLayout(panel);
    mainLayout->setContentsMargins(18, 18, 18, 18);
    mainLayout->setSpacing(14);

    auto *title = new QLabel("2D Nav Goal");
    title->setFont(QFont("Sans", 20, QFont::Bold));
    title->setFixedHeight(34);

    auto *desc = new QLabel(
        "Enter X, Y and yaw; the plugin publishes a PoseStamped\n"
        "to /goal_pose for Nav2.");
    desc->setFont(QFont("Sans", 11));
    desc->setWordWrap(true);
    desc->setFixedHeight(42);

    auto *grid = new QGridLayout();
    grid->setHorizontalSpacing(12);
    grid->setVerticalSpacing(10);

    QLabel *topicLabel = new QLabel("Topic:");
    QLabel *frameLabel = new QLabel("Frame:");
    QLabel *xLabel = new QLabel("X [m]:");
    QLabel *yLabel = new QLabel("Y [m]:");
    QLabel *yawLabel = new QLabel("Yaw [rad]:");
    QFont labelFont("Sans", 12, QFont::Bold);
    for (QLabel *l : {topicLabel, frameLabel, xLabel, yLabel, yawLabel})
    { l->setFont(labelFont); l->setFixedHeight(28); }

    topicEdit_ = new QLineEdit(QString::fromStdString(topic_));
    frameEdit_ = new QLineEdit(QString::fromStdString(default_frame_));
    xEdit_ = new QLineEdit("0.0");
    yEdit_ = new QLineEdit("0.0");
    yawEdit_ = new QLineEdit("0.0");
    QFont editFont("Sans", 12);
    for (QLineEdit *e : {topicEdit_, frameEdit_, xEdit_, yEdit_, yawEdit_})
    { e->setFont(editFont); e->setFixedHeight(34); e->setMinimumWidth(390); }

    grid->addWidget(topicLabel, 0, 0); grid->addWidget(topicEdit_, 0, 1);
    grid->addWidget(frameLabel, 1, 0); grid->addWidget(frameEdit_, 1, 1);
    grid->addWidget(xLabel, 2, 0);     grid->addWidget(xEdit_, 2, 1);
    grid->addWidget(yLabel, 3, 0);     grid->addWidget(yEdit_, 3, 1);
    grid->addWidget(yawLabel, 4, 0);   grid->addWidget(yawEdit_, 4, 1);
    grid->setColumnMinimumWidth(0, 95);
    grid->setColumnMinimumWidth(1, 390);
    grid->setColumnStretch(1, 1);

    auto *buttonRow = new QHBoxLayout();
    buttonRow->setSpacing(14);
    sendButton_ = new QPushButton("Send goal");
    zeroButton_ = new QPushButton("Zero");
    QFont buttonFont("Sans", 12, QFont::Bold);
    sendButton_->setFont(buttonFont); sendButton_->setFixedHeight(46);
    zeroButton_->setFont(buttonFont); zeroButton_->setFixedHeight(46);
    buttonRow->addWidget(sendButton_);
    buttonRow->addWidget(zeroButton_);

    statusLabel_ = new QLabel("Status: waiting");
    statusLabel_->setFont(QFont("Sans", 11));
    statusLabel_->setWordWrap(true);
    statusLabel_->setFixedHeight(54);
    statusLabel_->setStyleSheet(
        "background-color: rgba(255,255,255,0.05); border: 1px solid #555;"
        "padding: 8px; color: #f0f0f0;");

    mainLayout->addWidget(title);
    mainLayout->addWidget(desc);
    mainLayout->addLayout(grid);
    mainLayout->addLayout(buttonRow);
    mainLayout->addWidget(statusLabel_);

    outerLayout->addWidget(panel);
    this->setLayout(outerLayout);

    connect(sendButton_, SIGNAL(clicked()), this, SLOT(OnSendGoal()));
    connect(zeroButton_, SIGNAL(clicked()), this, SLOT(OnZero()));

    InitializeRosPublisher();
  }

  ~GazeboNavGoalGui() override
  {
    if (executor_)
      executor_->cancel();
    if (spin_thread_.joinable())
      spin_thread_.join();
  }

  void Load(sdf::ElementPtr _sdf) override
  {
    if (_sdf)
    {
      if (_sdf->HasElement("topic"))
        topic_ = _sdf->Get<std::string>("topic");
      if (_sdf->HasElement("default_frame"))
        default_frame_ = _sdf->Get<std::string>("default_frame");
      if (_sdf->HasElement("x"))
        pos_x_ = _sdf->Get<int>("x");
      if (_sdf->HasElement("y"))
        pos_y_ = _sdf->Get<int>("y");
    }

    this->move(pos_x_, pos_y_);
    topicEdit_->setText(QString::fromStdString(topic_));
    frameEdit_->setText(QString::fromStdString(default_frame_));

    InitializeRosPublisher();
    statusLabel_->setText(
        QString("Status: plugin loaded, topic=%1, frame=%2")
            .arg(QString::fromStdString(topic_))
            .arg(QString::fromStdString(default_frame_)));
  }

private slots:
  void OnZero()
  {
    xEdit_->setText("0.0");
    yEdit_->setText("0.0");
    yawEdit_->setText("0.0");
    statusLabel_->setText("Status: values reset to zero");
  }

  void OnSendGoal()
  {
    if (!node_)
    {
      statusLabel_->setText("Status: ROS not initialized");
      return;
    }

    bool okX = false, okY = false, okYaw = false;
    const double x = xEdit_->text().toDouble(&okX);
    const double y = yEdit_->text().toDouble(&okY);
    const double yaw = yawEdit_->text().toDouble(&okYaw);
    if (!okX || !okY || !okYaw)
    {
      statusLabel_->setText("Status: invalid numbers in X/Y/Yaw");
      return;
    }

    const std::string topic = topicEdit_->text().toStdString();
    const std::string frame = frameEdit_->text().toStdString();
    if (topic.empty()) { statusLabel_->setText("Status: topic must not be empty"); return; }
    if (frame.empty()) { statusLabel_->setText("Status: frame must not be empty"); return; }

    if (topic != topic_)
    {
      topic_ = topic;
      InitializeRosPublisher();
    }

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = node_->now();
    goal.header.frame_id = frame;
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    goal_pub_->publish(goal);

    statusLabel_->setText(
        QString("Status: sent -> frame=%1  x=%2  y=%3  yaw=%4")
            .arg(QString::fromStdString(frame))
            .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(yaw, 0, 'f', 3));

    RCLCPP_INFO(node_->get_logger(),
                "[gazebo_nav_goal_gui] Published goal on %s frame=%s x=%f y=%f yaw=%f",
                topic_.c_str(), frame.c_str(), x, y, yaw);
  }

private:
  void InitializeRosPublisher()
  {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    if (!node_)
    {
      rclcpp::NodeOptions opts;
      opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
      node_ = std::make_shared<rclcpp::Node>("gazebo_nav_goal_gui", opts);
      // Spin in the background so /clock is received and now() = sim time.
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(node_);
      spin_thread_ = std::thread([this]() { executor_->spin(); });
    }
    goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_, 1);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;

  std::string topic_;
  std::string default_frame_;
  int pos_x_;
  int pos_y_;

  QLineEdit *topicEdit_;
  QLineEdit *frameEdit_;
  QLineEdit *xEdit_;
  QLineEdit *yEdit_;
  QLineEdit *yawEdit_;
  QPushButton *sendButton_;
  QPushButton *zeroButton_;
  QLabel *statusLabel_;
};

GZ_REGISTER_GUI_PLUGIN(GazeboNavGoalGui)

}  // namespace gazebo

#include "gazebo_nav_goal_gui.moc"
