#include "settingpage.h"
#include <QLabel>
#include <QPushButton>
#include <QFormLayout>
#include <QLineEdit>
#include "interface.h"

SettingPage::SettingPage(QWidget* parent)
    : QWidget{ parent }, m_vlay(new QVBoxLayout(this)),
    m_ipLineEdit(new QLineEdit()), m_portLineEdit(new QLineEdit()), m_localportLineEdit(new QLineEdit())
{
        m_vlay->setAlignment(Qt::AlignCenter);
        m_vlay->setSpacing(20);
        // 创建标题标签
        QLabel *titleLabel = new QLabel("IotSmartVision");
        titleLabel->setObjectName("title");
        titleLabel->setAlignment(Qt::AlignCenter);
        m_vlay->addWidget(titleLabel);

        // 创建圆形头像
        QHBoxLayout *h_lay = new QHBoxLayout();
        h_lay->setContentsMargins(0,50,10,0);
        h_lay->setAlignment(Qt::AlignHCenter);
        CircleLabel *avatarLabel = new CircleLabel(nullptr,":Image/Img/avatar.jpg");
        avatarLabel->setFixedSize(150, 150);  // 设置固定大小，以显示圆形效果
        h_lay->addWidget(avatarLabel);
        m_vlay->addLayout(h_lay);

        QLabel *userLabel = new QLabel("Enjoy It");
        userLabel->setObjectName("user");
        userLabel->setAlignment(Qt::AlignCenter);
        m_vlay->addWidget(userLabel);
        userLabel->setContentsMargins(0,0,10,0);

        // 创建登录表单
        QFormLayout *formLayout = new QFormLayout();
        formLayout->setSpacing(20);
        m_vlay->addLayout(formLayout);
        formLayout->setContentsMargins(320,20,10,50);


        // 创建 bind port 输入框
        m_localportLineEdit->setAlignment(Qt::AlignCenter);
        QLabel* LocalPortLabel = new QLabel("Local Port ");
        LocalPortLabel->setObjectName("login");
        formLayout->addRow(LocalPortLabel, m_localportLineEdit);
        m_localportLineEdit->setFixedSize(300, 60);
        m_localportLineEdit->setText("3000");

        // 创建IP输入框
        m_ipLineEdit->setAlignment(Qt::AlignCenter);
        QLabel *ipLabel = new QLabel("IP ");
        ipLabel->setObjectName("login");
        formLayout->addRow(ipLabel, m_ipLineEdit);
        m_ipLineEdit->setFixedSize(300,60);
        m_ipLineEdit->setText("192.168.200.200");


        // 创建port输入框
        m_portLineEdit->setAlignment(Qt::AlignCenter);
        QLabel *portLabel = new QLabel("Port ");
        portLabel->setObjectName("login");
        formLayout->addRow(portLabel, m_portLineEdit);
        m_portLineEdit->setFixedSize(300,60);
        m_portLineEdit->setText("3000");

        QHBoxLayout* h_butlay = new QHBoxLayout();
        h_butlay->setAlignment(Qt::AlignHCenter);
        h_butlay->setSpacing(30);


        // 创建绑定按钮
        QPushButton* bindButton = new QPushButton("Bind");
        bindButton->setFixedSize(260, 60);
        h_butlay->addWidget(bindButton);
        m_vlay->addLayout(h_butlay);

        // 创建登录按钮
        QPushButton *loginButton = new QPushButton("Login");
        loginButton->setFixedSize(260,60);
        h_butlay->addWidget(loginButton);

        // 连接按钮的点击信号到相应的槽函数
        connect(loginButton, &QPushButton::clicked, this, [this](){
            QString ip = this->m_ipLineEdit->text();
            uint16_t port = this->m_portLineEdit->text().toUShort();
            setSendAddr(ip,port);
        });

        connect(bindButton, &QPushButton::clicked, this, [this]() {
            uint16_t port = this->m_localportLineEdit->text().toUShort();
            DeInitUdpSocket();
            InitUdpSocket(port);
            });

        InitStyle();
}


void SettingPage::InitStyle(){
    this->setStyleSheet("QLineEdit{"
                        "color: #C0C0C0;"
                        "background-color:#405361;"
                        "font-size:25px;"
                        "border-style:outset;"
                        "border-radius:10px;"
                        "font-style:MingLiU-ExtB;"
                        "}"
                        // "QLineEdit::placeholder{ color: red; }"
                        "QLabel#title{"
                        "color:white;"
                        "font-style:MingLiU-ExtB;"
                        "font-size:35px;"
                        "}"
                        "QLabel#user{"
                        "color: #E1FFFF;"
                        "font-style:italic;"
                        "font-size:25px;"
                        "}"
                        "QLabel#login{"
                        "background:rgba(85,170,255,0);"
                        "color:white;"
                        "font-style:MingLiU-ExtB;"
                        "font-size:30px;"
                        "}"
                        "QPushButton{"
                        "background:#ced1d8;"
                        "border-style:outset;"
                        "border-radius:15px;"
                        "font-style:MingLiU-ExtB;"
                        "font-size:22px;"
                        "}"
                        "QPushButton:hover{"
                        "background-color:#F08080;"
                        "border-style:inset;"
                        "font-size:22px;"
                        "font-style:MingLiU-ExtB;"
                        "}"
                        );
}
