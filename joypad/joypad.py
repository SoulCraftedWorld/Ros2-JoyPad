#!/usr/bin/env python3

import sys
import rclpy
from pkg_resources import run_script
from rclpy.node import Node
from geometry_msgs.msg import Twist

# PyQt5 imports. If you have PyQt6, change it to PyQt6
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel, QGridLayout, QMessageBox, QInputDialog
from PyQt5.QtGui import QKeyEvent, QFont, QPalette, QColor
from PyQt5.QtCore import QMetaObject, Qt, QTimer, QThread

from functools import partial

def to_gard_sec(rad_sec):
    return rad_sec * (180 / 3.14159)
def to_rad_sec(grad_sec):
    return grad_sec * (3.14159 / 180)
def to_km_h(speed_ms):
    return speed_ms * 3.6
def to_m_s(speed_km_h):
    return speed_km_h / 3.6

# Speeds
LIN_SPD = to_m_s(0.8) # initial linear speed km/h
ANG_SPD = to_rad_sec(20) # initial angular speed grad/sec
SPEED_STEP = to_m_s(0.1)  # Speed step km/h
ANG_STEP = to_rad_sec(2.5)  # Angular speed step grad/sec

# Constants
CMD_VEL_UPDATE_INTERVAL = 500  # ms
CMD_VEL_PUBLIC_INTERVAL = 50  # ms
CMD_VEL_CONNECTION_TIMEOUT = 2000  # ms
RELEASE_DELAY = 70  # ms
SPEED_HOLD_INTERVAL = 200  # ms
SPEED_INITIAL_DELAY = 300

DEFAULT_TOPIC_NAME = '/cmd_vel'

rus_key_w = int(1062) #ord('ц')
rus_key_a = int(1060)  #ord('ф')
rus_key_s = int(1067)   # ord('ы')
rus_key_d = int(1042)  #ord('в')
rus_key_k = int(1051)  #ord('л')
rus_key_x = int(1063)  #ord('х')

rus_key_w2 = ord('ц')
rus_key_a2 = ord('ф')
rus_key_s2 = ord('ы')
rus_key_d2 = ord('в')
rus_key_k2 = ord('л')

#print(f"RUS keys: {rus_key_w}, {rus_key_a}, {rus_key_s}, {rus_key_d}, {rus_key_k}")
# 1081, 1092, 1099, 1074, 1083

directions_by_lin_ang_speed = {
    (1, 0): '⬆️',
    (-1, 0): '⬇️',
    (0, 1): '⬅️',
    (0, -1): '➡️',
    (1, 1): '↖️',
    (1, -1): '↗️',
    (-1, 1): '↙️',
    (-1, -1): '↘️',
    (0, 0): '🛑'
}

# Dictionary for key mapping
key_mapping = {
    frozenset([Qt.Key_W]): (1, 0, '⬆️'),
    frozenset([Qt.Key_Up]): (1, 0, '⬆️'),
    frozenset([rus_key_w]): (1, 0, '⬆️'),
    frozenset([rus_key_w2]): (1, 0, '⬆️'),

    frozenset([Qt.Key_W, Qt.Key_D]): (1, -1, '↗️'),
    frozenset([Qt.Key_Up, Qt.Key_Right]): (1, -1, '↗️'),
    frozenset([rus_key_w, rus_key_d]): (1, -1, '↗️'),
    frozenset([rus_key_w2, rus_key_d2]): (1, -1, '↗️'),

    frozenset([Qt.Key_S]): (-1, 0, '⬇️'),
    frozenset([Qt.Key_Down]): (-1, 0, '⬇️'),
    frozenset([rus_key_s]): (-1, 0, '⬇️'),
    frozenset([rus_key_s2]): (-1, 0, '⬇️'),

    frozenset([Qt.Key_S, Qt.Key_D]): (-1, -1, '↘️'),
    frozenset([Qt.Key_Down, Qt.Key_Right]): (-1, -1, '↘️'),
    frozenset([rus_key_s, rus_key_d]): (-1, -1, '↘️'),
    frozenset([rus_key_s2, rus_key_d2]): (-1, -1, '↘️'),

    frozenset([Qt.Key_A]): (0, 1, '⬅️'),
    frozenset([Qt.Key_Left]): (0, 1, '⬅️'),
    frozenset([rus_key_a]): (0, 1, '⬅️'),
    frozenset([rus_key_a2]): (0, 1, '⬅️'),

    frozenset([Qt.Key_W, Qt.Key_A]): (1, 1, '↖️'),
    frozenset([Qt.Key_Up, Qt.Key_Left]): (1, 1, '↖️'),
    frozenset([rus_key_w, rus_key_a]): (1, 1, '↖️'),
    frozenset([rus_key_w2, rus_key_a2]): (1, 1, '↖️'),

    frozenset([Qt.Key_D]): (0, -1, '➡️'),
    frozenset([Qt.Key_Right]): (0, -1, '➡️'),
    frozenset([rus_key_d]): (0, -1, '➡️'),
    frozenset([rus_key_d2]): (0, -1, '➡️'),

    frozenset([Qt.Key_S, Qt.Key_A]): (-1, 1, '↙️'),
    frozenset([Qt.Key_Down, Qt.Key_Left]): (-1, 1, '↙️'),
    frozenset([rus_key_s, rus_key_a]): (-1, 1, '↙️'),
    frozenset([rus_key_s2, rus_key_a2]): (-1, 1, '↙️'),

    frozenset([Qt.Key_X]): (0, 0, '🛑'),
    frozenset([Qt.Key_K]): (0, 0, '🛑'),
    frozenset([Qt.Key_Return]): (0, 0, '🛑'),
    frozenset([Qt.Key_Backspace]): (0, 0, '🛑'),
    frozenset([rus_key_x]): (0, 0, '🛑'),
    frozenset([rus_key_k]): (0, 0, '🛑'),

}

lang_dict = {
    'ru': {
        'lang_sign': '🇷🇺',
        'select_lang': 'Выбрать ',
        'app_name': 'ROS2 JoyPad',
        'source': 'Источник',
        'lin_speed': 'Линейная',
        'ang_speed': 'Угловая',
        'speed_val': '{lin_speed:.2f} км/ч',
        'ang_speed_val': '{ang_speed:.2f} °/с',
        'current_speed': '🚗 Текущая-',
        'direction': 'Направление',
        'stop': '❌ Стоп',
        'connect': '📢 Топик {state} ',
        'current_speed_val': '{current_lin_x:.2f} км/ч',
        'current_ang_speed_val': '{current_ang_z:.2f} °/с',
        'lin_speed_up': 'Лин. ➕',
        'lin_speed_down': 'Лин. ➖',
        'ang_speed_up': 'Угл. ➕',
        'ang_speed_down': 'Угл. ➖',
        'about': 'О приложении',
        'to_default': ' Сбросить ',
        'keep_publishing': '🔄 Авто-вещание',
        'flex_lin_speed': '🎚️ Мягкое ускор.',
        'about_text': "🔹 ROS2 JoyPad 🔹\n"
                "Удобное управление роботом через ROS2.\n\n"
                "📡 Возможности:\n\n"
                "🚀 Движение и повороты робота\n"
                "⌨️ Поддержка клавиатуры и кнопок\n"
                "⏳ Активно только при удержании клавиши\n"
                "📊 Мониторинг скорости\n"
                "🌍 Многоязычный интерфейс\n"
                "📢 Контроль соединения и показаний топика\n"
                "🔄 Авто-вещание - постоянное вещанеи на топик\n"      
                "🎚️ Мягкое ускор.– плавное изменение скорости при управлении.\n\n"
                "© 2025 ROS2 JoyPad JCreator\n"
    },
    'en': {
        'lang_sign': '🇬🇧',
        'select_lang': 'Select ',
        'app_name': 'ROS2 JoyPad',
        'source': 'Source',
        'lin_speed': 'Linear',
        'ang_speed': 'Angular',
        'speed_val': '{lin_speed:.2f} km/h',
        'ang_speed_val': '{ang_speed:.2f} °/s',
        'current_speed': '🚗 Current-',
        'direction': 'Direction',
        'stop': '❌ Stop',
        'connect': '📢 Topic {state} ',
        'current_speed_val': '{current_lin_x:.2f} km/h',
        'current_ang_speed_val': '{current_ang_z:.2f} °/s',
        'lin_speed_up': 'Lin. ➕',
        'lin_speed_down': 'Lin. ➖',
        'ang_speed_up': 'Ang. ➕',
        'ang_speed_down': 'Ang. ➖',
        'about': 'About',
        'to_default': ' Reset ',
        'keep_publishing': '🔄 Auto Stream',
        'flex_lin_speed': '🎚️ Flex Accel',
        'about_text': "🔹 ROS2 JoyPad 🔹\n"
                "Convenient robot control via ROS2.\n\n"
                "📡 Features:\n\n"
                "🚀 Robot movement and turns\n"
                "⌨️ Keyboard and button support\n"
                "⏳ Active only when holding a key\n"
                "📊 Speed monitoring\n"
                "🌍 Multilingual interface\n"
                "📢 Connection control and values of topic\n"
                "🔄 Auto Stream - repeate streaming on topic\n"
                "🎚️ Flex Accel – smooth acceleration control\n\n"
                "© 2025 ROS2 JoyPad JCreator\n"
    }
}

button_style = """
            QPushButton {
                background-color: #3498db; 
                color: white;
                border-radius: 5px;
                padding: 5px;
            }
            QPushButton:hover {
                background-color: #2980b9; 
            }
            QPushButton:pressed {
                background-color: #1c5a85; 
            }
        """

# Class for ROS2 thread
class RosThread(QThread):
    """Thread for ROS2"""

    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)


class JoyPadGui(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'joypad')
        QWidget.__init__(self)

        self.inc_dec_buttons_timers = {}
        self.lang_available = lang_dict.keys()
        # Check if you have this font or use 'sudo apt install fonts-noto-color-emoji'
        self.emoji_font = QFont("Noto Color Emoji", 16)
        self.emoji_font_small = QFont("Noto Color Emoji", 12)
        self.regular_font = QFont("Arial", 16)
        self.emoji_font_big = QFont("Noto Color Emoji", 39)
        self.emoji_font_middle = QFont("Noto Color Emoji", 23)
        self.default_font_color = "color: black;"

        if len(self.lang_available) == 0:
            self.cur_lang = 'en'
        else:
            self.cur_lang = list(self.lang_available)[0]

        self.layout = QVBoxLayout()

        self.setFocusPolicy(Qt.StrongFocus)
        self.activateWindow()
        self.raise_()

        # ROS2 Publisher & Subscriber
        self.pub_cmd_vel_topic = DEFAULT_TOPIC_NAME
        self.sub_cmd_vel_topic = DEFAULT_TOPIC_NAME
        self.pub_cmd_vel = None
        self.sub_cmd_vel = None

        # Speeds
        self.lin_speed = LIN_SPD
        self.ang_speed = ANG_SPD

        self.joypad_lin_speed = 0.0
        self.joypad_ang_speed = 0.0

        self.pressed_keys = set()  # Список нажатых клавиш
        self.release_timers = {}  # Таймеры для обработки задержки отпускания клавиш
        self.movement_active = False  # Флаг активности движения
        self.current_lin_x = 0.0
        self.current_ang_z = 0.0
        self.subscr_lin_x = 0.0
        self.subscr_ang_z = 0.0
        self.curr_lin_x_next_change_time = 0.0
        self.curr_ang_z_next_change_time = 0.0
        self.is_cmd_vel_connected = False

        self.current_lin_next_change_time = 0
        self.current_ang_next_change_time = 0
        self.last_cmd_vel_update_time = 0

        # Таймер для обновления сообщений (каждые 50 мс)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_twist)
        # self.timer.start(50)
        self.keep_publishing = False

        # Таймер проверки соединения с `/cmd_vel`
        self.cmd_vel_timeout_timer = QTimer()
        self.cmd_vel_timeout_timer.setSingleShot(True)
        self.cmd_vel_timeout_timer.timeout.connect(self.set_lost_connection)

        self.create_publisher_for_cmd(self.pub_cmd_vel_topic)
        self.subscribe_to_topic(self.sub_cmd_vel_topic)

        self.key_buttons_match = {}

        self.flex_lin_speed = False
        self.lin_speed_accum = 0.0 
        self.last_public_time_sec = 0

        # Интерфейс
        self.initUI()
        self.setFocusPolicy(Qt.StrongFocus)  # Позволяет окну принимать события клавиатуры
    
    def subscribe_to_topic(self, topic_name):
        """Subscribe to topic"""
        if self.cmd_vel_timeout_timer.isActive():
            self.cmd_vel_timeout_timer.stop()

        if self.sub_cmd_vel is not None:
            self.sub_cmd_vel.destroy()

        self.sub_cmd_vel = self.create_subscription(Twist, topic_name, self.cmd_vel_callback, 10)
        self.cmd_vel_timeout_timer.start(CMD_VEL_CONNECTION_TIMEOUT)

    def create_publisher_for_cmd(self, topic):
        """Create publisher"""
        if self.pub_cmd_vel is not None:
            self.pub_cmd_vel.destroy()

        self.pub_cmd_vel =  self.create_publisher(Twist, topic, 10)


    def show_about_dialog(self):
        """Show about dialog"""
        info_text = lang_dict[self.cur_lang]['about_text']
        message_box = QMessageBox()
        message_box.setWindowTitle(lang_dict[self.cur_lang]['about'])
        message_box.setStyleSheet("QLabel{ font-size: 16px; font-family: 'Arial, Noto Color Emoji'; }")
        # message_box.information(self, lang_dict[self.cur_lang]['about'], info_text)
        message_box.setText(info_text)
        message_box.exec_()

    # Change language
    def change_lang(self):
        # get next language
        next_lang_num = 0
        for i, lang in enumerate(self.lang_available):
            if lang == self.cur_lang:
                next_lang_num = i + 1
                break

        if next_lang_num >= len(self.lang_available):
            next_lang_num = 0

        self.cur_lang = list(self.lang_available)[next_lang_num]
        self.initUI()

    def clearLayout(self, layout):
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()  # Удаляем виджет
            else:
                self.clearLayout(item.layout())  # Рекурсивно удаляем вложенные layout'ы

    def edit_topic(self):
        """Edit topic"""
        text, ok = QInputDialog.getText(self, 'Edit topic', 'Enter topic name:', text=self.sub_cmd_vel_topic)
        if ok and text != '' and text != self.sub_cmd_vel_topic:
            # remove all spaces
            text = text.replace(" ", "")
            if text != '':
                self.sub_cmd_vel_topic = text
                self.pub_cmd_vel_topic = text
                self.create_publisher_for_cmd(self.pub_cmd_vel_topic)
                self.subscribe_to_topic(self.sub_cmd_vel_topic)
                self.initUI()
                return
        if self.sub_cmd_vel_topic =='':
            self.sub_cmd_vel_topic = DEFAULT_TOPIC_NAME
            self.pub_cmd_vel_topic = DEFAULT_TOPIC_NAME
            self.create_publisher_for_cmd(self.pub_cmd_vel_topic)
            self.subscribe_to_topic(self.sub_cmd_vel_topic)
            self.initUI()

    def initUI(self):

        if len(self.inc_dec_buttons_timers) > 0:
            for timer in self.inc_dec_buttons_timers.values():
                timer.stop()
                timer.deleteLater()
            self.inc_dec_buttons_timers.clear()

        # CLear layout
        self.clearLayout(self.layout)
        # Update GUI
        self.repaint()

        """Creating GUI"""
        self.setWindowTitle(lang_dict[self.cur_lang]['app_name'])
        self.setGeometry(300, 300, 350, 450)
        self.default_font_color = "color: black;"

        header_grid = QGridLayout()


        self.btn_about = QPushButton(lang_dict[self.cur_lang]['about'])
        self.btn_about.setFont(self.emoji_font_small)
        self.btn_about.clicked.connect(self.show_about_dialog)
        header_grid.addWidget(self.btn_about, 0, 0)

        self.default_font_color = self.btn_about.palette().color(QPalette.Text).name()

        header_grid.addWidget(QLabel(" "), 0, 1)

        lang_signs = []
        for lang in lang_dict.keys():
            if lang != self.cur_lang:
                lang_signs.append(lang_dict[lang]['lang_sign'])

        # right top corner
        langs_str = lang_dict[self.cur_lang]['select_lang'] + " / " + " ".join(lang_signs)

        self.btn_change_lang = QPushButton(langs_str)
        self.btn_change_lang.setFont(self.emoji_font_small)
        self.btn_change_lang.clicked.connect(lambda: self.change_lang())
        # self.layout.addWidget(self.btn_change_lang, 0, Qt.AlignRight)
        header_grid.addWidget(self.btn_change_lang, 0, 2)

        self.layout.addLayout(header_grid)

        #  self.layout.addWidget(QLabel(" "), 1, Qt.AlignCenter)
        top_grid = QGridLayout()
        top_grid.setRowMinimumHeight(2, 50)  # Set row height (row count, height)
        top_grid.setRowStretch(2, 0) # Set row stretch (row count, stretch)

        # layer 0
        self.label_speeds_legend = QLabel(lang_dict[self.cur_lang]['source'])
        self.label_speeds_legend.setFont(self.emoji_font)
        top_grid.addWidget(self.label_speeds_legend, 0, 0)

        self.label_speeds_legend = QLabel(lang_dict[self.cur_lang]['lin_speed'])
        self.label_speeds_legend.setFont(self.emoji_font)
        top_grid.addWidget(self.label_speeds_legend, 0, 1)

        self.label_ang_speeds_legend = QLabel(lang_dict[self.cur_lang]['ang_speed'])
        self.label_ang_speeds_legend.setFont(self.emoji_font)
        top_grid.addWidget(self.label_ang_speeds_legend, 0, 2)

        self.label_direction_name = QLabel(lang_dict[self.cur_lang]['direction'])
        self.label_direction_name.setFont(self.emoji_font)
        top_grid.addWidget(self.label_direction_name, 0, 3)

        # layer 1
        self.btn_cur_topic = QPushButton(self.sub_cmd_vel_topic)
        self.btn_cur_topic.setFont(self.emoji_font)
        self.btn_cur_topic.clicked.connect(lambda: self.edit_topic())
        top_grid.addWidget(self.btn_cur_topic, 1, 0)

        self.label_current_speeds = QLabel(
            lang_dict[self.cur_lang]['current_speed_val'].format(current_lin_x=to_km_h(self.current_lin_x)))
        self.label_current_speeds.setFont(self.regular_font)
        top_grid.addWidget(self.label_current_speeds, 1, 1, alignment=Qt.AlignCenter)

        self.label_current_ang_speeds = QLabel(
            lang_dict[self.cur_lang]['current_ang_speed_val'].format(current_ang_z=to_gard_sec(self.current_ang_z)))
        self.label_current_ang_speeds.setFont(self.regular_font)
        top_grid.addWidget(self.label_current_ang_speeds, 1, 2, alignment=Qt.AlignCenter)

        self.label_direction_on_topic = QLabel('🛑')
        self.label_direction_on_topic.setFont(self.emoji_font_middle)
        self.label_direction_on_topic.setFixedHeight(60)
        top_grid.addWidget(self.label_direction_on_topic, 1, 3, alignment=Qt.AlignCenter)

        # layer 2
        self.label_cur_speeds_legend = QLabel('JoyPad:')  # lang_dict[self.cur_lang]['current_speed'])
        self.label_cur_speeds_legend.setFont(self.emoji_font)
        top_grid.addWidget(self.label_cur_speeds_legend, 2, 0)

        self.label_joypad_lin_speeds = QLabel(
            lang_dict[self.cur_lang]['speed_val'].format(lin_speed=to_km_h(self.joypad_lin_speed)))
        self.label_joypad_lin_speeds.setFont(self.regular_font)
        top_grid.addWidget(self.label_joypad_lin_speeds, 2, 1, alignment=Qt.AlignCenter)

        self.label_joypad_ang_speeds = QLabel(
            lang_dict[self.cur_lang]['ang_speed_val'].format(ang_speed=to_gard_sec(self.joypad_lin_speed)))
        self.label_joypad_ang_speeds.setFont(self.regular_font)
        top_grid.addWidget(self.label_joypad_ang_speeds, 2, 2, alignment=Qt.AlignCenter)

        self.label_joypad_direction = QLabel('🛑')
        self.label_joypad_direction.setFont(self.emoji_font_middle)
        top_grid.addWidget(self.label_joypad_direction, 2, 3, alignment=Qt.AlignCenter)


        self.layout.addLayout(top_grid)



        # Speed control menu
        speed_grid = QGridLayout()

        self.btn_lin_speed_up = QPushButton(lang_dict[self.cur_lang]['lin_speed_up'])
        self.btn_lin_speed_up.setFont(self.emoji_font)
        self.btn_lin_speed_down = QPushButton(lang_dict[self.cur_lang]['lin_speed_down'])
        self.btn_lin_speed_down.setFont(self.emoji_font)
        self.btn_ang_speed_up = QPushButton(lang_dict[self.cur_lang]['ang_speed_up'])
        self.btn_ang_speed_up.setFont(self.emoji_font)
        self.btn_ang_speed_down = QPushButton(lang_dict[self.cur_lang]['ang_speed_down'])
        self.btn_ang_speed_down.setFont(self.emoji_font)

        self.btn_lin_speed_default = QPushButton(lang_dict[self.cur_lang]['to_default'])
        self.btn_lin_speed_default.setFont(self.emoji_font)
        self.btn_ang_speed_default = QPushButton(lang_dict[self.cur_lang]['to_default'])
        self.btn_ang_speed_default.setFont(self.emoji_font)

        self.label_preset_lin_speeds = QLabel(
            lang_dict[self.cur_lang]['speed_val'].format(lin_speed=to_km_h(self.lin_speed)))
        self.label_preset_lin_speeds.setFont(self.regular_font)

        self.label_preset_ang_speeds = QLabel(
            lang_dict[self.cur_lang]['ang_speed_val'].format(ang_speed=to_gard_sec(self.ang_speed)))
        self.label_preset_ang_speeds.setFont(self.regular_font)

        speed_grid.addWidget(self.btn_lin_speed_default, 0, 0)
        speed_grid.addWidget(self.btn_lin_speed_down, 0, 1)
        speed_grid.addWidget(self.label_preset_lin_speeds, 0, 2, alignment=Qt.AlignCenter)
        speed_grid.addWidget(self.btn_lin_speed_up, 0, 3)

        speed_grid.addWidget(self.btn_ang_speed_default, 1, 0)
        speed_grid.addWidget(self.btn_ang_speed_down, 1, 1)
        speed_grid.addWidget(self.label_preset_ang_speeds, 1, 2, alignment=Qt.AlignCenter)
        speed_grid.addWidget(self.btn_ang_speed_up, 1, 3)

        self.btn_lin_speed_up.pressed.connect(lambda: self.start_inc_dec_buttons_timer(self.increase_lin_speed))
        self.btn_lin_speed_down.pressed.connect(lambda: self.start_inc_dec_buttons_timer(self.decrease_lin_speed))
        self.btn_ang_speed_up.pressed.connect(lambda: self.start_inc_dec_buttons_timer(self.increase_ang_speed))
        self.btn_ang_speed_down.pressed.connect(lambda: self.start_inc_dec_buttons_timer(self.decrease_ang_speed))
        self.btn_ang_speed_default.clicked.connect(lambda: self.set_ang_speed_default())
        self.btn_lin_speed_default.clicked.connect(lambda: self.set_speed_default())

        self.btn_lin_speed_up.released.connect(lambda: self.stop_inc_dec_buttons_timer(self.increase_lin_speed))
        self.btn_lin_speed_down.released.connect(lambda: self.stop_inc_dec_buttons_timer(self.decrease_lin_speed))
        self.btn_ang_speed_up.released.connect(lambda: self.stop_inc_dec_buttons_timer(self.increase_ang_speed))
        self.btn_ang_speed_down.released.connect(lambda: self.stop_inc_dec_buttons_timer(self.decrease_ang_speed))

        self.layout.addLayout(speed_grid)


        button_w = 140
        button_h = 75
        grid_widget = QWidget()
        grid_widget.setFixedSize(int(button_w * 3.5), int(button_h * 3.5))

        grid = QGridLayout()
        grid.setContentsMargins(10, 10, 10, 10)  # Padding

        # Direction buttons
        self.btn_up = QPushButton("W⬆️")
        self.btn_up.setFont(self.emoji_font_big)
        self.btn_up.setFixedSize(button_w, button_h)
        self.btn_down = QPushButton("S⬇️")
        self.btn_down.setFont(self.emoji_font_big)
        self.btn_down.setFixedSize(button_w, button_h)
        self.btn_left = QPushButton("A⬅️")
        self.btn_left.setFont(self.emoji_font_big)
        self.btn_left.setFixedSize(button_w, button_h)
        self.btn_right = QPushButton("➡️D")
        self.btn_right.setFont(self.emoji_font_big)
        self.btn_right.setFixedSize(button_w, button_h)
        self.btn_stop = QPushButton("X🔲")
        self.btn_stop.setFont(self.emoji_font_big)
        self.btn_stop.setFixedSize(button_w, button_h)

        grid.addWidget(self.btn_up, 0, 1)
        grid.addWidget(self.btn_left, 1, 0)
        grid.addWidget(self.btn_stop, 1, 1)
        grid.addWidget(self.btn_right, 1, 2)
        grid.addWidget(self.btn_down, 2, 1)

        self.btn_up.setStyleSheet(button_style)
        self.btn_down.setStyleSheet(button_style)
        self.btn_left.setStyleSheet(button_style)
        self.btn_right.setStyleSheet(button_style)

        # key buttons handling
        self.key_buttons_match[Qt.Key_W] = self.btn_up
        self.key_buttons_match[Qt.Key_S] = self.btn_down
        self.key_buttons_match[Qt.Key_A] = self.btn_left
        self.key_buttons_match[Qt.Key_D] = self.btn_right
        self.key_buttons_match[Qt.Key_X] = self.btn_stop
        self.key_buttons_match[Qt.Key_K] = self.btn_stop
        self.key_buttons_match[Qt.Key_Return] = self.btn_stop

        # rus keys handling
        self.key_buttons_match[rus_key_w] = self.btn_up
        self.key_buttons_match[rus_key_s] = self.btn_down
        self.key_buttons_match[rus_key_a] = self.btn_left
        self.key_buttons_match[rus_key_d] = self.btn_right
        self.key_buttons_match[rus_key_x] = self.btn_stop

        self.btn_up.pressed.connect(lambda: self.handle_key_press(Qt.Key_W))
        self.btn_down.pressed.connect(lambda: self.handle_key_press(Qt.Key_S))
        self.btn_left.pressed.connect(lambda: self.handle_key_press(Qt.Key_A))
        self.btn_right.pressed.connect(lambda: self.handle_key_press(Qt.Key_D))
        self.btn_stop.pressed.connect(lambda: self.stop_motion(Qt.Key_X))

        self.btn_up.released.connect(lambda: self.handle_key_release(Qt.Key_W))
        self.btn_down.released.connect(lambda: self.handle_key_release(Qt.Key_S))
        self.btn_left.released.connect(lambda: self.handle_key_release(Qt.Key_A))
        self.btn_right.released.connect(lambda: self.handle_key_release(Qt.Key_D))
        self.btn_stop.released.connect(lambda: self.handle_key_release(Qt.Key_X))

        self.layout.addLayout(grid)

        self.layout.addWidget(QLabel(" "), 1, Qt.AlignCenter)

        bottom_grid = QGridLayout()
        self.label_connect_cmd_vel_topic = QLabel(
                lang_dict[self.cur_lang]['connect'].format(state='🔗'))
        self.label_connect_cmd_vel_topic.setFont(self.emoji_font_small)
        bottom_grid.addWidget(self.label_connect_cmd_vel_topic, 0, 0)

        self.btn_keep_publishing = QPushButton(lang_dict[self.cur_lang]['keep_publishing'])
        self.btn_keep_publishing.setFont(self.emoji_font_small)
        self.btn_keep_publishing.setCheckable(True)
        self.change_button_color(self.btn_keep_publishing, "grey")
        self.btn_keep_publishing.toggled.connect(
            lambda checked: self.change_button_color(self.btn_keep_publishing, "blue" if checked else "grey"))

        self.btn_keep_publishing.clicked.connect(lambda: self.handle_keep_publishing_pressed())
        bottom_grid.addWidget(self.btn_keep_publishing, 0, 1)

        self.btn_flex_lin_speed = QPushButton(lang_dict[self.cur_lang]['flex_lin_speed'])
        self.btn_flex_lin_speed.setFont(self.emoji_font_small)
        self.btn_flex_lin_speed.setCheckable(True)
        self.change_button_color(self.btn_flex_lin_speed, "grey")
        self.btn_flex_lin_speed.toggled.connect(
            lambda checked: self.change_button_color(self.btn_flex_lin_speed, "blue" if checked else "grey"))
        self.btn_flex_lin_speed.clicked.connect(lambda: self.handle_btn_flex_lin_speed_pressed())
        bottom_grid.addWidget(self.btn_flex_lin_speed, 0, 2)

        self.layout.addLayout(bottom_grid)

        self.setLayout(self.layout)


        self.update_connection()
        self.handle_keep_publishing()

    def change_button_color(self, button, color):
        button.setStyleSheet(f"background-color: {color};")

    def set_color(self, q_object, color):
        palette = q_object.palette()
        palette.setColor(QPalette.Button, QColor(color))  # Set color
        q_object.setPalette(palette)
        q_object.update()

    def handle_btn_flex_lin_speed_pressed(self):
        self.flex_lin_speed = not self.flex_lin_speed
        self.lin_speed_accum = 0.0

        # switch keep_publishing on if flex_lin_speed is on
        if self.flex_lin_speed and self.keep_publishing != True:
            self.btn_keep_publishing.click()


    def handle_keep_publishing_pressed(self):
        self.keep_publishing = not self.keep_publishing
        self.handle_keep_publishing()

    def handle_keep_publishing(self):
        self.start_publishing(self.keep_publishing)
        #self.btn_keep_publishing.setText(lang_dict[self.cur_lang]['keep_publishing'] + ' ' + ('🔵' if self.keep_publishing else '🔴'))

    def keyPressEvent(self, event):
        """Handle key press"""
        key = frozenset({event.key()})  # Convert key to frozenset
        if key in key_mapping:
            self.handle_key_press(event.key())


    def keyReleaseEvent(self, event):
        """Handle key release"""
        key = event.key()
        if key in self.pressed_keys:
            self.handle_key_release(key)

    def handle_key_press(self, key):
        """Add key to pressed keys in set if it is allowed"""

        if key in self.release_timers:
            if self.release_timers[key] is not None:
                self.release_timers[key].stop()  # Stop release timer
                self.release_timers[key].deleteLater()
            del self.release_timers[key]  # Delete release timer

        # не более двух клавиш одновременно
        temp_keys = self.pressed_keys | {key}  # Add new key to pressed keys

        if any(frozenset(temp_keys) == k for k in key_mapping):  # Check if new set is in key_mapping
            self.pressed_keys.add(key)
            if key in self.key_buttons_match:
                self.key_buttons_match[key].setDown(True)
            self.movement_active = True  # Set movement flag
            self.update_twist()

    def handle_key_release(self, key):
        """Delay key release"""
        if key in self.pressed_keys:
            timer = QTimer()
            timer.setSingleShot(True)  # One shot timer
            timer.timeout.connect(lambda: self.finalize_key_release(key))  # Finalize key release
            timer.start(RELEASE_DELAY)
            self.release_timers[key] = timer


    def finalize_key_release(self, key):
        """Finalize key release"""
        if key in self.pressed_keys:
            self.pressed_keys.remove(key)
            if key in self.key_buttons_match:
                self.key_buttons_match[key].setDown(False)

        if len(self.pressed_keys) == 0:
            self.movement_active = False

        self.update_twist()

    def handle_lin_speed_on_release(self, release_step):
        if self.lin_speed_accum != 0.0:
            if self.lin_speed_accum > 0.0:
                self.lin_speed_accum -= release_step
                if self.lin_speed_accum < 0.0:
                    self.lin_speed_accum = 0.0
            else:
                self.lin_speed_accum += release_step
                if self.lin_speed_accum > 0.0:
                    self.lin_speed_accum = 0.0

    def update_twist(self):
        """Update twist message"""
        lin_x, ang_z = 0.0, 0.0
        directions = []

        curr_time_sec = self.get_clock().now().nanoseconds / 1e9
        d_time_sec = curr_time_sec - self.last_public_time_sec
        self.last_public_time_sec = curr_time_sec
        if d_time_sec > 0.25:
            d_time_sec = 0.25

        if self.movement_active:
            pressed_set = frozenset(self.pressed_keys)
            if pressed_set in key_mapping:
                lx, az, direction = key_mapping[pressed_set]

                # mode for flexible linear speed - increase if lin key pressed or decrease to 0 if lin key released
                if self.flex_lin_speed:
                    lin_speed_step = self.lin_speed * d_time_sec
                    if lx == 0.0: # if lin key released
                        if az == 0.0:
                            self.lin_speed_accum = 0.0
                        else:
                            self.handle_lin_speed_on_release(lin_speed_step/ 3.0)

                    else:
                        lin_x_max = self.lin_speed * lx
                        if lx > 0.0: # Forward
                            # check for reverse
                            if self.lin_speed_accum < 0.0:
                                self.lin_speed_accum += lin_speed_step * 2.0
                            else:
                                self.lin_speed_accum += lin_speed_step
                                if self.lin_speed_accum > lin_x_max:
                                    self.lin_speed_accum = lin_x_max
                        else: # Backward
                            # check for forward
                            if self.lin_speed_accum > 0.0:
                                self.lin_speed_accum -= lin_speed_step * 2.0
                            else:
                                self.lin_speed_accum -= lin_speed_step
                                if self.lin_speed_accum < lin_x_max:
                                    self.lin_speed_accum = lin_x_max

                    # set lin_x to accumulated value
                    lin_x = self.lin_speed_accum

                else:
                    lin_x = lx * self.lin_speed

                ang_z = az * self.ang_speed
                directions.append(direction)
        else:
            if self.flex_lin_speed:
                self.handle_lin_speed_on_release(self.lin_speed * d_time_sec/ 3.0)
                lin_x = self.lin_speed_accum

        self.joypad_lin_speed = lin_x
        self.joypad_ang_speed = ang_z

        twist = Twist()
        twist.linear.x = lin_x
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = ang_z
        self.pub_cmd_vel.publish(twist)
  
        # Update GUI in main thread only!
        QTimer.singleShot(0, lambda: self.update_key_gui(directions))
 
        self.start_publishing(self.movement_active)
        

    def start_publishing(self, is_active):
        if is_active:
            """Start publishing"""
            if not self.timer.isActive():
                self.timer.start(CMD_VEL_PUBLIC_INTERVAL)
        else:
            if self.keep_publishing == False:
                """Stop publishing"""
                if self.timer.isActive():
                    self.timer.stop()
            

    def update_key_gui(self, directions):
        """Update GUI with directions"""

        if directions:
            direction_str = " ".join(directions)
        else:
            direction_str = '🛑'

        """Update GUI"""
        self.label_joypad_direction.setText(direction_str)

        self.label_joypad_lin_speeds.setText(
            lang_dict[self.cur_lang]['speed_val'].format(lin_speed=to_km_h(self.joypad_lin_speed)))

        self.label_joypad_ang_speeds.setText(
            lang_dict[self.cur_lang]['ang_speed_val'].format(ang_speed=to_gard_sec(self.joypad_ang_speed)))


    def stop_motion(self, key):
        """Stop motion"""
        self.pressed_keys.clear()
        self.movement_active = False
        self.lin_speed_accum = 0.0
        self.update_twist()

        if key in self.key_buttons_match:
            self.key_buttons_match[key].setDown(True)

    def set_speed_default(self):
        """Set default speed"""
        self.lin_speed = LIN_SPD
        self.label_preset_lin_speeds.setText(lang_dict[self.cur_lang]['speed_val'].format(lin_speed=to_km_h(self.lin_speed)))

    def set_ang_speed_default(self):
        """Set default angular speed"""
        self.ang_speed = ANG_SPD
        self.label_preset_ang_speeds.setText(lang_dict[self.cur_lang]['ang_speed_val'].format(ang_speed=to_gard_sec(self.ang_speed)))

    def start_inc_dec_buttons_timer(self, repeat_function):
        """Start timer for increment or decrement"""
        if repeat_function in self.inc_dec_buttons_timers:
            return

        # Run function immediately
        repeat_function()

        # Create timer
        timer = QTimer()
        timer.timeout.connect(lambda: self.start_continuous_repeat(repeat_function, timer))
        timer.setSingleShot(False) # Repeat timer
        timer.setInterval(SPEED_HOLD_INTERVAL)  # Set interval
        timer.start(SPEED_INITIAL_DELAY)  # Start timer with delay SPEED_HOLD_INTERVAL
        # Save timer
        self.inc_dec_buttons_timers[repeat_function] = timer

    def start_continuous_repeat(self, repeat_function, timer):
        """Start continuous repeat"""
        repeat_function()
        timer.timeout.connect(repeat_function)  # Connect function to timer
        timer.setSingleShot(False)
        timer.start(SPEED_HOLD_INTERVAL)  # Start timer with interval SPEED_HOLD_INTERVAL


    def stop_inc_dec_buttons_timer(self, repeat_function):
        """Stop timer for increment or decrement"""
        if repeat_function in self.inc_dec_buttons_timers:
            if self.inc_dec_buttons_timers[repeat_function] is not None:
                self.inc_dec_buttons_timers[repeat_function].stop()
                self.inc_dec_buttons_timers[repeat_function].deleteLater()
            del self.inc_dec_buttons_timers[repeat_function]



    def increase_lin_speed(self):
        """Increase linear speed"""
        self.lin_speed += SPEED_STEP
        self.label_preset_lin_speeds.setText(lang_dict[self.cur_lang]['speed_val'].format(lin_speed=to_km_h(self.lin_speed)))

    def decrease_lin_speed(self):
        """Decrease linear speed"""
        self.lin_speed = max(0.0, self.lin_speed - SPEED_STEP)
        self.label_preset_lin_speeds.setText(lang_dict[self.cur_lang]['speed_val'].format(lin_speed=to_km_h(self.lin_speed)))

    def increase_ang_speed(self):
        """Increase angular speed"""
        self.ang_speed += ANG_STEP
        self.label_preset_ang_speeds.setText(lang_dict[self.cur_lang]['ang_speed_val'].format(ang_speed=to_gard_sec(self.ang_speed)))

    def decrease_ang_speed(self):
        """Decrease angular speed"""
        self.ang_speed = max(0.0, self.ang_speed - ANG_STEP)
        self.label_preset_ang_speeds.setText(lang_dict[self.cur_lang]['ang_speed_val'].format(ang_speed=to_gard_sec(self.ang_speed)))

    def update_connection(self):
        """Update connection status with /cmd_vel topic"""
        if self.is_cmd_vel_connected:
            self.label_connect_cmd_vel_topic.setText(lang_dict[self.cur_lang]['connect'].format(state='🔗'))
            self.label_connect_cmd_vel_topic.setStyleSheet("color: green;")
        else:
            self.label_connect_cmd_vel_topic.setText(lang_dict[self.cur_lang]['connect'].format(state='🚫'))
            self.label_connect_cmd_vel_topic.setStyleSheet("color: red;")
            self.update_gui() # update last value



    def set_lost_connection(self):
        """Set lost connection"""
        self.is_cmd_vel_connected = False
        # Update GUI in main thread only!
        QTimer.singleShot(0, lambda: self.update_connection())


    def cmd_vel_callback(self, msg):
        """Update current speeds"""
        # Update GUI in main thread only!
        self.subscr_lin_x = msg.linear.x
        self.subscr_ang_z = msg.angular.z
        QTimer.singleShot(0, self.update_gui)

    def update_lin_speed_color(self):
        self.label_current_speeds.setStyleSheet(f"color: {self.default_font_color};")

    def update_ang_speed_color(self):
        self.label_current_ang_speeds.setStyleSheet(f"color: {self.default_font_color};")

    def update_gui(self):
        """Update GUI"""
        lin_x = self.subscr_lin_x
        ang_z = self.subscr_ang_z
        now = self.get_clock().now().nanoseconds / 1e6

        if now < self.last_cmd_vel_update_time + CMD_VEL_UPDATE_INTERVAL:
            if self.is_cmd_vel_connected:
                return

        self.last_cmd_vel_update_time = now

        is_updated = False

        if lin_x != self.current_lin_x and now > self.curr_lin_x_next_change_time:
            self.label_current_speeds.setText(lang_dict[self.cur_lang]['current_speed_val'].format(current_lin_x=to_km_h(lin_x)))
            if (lin_x > 0):
                self.label_current_speeds.setStyleSheet("color: green;")
            else:
                self.label_current_speeds.setStyleSheet("color: red;")

            QTimer.singleShot(1700, self.update_lin_speed_color)
            self.current_lin_x = lin_x
            self.curr_lin_x_next_change_time = now + CMD_VEL_UPDATE_INTERVAL
            is_updated = True

        if  ang_z != self.current_ang_z and now > self.curr_ang_z_next_change_time:
            self.label_current_ang_speeds.setText(lang_dict[self.cur_lang]['current_ang_speed_val'].format(current_ang_z=to_gard_sec(ang_z)))
            if (ang_z > 0):
                self.label_current_ang_speeds.setStyleSheet("color: green;")
            else:
                self.label_current_ang_speeds.setStyleSheet("color: red;")

            QTimer.singleShot(1700, self.update_ang_speed_color)
            self.current_ang_z = ang_z
            self.curr_ang_z_next_change_time = now + CMD_VEL_UPDATE_INTERVAL
            is_updated = True

        if is_updated:
            ang_factor = 0
            lin_vel_factor = 0
            if ang_z > 0:
                ang_factor = 1
            elif ang_z < 0:
                ang_factor = -1

            if lin_x > 0:
                lin_vel_factor = 1
            elif lin_x < 0:
                lin_vel_factor = -1

            direction = directions_by_lin_ang_speed[(lin_vel_factor, ang_factor)]
            self.label_direction_on_topic.setText(direction)


        self.is_cmd_vel_connected = True
        self.update_connection()

        # Reset connection timeout timer
        self.cmd_vel_timeout_timer.start(CMD_VEL_CONNECTION_TIMEOUT)



def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    window = JoyPadGui()
    window.show()
    ros_thread = RosThread(window)
    ros_thread.start()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
