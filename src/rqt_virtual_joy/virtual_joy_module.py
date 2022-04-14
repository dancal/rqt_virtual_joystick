import os
import rospy
import rospkg
from sensor_msgs.msg import Joy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget,QGraphicsView
from python_qt_binding.QtGui import QCursor
from python_qt_binding import QtCore

from math import fabs
from numpy import array_equal

# rosrun rqt_virtual_joy rqt_virtual_joy
class MyPlugin(Plugin):

    def __init__(self, context):

        super(MyPlugin, self).__init__(context)

        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        parser.add_argument("-t", "--topic",
                      dest="topic",
                      type=str,
                      help="Set topic to publish [default:/notspot_joy/joy_ramped]",
                      default="/notspot_joy/joy_ramped")
        parser.add_argument("-r", "--rate",
                      dest="rate",
                      type=float,
                      help="Set publish rate [default:20]",
                      default=20)
        parser.add_argument("--type",
                      dest="type",
                      type=str,
                      choices=['circle', 'square'],
                      default='circle')


        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # target
        self.target_joy = Joy()
        self.target_joy.axes = [0.,0.,1.,0.,0.,1.,0.,0.]
        self.target_joy.buttons = [0,0,0,0,0,0,0,0,0,0,0]

        # last
        self.last_joy = Joy()
        self.last_joy.axes = [0.,0.,1.,0.,0.,1.,0.,0.]
        self.last_joy.buttons = [0,0,0,0,0,0,0,0,0,0,0]
        self.last_send_time = rospy.Time.now()

        self.use_button = True

        self.speed_index = 2
        self.available_speeds = [0.5, 1.0, 3.0, 4.0]

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_virtual_joy'), 'resource', 'VirtualJoy.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.topicLineEdit.returnPressed.connect(self.topicNameUpdated)
        self._widget.topicLineEdit.setText(args.topic)  # Default Topic
        self.updatePublisher()

        self._widget.publishCheckBox.stateChanged.connect(self.publishCheckboxChanged)
        self._widget.rateSpinBox.valueChanged.connect(self.publishRateSpinBoxChanged)
        self._widget.rateSpinBox.setValue(args.rate)

        self._widget.joy.xMoved.connect(self.receiveX)
        self._widget.joy.yMoved.connect(self.receiveY)

        self._widget.shapeSelectBox.addItem("square")
        self._widget.shapeSelectBox.addItem("circle")

        self._widget.shapeSelectBox.activated.connect(self.indexChanged)
        self._widget.shapeSelectBox.setCurrentText(args.type) # circle
        self._widget.joy.setMode(args.type)


    def topicNameUpdated(self):
        self.updatePublisher()


    def updatePublisher(self):
        topic = str(self._widget.topicLineEdit.text())
        try:
            if self.pub != None:
                self.pub.unregister()
        except:
            pass
        self.pub = None
        self.pub = rospy.Publisher(topic, Joy,queue_size=10)

    def startIntervalTimer(self,msec):

        try:
            self._timer.stop()
        except:
            self._timer = QtCore.QTimer(self)
            self._timer.timeout.connect(self.processTimerShot)
        
        if msec > 0:
            self._timer.setInterval(msec)
            self._timer.start()


    def publishCheckboxChanged(self,status):
        self.updateROSPublishState()

    def publishRateSpinBoxChanged(self,status):
        self.updateROSPublishState()
        
    def updateROSPublishState(self):

        if self._widget.publishCheckBox.checkState() == QtCore.Qt.Checked:
            rate = self._widget.rateSpinBox.value()
            self.startIntervalTimer(float(10000.0/rate))
        else:
            self.startIntervalTimer(-1)  # Stop Timer (Stop Publish)

    def indexChanged(self,index):
        text = str(self._widget.shapeSelectBox.currentText())
        self._widget.joy.setMode(str(text))

    def receiveX(self,val):
        self.updateJoyPosLabel()

    def receiveY(self,val):
        self.updateJoyPosLabel()

    def updateJoyPosLabel(self):
        pos = self.getROSJoyValue()
        text = "({:1.2f},{:1.2f})".format(pos['x'],pos['y'])
        self._widget.joyPosLabel.setText(text)

    def ramped_vel(self,v_prev,v_target,t_prev,t_now):
        # This function was originally not written by me:
        # https://github.com/osrf/rosbook/blob/master/teleop_bot/keys_to_twist_with_ramps.py
        step = (t_now - t_prev).to_sec()
        sign = self.available_speeds[self.speed_index] if \
                (v_target > v_prev) else -self.available_speeds[self.speed_index]
        error = fabs(v_target - v_prev)

        # if we can get there within this timestep -> we're done.
        if error < self.available_speeds[self.speed_index]*step:
            return v_target
        else:
            return v_prev + sign * step # take a step toward the target

    def processTimerShot(self):
        t_now           = rospy.Time.now()

        joy             = self.getROSJoyValue()

        msg             = Joy()
        msg.axes        = self.target_joy.axes
        msg.buttons     = self.last_joy.buttons

        msg.header.stamp = rospy.Time.now()
        msg.axes[0]     = float(joy['x'])
        msg.axes[1]     = float(joy['y'])
        #msg.axes[3]     = float(joy['x'])
        #msg.axes[4]     = float(joy['y'])
        #msg.axes.append(float(joy['x']))
        #msg.axes.append(float(joy['y']))
        # joy.axes.append(self.ramped_vel(self.last_joy.axes[i], self.target_joy.axes[i],self.last_send_time,t_now))

        for i in range(len(self.target_joy.axes)):
            i = i + 1
            msg.buttons[i] = int(eval("self._widget.button"+str(i)).isDown() == True)

        #msg.buttons[2]  = True
        self.last_joy.buttons = msg.buttons
        #print("buttons_change = ", msg.buttons)

        try:
            self.pub.publish(msg)
        except:
            rospy.logwarn("publisher not initialized")
            pass

        self.last_send_time = t_now

    def getROSJoyValue(self):
        return self._widget.joy.getJoyValue()
        #return self.convertREPCoordinate(self._widget.joy.getJoyValue())

    def convertREPCoordinate(self,input):
        output = {}
        output['x'] = input['y']
        output['y'] = input['x']
        return output

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.pub.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog