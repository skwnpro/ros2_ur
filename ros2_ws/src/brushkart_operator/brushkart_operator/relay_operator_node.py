#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

import RPi.GPIO as GPIO

from brushkart_msgs.msg import RelayOperator

class RelayOperatorNode(Node):

    # ==================================

    # GPIO Mapping

    # ==================================

    RELAY1_BRUSH   = 5

    RELAY2_FORWARD = 6

    RELAY3_RETREAT = 13

    RELAY4_LEFT    = 19

    RELAY5_RIGHT   = 26

    RELAY6_LIGHT   = 16

    # ==================================

    # Ubah False jika relay active-low

    # ==================================

    ACTIVE_HIGH = True

    def __init__(self):

        super().__init__('relay_operator_node')

        GPIO.setmode(GPIO.BCM)

        GPIO.setwarnings(False)

        self.relay_pins = [

            self.RELAY1_BRUSH,

            self.RELAY2_FORWARD,

            self.RELAY3_RETREAT,

            self.RELAY4_LEFT,

            self.RELAY5_RIGHT,

            self.RELAY6_LIGHT

        ]

        for pin in self.relay_pins:

            GPIO.setup(pin, GPIO.OUT)

        self.all_off()

        self.subscription = self.create_subscription(

            RelayOperator,

            '/relay_operator',

            self.relay_callback,

            10

        )

        self.get_logger().info(

            'Relay Operator Node Started'

        )

    # ==================================

    def relay_write(self, pin, state):

        if self.ACTIVE_HIGH:

            GPIO.output(

                pin,

                GPIO.HIGH if state else GPIO.LOW

            )

        else:

            GPIO.output(

                pin,

                GPIO.LOW if state else GPIO.HIGH

            )

    # ==================================

    def all_off(self):

        for pin in self.relay_pins:

            self.relay_write(pin, False)

    # ==================================

    def relay_callback(self, msg):

        # E-STOP PRIORITAS TERTINGGI

        if msg.estop:

            self.all_off()

            self.get_logger().warn(

                'EMERGENCY STOP'

            )

            return

        self.relay_write(

            self.RELAY1_BRUSH,

            msg.relay1_brush

        )

        self.relay_write(

            self.RELAY2_FORWARD,

            msg.relay2_forward

        )

        self.relay_write(

            self.RELAY3_RETREAT,

            msg.relay3_retreat

        )

        self.relay_write(

            self.RELAY4_LEFT,

            msg.relay4_left

        )

        self.relay_write(

            self.RELAY5_RIGHT,

            msg.relay5_right

        )

        self.relay_write(

            self.RELAY6_LIGHT,

            msg.relay6_light

        )

    # ==================================

    def destroy_node(self):

        self.all_off()

        GPIO.cleanup()

        super().destroy_node()

def main(args=None):

    rclpy.init(args=args)

    node = RelayOperatorNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':

    main()