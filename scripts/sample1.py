import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters

class ReconfigurableNode(Node):
    def __init__(self):
        super().__init__('reconfigurable_node')
        
        # Create parameters and initialize with default values
        self.declare_parameter('param_int', 42)
        self.declare_parameter('param_double', 3.14)

        # Create a service to handle dynamic reconfiguration
        self.get_logger().info('Creating reconfigure service...')
        self.reconfigure_service = self.create_service(
            SetParameters, 'set_parameters', self.set_parameters_callback)

    def set_parameters_callback(self, request, response):
        self.get_logger().info('!!!!entered callback')
        for param in request.parameters:
            if param.name == 'param_int':
                if param.value.type == param.value.INTEGER:
                    self.get_logger().info(f'Setting param_int to {param.value.integer_value}')
                    self.set_parameter('param_int', param.value.integer_value)
                else:
                    self.get_logger().warn('Invalid value type for param_int')
            elif param.name == 'param_double':
                if param.value.type == param.value.DOUBLE:
                    self.get_logger().info(f'Setting param_double to {param.value.double_value}')
                    self.set_parameter('param_double', param.value.double_value)
                else:
                    self.get_logger().warn('Invalid value type for param_double')
            else:
                self.get_logger().warn(f'Unknown parameter: {param.name}')
        response.successful = True
        response.reason = 'Parameters set successfully'
        return response

    def update_parameters(self, param_int, param_double):
        # Create a list of parameter changes
        self.get_logger().info('!!!!enternd updated')
        parameters = [
            Parameter(name='param_int', value=ParameterValue(integer_value=param_int)),
            Parameter(name='param_double', value=ParameterValue(double_value=param_double))
        ]

		# Create a SetParameters request
        request = SetParameters.Request()
        request.parameters = parameters

        # Create a client to call the set_parameters service
        client = self.create_client(SetParameters, 'set_parameters')
        self.get_logger().info('!!!!created client')

        if not client.service_is_ready():
            client.wait_for_service()
        self.get_logger().info('!!!!received service')
        future = client.call(request)
        self.get_logger().info('!!!!called using client')

        # Wait for the service call to complete
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            self.get_logger().info('Parameters updated successfully')
        else:
            self.get_logger().error('Failed to update parameters')
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = ReconfigurableNode()
    
    # Example: Call the update_parameters function to change parameter values
    new_param_int = 123
    new_param_double = 2.718
    node.get_logger().info('!!!!CAlling to update parameters')
    result = node.update_parameters(new_param_int, new_param_double)
    if result.successful:
        node.get_logger().info('Parameters updated successfully')
    else:
        node.get_logger().error('Failed to update parameters')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
